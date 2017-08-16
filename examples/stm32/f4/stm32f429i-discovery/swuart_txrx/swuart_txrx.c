/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/exti.h>

#define DBG 0      //用於開啟某些 GPIO 來 debug
#if DBG
#define LED_PORT              GPIOG
#define LED_GREEN_PIN         GPIO13
#define LED_RED_PIN           GPIO14
#endif

#define SWUART_BAUD_RATE          9600//115200 //note: 跑 HSE 16M , TX 最高 38400 bps
#define SWUART_TX_CLOCK           RCC_GPIOB    //KC_DBG for 429; RCC_GPIOA    //bit value to set TX clock
#define SWUART_TX_PORT            GPIOB        //the port TX assigned to
#define SWUART_TX_PIN             GPIO4        //the pin TX assigned to

#define SWUART_RX_CLOCK           RCC_GPIOA    //bit value to set RX clock
#define SWUART_RX_PORT            GPIOA        //the port RX assigned to
#define SWUART_RX_PIN             GPIO5        //the pin RX assigned to
#define SWUART_RX_NVIC_EXTI_IRQ   NVIC_EXTI9_5_IRQ  //the IRQ value hadling RX EXTI
#define SWUART_RX_EXTI            EXTI5             //the EXTI RX falling will trigger
#define SWUART_RX_EXTI_ISR        exti9_5_isr       //the ISR hadling RX EXTI

static void delay_ms(uint32_t ms);
static void swuart_send_byte(uint8_t b);
void swuart_buf_put(uint8_t b);
int16_t swuart_buf_get(void);

static const struct rcc_clock_scale clock_setup =
{ /* 168MHz */
    .pllm = 8,
    .plln = 336,
    .pllp = 2,
    .pllq = 7,
    .pllr = 0,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
            FLASH_ACR_LATENCY_5WS,
    .ahb_frequency  = 168000000,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
};

/*
 * @brief    Set the GPIO used of this module.
 */
static void gpio_setup(void)
{
    /**** 初始化 TX 作為輸出 (ESC Err pin, PA4) *****/
    // 設定 TX 時脈
    rcc_periph_clock_enable(SWUART_TX_CLOCK);
    // 設定 TX 輸出上拉
    gpio_mode_setup(SWUART_TX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
    SWUART_TX_PIN);
    // TX 一開始輸出 high
    gpio_set(SWUART_TX_PORT, SWUART_TX_PIN);

    /**** 初始化 RX 作為輸入 (CH3 pin, PA5) *****/
    // 設定 RX 時脈
    rcc_periph_clock_enable(SWUART_RX_CLOCK);
    // 設定 RX 為輸入
    gpio_mode_setup(SWUART_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, //todo KC_DBG
    SWUART_RX_PIN);

#if DBG
    rcc_periph_clock_enable(RCC_GPIOG);
    /* Set GPIO13 (in GPIO port G) to 'output push-pull'. */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GREEN_PIN | LED_RED_PIN);
    gpio_clear(LED_PORT, LED_GREEN_PIN);
    gpio_clear(LED_PORT, LED_RED_PIN);
#endif
}

/*
 * @brief    Initialize the cycle counter / EXTI / TIMER for TX / RX.
 */
static void swuart_init(void)
{
    // 啟用 DWT counter 作為精準 delay 計數
    dwt_enable_cycle_counter();

    // RX_EXTI (ex. EXTI5) 選用作動的 port
    exti_select_source(SWUART_RX_EXTI, SWUART_RX_PORT);
    // 指定下降緣觸發
    exti_set_trigger(SWUART_RX_EXTI, EXTI_TRIGGER_FALLING);
    // 打開 RX_EXTI 中斷開關 & 觸發事件
    exti_enable_request(SWUART_RX_EXTI);
    // 啟用 RX GPIO 巢狀中斷 (external interrut)
    nvic_enable_irq(SWUART_RX_NVIC_EXTI_IRQ);

    // 啟動 Timer2 時脈 (定時中斷用來取樣 RX)
    rcc_periph_clock_enable(RCC_TIM2);
    // Reset Timer2 相關週邊
    rcc_periph_reset_pulse(RST_TIM2);
    // 啟用 Timer2 巢狀中斷
    nvic_enable_irq(NVIC_TIM2_IRQ);
    // 設定 Timer2 計數模式 (方向)
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // 不降頻
    timer_set_prescaler(TIM2, 0);
    // 計數滿半個 bit 時間就中斷
    //** note: STM32f429 timer2 受到 APB1 推動，每一個 H-L clock 推動 2 次 (count).
    //** note: 300, 為進入 timer 中斷的 overhead cyc clock 數，因 compile 優化或是 baudrate 而要特調
    timer_set_period(TIM2, ((2*clock_setup.apb1_frequency) / (2*SWUART_BAUD_RATE)) - 1 - 300);
    //打開 Timer2 更新中斷開關
    timer_enable_irq(TIM2, TIM_DIER_UIE);

#if DBG
//    // Timer2 counter 歸零
//   timer_set_counter(TIM2, 0);
//   // Timer2 開始計數
//   timer_enable_counter(TIM2);
#endif
}

#define SWUART_BITBUF_SIZE 20
static uint8_t bitbuf[SWUART_BITBUF_SIZE];
static uint8_t bit_index = 0;  //標記 bitbuf[] 存取位置
static uint8_t byte_tmp = 0;   //將 bit 併為 byte 存放處
void tim2_isr(void)
{
    //發生 update (overflow) 事件
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        //清除中斷事件旗標，避免持續觸發
        timer_clear_flag(TIM2, TIM_SR_UIF);
        //counter 歸零，重新計數
        timer_set_counter(TIM2, 0);
        //取樣 RX 位準
        bitbuf[bit_index] = gpio_get(SWUART_RX_PORT, SWUART_RX_PIN);

        if (bit_index == 19) //stop bit 1st sampling
        {
            //停止 counter 計數
            timer_disable_counter(TIM2);

            //合併 bit 成為 byte
            byte_tmp = 0;
            byte_tmp |= (bitbuf[3] != 0) ? 0x01 : 0; // bit 0
            byte_tmp |= (bitbuf[5] != 0) ? 0x02 : 0; // bit 1
            byte_tmp |= (bitbuf[7] != 0) ? 0x04 : 0; // bit 2
            byte_tmp |= (bitbuf[9] != 0) ? 0x08 : 0; // bit 3
            byte_tmp |= (bitbuf[11] != 0) ? 0x10 : 0; // bit 4
            byte_tmp |= (bitbuf[13] != 0) ? 0x20 : 0; // bit 5
            byte_tmp |= (bitbuf[15] != 0) ? 0x40 : 0; // bit 6
            byte_tmp |= (bitbuf[17] != 0) ? 0x80 : 0; // bit 7
            // bit 位置標記歸零
            bit_index = 0;

            //啟動下一次 RX 下降緣中斷
            exti_enable_request(SWUART_RX_EXTI);

            //todo push to queue
            swuart_buf_put(byte_tmp);
        }
        //指向下一位
        bit_index++;
    }
}

/*
 * @brief    當 RX 首發生 falling edge，觸發本中斷，表示 start bit 開始。
 *           接著應停止本中斷，啟動 timer 中斷，每半個 bit 取樣一次。
 *           當 timer 取樣到 stop bit，再開啟本中斷，依此循環。
 */
void SWUART_RX_EXTI_ISR(void)
{
    //關閉 EXTI 自身巢狀中斷，交棒給 timer
    exti_reset_request(SWUART_RX_EXTI);
    exti_disable_request(SWUART_RX_EXTI);

    //變數歸零
    bit_index = 0;
    byte_tmp = 0;

    //counter 歸零
    timer_set_counter(TIM2, 0);
    //timer 開始計數
    timer_enable_counter(TIM2);
}


/*
 * @brief    以 blocking 方式，發送 1 byte.
 * @param    待發送的 byte value.
 */
static void swuart_send_byte(uint8_t b)
{
    //count the bit time to cyc_counts
    //uint32_t bitcnt = 4*(rcc_apb1_frequency / SWUART_BAUD_RATE);
    uint32_t bitcnt = (clock_setup.ahb_frequency / SWUART_BAUD_RATE);

    //send start bit
    DWT_CYCCNT = 0;
    gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 0
    DWT_CYCCNT = 0;
    ((b >> 0) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 1
    DWT_CYCCNT = 0;
    ((b >> 1) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 2
    DWT_CYCCNT = 0;
    ((b >> 2) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 3
    DWT_CYCCNT = 0;
    ((b >> 3) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 4
    DWT_CYCCNT = 0;
    ((b >> 4) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 5
    DWT_CYCCNT = 0;
    ((b >> 5) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 6
    DWT_CYCCNT = 0;
    ((b >> 6) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 7
    DWT_CYCCNT = 0;
    ((b >> 7) & 0x01) ? gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send stop bit
    DWT_CYCCNT = 0;
    gpio_set(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }
}

/*
 * @brief    使用最基本的 DWT cycle clock 來計數 delay 時間。
 *           ** note: 使用前，要先設定一次 dwt_enable_cycle_counter();
 */
static void delay_ms(uint32_t ms)
{
    for(; ms > 0; ms--)
    {
        //for stm32f429@168 MHz, offset 500，ms = 10，示波器量測很精準
        DWT_CYCCNT = 500;
        for(; DWT_CYCCNT < clock_setup.ahb_frequency/1000;)
        { } //1 ms
    }
}

#define RXBUF_SIZE    256
static unsigned rx_head = 0, rx_tail = 0;
static uint8_t rx_buf[RXBUF_SIZE];

/*
 * @brief   將 1 個 byte, b, 放入 buffer 暫存。
 */
void swuart_buf_put(uint8_t b)
{
    uint16_t next = (rx_head + 1) % RXBUF_SIZE;

    //放到 head 上，next 為 head 下個推進點。
    //判斷確保下次推進不會蓋到 tail.==> head tail 倒追最接近時，之間仍留一個空位
    if (next != rx_tail)
    {
        rx_buf[rx_head] = b;
        rx_head = next;
    }
}

/*
 * @brief    從 buffer 中取出 1 byte.
 * @return   取不到東西，就 return -1.
 */
int16_t swuart_buf_get(void)
{
    int16_t ret = -1;

    if (rx_tail != rx_head)
    {
        ret = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RXBUF_SIZE;
    }
    return ret;
}

/*
 * @brief    主程式。
 */
#define TEST_TX_GPIO        0    //測試 TX H/L 變化
#define TEST_TX_TRANSMITION 0    //測試 TX 傳送 0 ~ 255
#define TEST_RX_ECHO        0    //測試 RX 收到 1 byte 且透過 TX echo
#define TEST_RING_BUFFER    1    //測試中斷取樣匯入 buffer, main loop echo 印出


#if TEST_TX_GPIO //TX output only
int main(void)
{
    uint16_t i;

    //設置外頻，使得內頻為 168 MHz.
    rcc_clock_setup_hse_3v3(&clock_setup);
    dwt_enable_cycle_counter();

    //設置硬體 GPIO
    gpio_setup();

    for(i = 0;; i++)
    {
        delay_ms(10);
        gpio_toggle(SWUART_TX_PORT, SWUART_TX_PIN);
    }

    if (0) //假裝程式有用到
    {
        //設置硬體 EXTI / TIMER
        swuart_init();
        swuart_send_byte((uint8_t)(i % 256));
    }

    return 0;
}
#endif

#if TEST_TX_TRANSMITION
int main(void)
{
    uint16_t i;

    //設置外頻，使得內頻為 168 MHz.
    rcc_clock_setup_hse_3v3(&clock_setup);

    //設置硬體 GPIO
    gpio_setup();

    //設置 software uart 用到的 timer/exti
    swuart_init();

    for(i = 0;; i++)
    {
        gpio_set(SWUART_TX_PORT, SWUART_TX_PIN);
        delay_ms(1);
        swuart_send_byte((uint8_t)(i % 256));
    }

    return 0;
}
#endif

#if TEST_RX_ECHO
int main(void)
{
    uint16_t i;

    //設置外頻，使得內頻為 168 MHz.
    rcc_clock_setup_hse_3v3(&clock_setup);

    //設置硬體 GPIO
    gpio_setup();

    //設置 software uart 用到的 timer/exti
    swuart_init();

    //非 0 即 echo
    for(i = 0;; i++)
    {
        if(byte_tmp != 0)
        {
            swuart_send_byte(byte_tmp);
            byte_tmp = 0;
        }
    }

    return 0;
}
#endif

#if TEST_RING_BUFFER
int main(void)
{
    uint16_t i;

    //設置外頻，使得內頻為 168 MHz.
    rcc_clock_setup_hse_3v3(&clock_setup);

    //設置硬體 GPIO
    gpio_setup();

    //設置 software uart 用到的 timer/exti
    swuart_init();

    //非 0 即 echo
    for(i = 0;; i++)
    {
        int16_t b;

        b = swuart_buf_get();
        if(b >= 0)
        {
            swuart_send_byte((uint8_t)b);
        }
    }

    return 0;
}
#endif
