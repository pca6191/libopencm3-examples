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

#define USE429 1

#define SWUART_BAUD_RATE          9600//115200 //note: 跑 HSE 16M , TX 最高 38400 bps
#define SWUART_TX_CLOCK           RCC_GPIOA    //bit value to set TX clock
#if USE429
#define SWUART_TX_PORT            GPIOB        //the port TX assigned to
#else
#define SWUART_TX_PORT            GPIOA        //the port TX assigned to
#endif
#define SWUART_TX_PIN             GPIO4        //the pin TX assigned to

#define SWUART_RX_CLOCK           RCC_GPIOA    //bit value to set RX clock
#define SWUART_RX_PORT            GPIOA        //the port RX assigned to
#define SWUART_RX_PIN             GPIO5        //the pin RX assigned to
#define SWUART_RX_NVIC_EXTI_IRQ   NVIC_EXTI9_5_IRQ  //the IRQ value hadling RX EXTI
#define SWUART_RX_EXTI            EXTI5             //the EXTI RX falling will trigger
#define SWUART_RX_EXTI_ISR        exti9_5_isr       //the ISR hadling RX EXTI

#if USE429
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
#else
/* standard clocking for F2 boards */
static const struct rcc_clock_scale clock_setup =
{
        .pllm = 24,  //外頻 24 MHz 振盪器, 以下欲得到 120M 內頻
        .plln = 240,
        .pllp = 2,
        .pllq = 5,
        .hpre = RCC_CFGR_HPRE_DIV_NONE,
        .ppre1 = RCC_CFGR_PPRE_DIV_4,
        .ppre2 = RCC_CFGR_PPRE_DIV_2,
        .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_5WS,
        .apb1_frequency = 30000000,
        .apb2_frequency = 60000000,
};
#endif

/*
 * @brief    Set the GPIO used of this module.
 */
static void gpio_setup(void)
{
    /**** init TX as output (ESC Err pin, PA4) *****/
    // Enable TX pin clock.
    rcc_periph_clock_enable(SWUART_TX_CLOCK);
    // Set TX pin to 'output pull up'.
    gpio_mode_setup(SWUART_TX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
            SWUART_TX_PIN);
    //set TX idle (High)
    gpio_set(SWUART_TX_PORT, SWUART_TX_PIN);

    /**** init RX as input (CH3 pin, PA5) *****/
    // Enable GPIOC clock.
    rcc_periph_clock_enable(SWUART_RX_CLOCK);
    // Set RX pin to 'input no pull'.
    gpio_mode_setup(SWUART_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            SWUART_RX_PIN);

#if USE429
    rcc_periph_clock_enable(RCC_GPIOG);
    /* Set GPIO13 (in GPIO port G) to 'output push-pull'. */
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    gpio_clear(GPIOG, GPIO13);
#endif
}

/*
 * @brief    Initialize the cycle counter / EXTI / TIMER for TX / RX.
 */
static void swuart_init(void)
{
    /**** init for TX *****/
    //enable DWT counter for measure delay time precisely.
    dwt_enable_cycle_counter();

    /**** init RX EXTI (CH3 pin, PA5) *****/
    // Enable RX pin as EXTI (external interrut)
    nvic_enable_irq(SWUART_RX_NVIC_EXTI_IRQ);
    exti_select_source(SWUART_RX_EXTI, SWUART_RX_PIN);
    exti_set_trigger(SWUART_RX_EXTI, EXTI_TRIGGER_FALLING);
    exti_enable_request(SWUART_RX_EXTI);

    /**** initial Timer2 for RX*****/
    // Enable TIM2 clock.
    rcc_periph_clock_enable(RCC_TIM2);
    //Enable TIM2 interrupt.
    nvic_enable_irq(NVIC_TIM2_IRQ);
    // Reset TIM2 peripheral to defaults.
    rcc_periph_reset_pulse(RST_TIM2);
    /* Timer global mode:
     * - No divider / Alignment edge / Direction up
     */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // 不降頻
#if USE429
    timer_set_prescaler(TIM2, 0);//SWUART_BAUD_RATE - 1);
#else
    timer_set_prescaler(TIM2, 0);
#endif

    //算滿半個 bit 時間就中斷 (stm32f205 實證：Timer2 基頻 = CPU_FREP / 2 = 60 MHz)
    timer_set_period(TIM2,
            ((clock_setup.apb1_frequency) / (2 * SWUART_BAUD_RATE)) - 1);

    /* Counter & IRQ enable. */
    timer_enable_irq(TIM2, TIM_DIER_UIE);
#if USE429
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);
#endif
}

#define SWUART_BITBUF_SIZE 20
static uint8_t bitbuf[SWUART_BITBUF_SIZE];
static uint8_t bit_index = 0;
static uint8_t byte_tmp = 0;
void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
#if USE429
        gpio_toggle(GPIOG, GPIO13);
#endif
        timer_clear_flag(TIM2, TIM_SR_UIF);
        timer_set_counter(TIM2, 0);
        bitbuf[bit_index] = gpio_get(SWUART_RX_PORT, SWUART_RX_PIN);
        bit_index++;

        if (bit_index == 18) //stop bit 1st sampling
        {
            //restart to wait for falling edge of start bit.
            timer_disable_counter(TIM2);
            exti_enable_request(SWUART_RX_EXTI);
            bit_index = 0;

            //merge bit to byte
            byte_tmp |= (bitbuf[3] != 0) ? 0x01 : 0; // bit 0
            byte_tmp |= (bitbuf[5] != 0) ? 0x02 : 0; // bit 1
            byte_tmp |= (bitbuf[7] != 0) ? 0x04 : 0; // bit 2
            byte_tmp |= (bitbuf[9] != 0) ? 0x08 : 0; // bit 3
            byte_tmp |= (bitbuf[11] != 0) ? 0x10 : 0; // bit 4
            byte_tmp |= (bitbuf[13] != 0) ? 0x20 : 0; // bit 5
            byte_tmp |= (bitbuf[15] != 0) ? 0x40 : 0; // bit 6
            byte_tmp |= (bitbuf[17] != 0) ? 0x80 : 0; // bit 7

            //todo push to queue
        }
    }
}

/*
 * @brief    當 RX 首發生 falling edge，觸發本中斷，表示 start bit 開始。
 *           接著應停止本中斷，啟動 timer 中斷，每半個 bit 取樣一次。
 *           當 timer 取樣到 stop bit，再開啟本中斷，依此循環。
 */
void SWUART_RX_EXTI_ISR(void)
{
    //關閉自身，交棒給 timer
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);
    exti_disable_request(SWUART_RX_EXTI);
    //變數歸零
    bit_index = 0;
    byte_tmp = 0;
}


/*
 * @brief    以 blocking 方式，發送 1 byte.
 * @param    待發送的 byte value.
 */
static void swuart_send_byte(uint8_t b)
{
    //count the bit time to cyc_counts
    uint32_t bitcnt = 4*(rcc_apb1_frequency / SWUART_BAUD_RATE);

    //send start bit
    DWT_CYCCNT = 0;
    gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 0
    DWT_CYCCNT = 0;
    ((b >> 0) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 1
    DWT_CYCCNT = 0;
    ((b >> 1) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 2
    DWT_CYCCNT = 0;
    ((b >> 2) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 3
    DWT_CYCCNT = 0;
    ((b >> 3) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 4
    DWT_CYCCNT = 0;
    ((b >> 4) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 5
    DWT_CYCCNT = 0;
    ((b >> 5) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 6
    DWT_CYCCNT = 0;
    ((b >> 6) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send bit 7
    DWT_CYCCNT = 0;
    ((b >> 7) & 0x01) ?
            gpio_set(SWUART_TX_PORT, SWUART_TX_PIN) : gpio_clear(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }

    //send stop bit
    DWT_CYCCNT = 0;
    gpio_set(SWUART_TX_PORT, SWUART_TX_PIN);
    for(; DWT_CYCCNT < bitcnt;) { }
}

/*
 * @brief    主程式。
 */
int main(void)
{
    uint16_t i;

    //設置外頻，使得內頻為 168 MHz.
    rcc_clock_setup_hse_3v3(&clock_setup);

    //設置硬體 GPIO / EXTI / TIMER
    gpio_setup();
    swuart_init();

    //gpio_set(GPIOG, GPIO13);

    for(i = 0;; i++)
    {

//        uint32_t cnt;
//        gpio_set(GPIOG, GPIO13); //LED green
//
//        for(int j = 0; j < 500; j++)
//        {
//            DWT_CYCCNT = 0;
//            cnt = DWT_CYCCNT;
//            for(; cnt < (1024 * 168);)
//            {
//                cnt = DWT_CYCCNT;
//            } //1 ms
//        }
//
//        gpio_clear(GPIOG, GPIO13); //LED green
//
//        for(int j = 0; j < 500; j++)
//        {
//            DWT_CYCCNT = 0;
//            cnt = DWT_CYCCNT;
//            for(; cnt < (1024 * 168);)
//            {
//                cnt = DWT_CYCCNT;
//            } //1 ms
//        }
    }
    swuart_send_byte((uint8_t)(i % 256));

    return 0;
}
