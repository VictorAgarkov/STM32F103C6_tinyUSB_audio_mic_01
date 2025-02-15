/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   manufacturer: STMicroelectronics
*/

#include "stm32f1xx_hal.h"
#include "bsp/board_api.h"
#include "gpio_F1xx.h"
#include "goods.h"

//#include "board.h"

void user_systick_handler(uint32_t tick_count);

//--------------------------------------------------------------------+


struct s_PinDef g_PinsUsb[] =
{
	#ifdef USB_CONNECT_PIN
		{GPIOA, 15,  GPIO_Speed_10MHz | GPIO_Mode_OUT_PP,  0},  // USB pullup
	#endif
//	{GPIOA, 11,  GPIO_Speed_50MHz | GPIO_Mode_AF_PP,  0},  // USB
//	{GPIOA, 12,  GPIO_Speed_50MHz | GPIO_Mode_AF_PP,  0},  // USB
	{GPIOA, 11,  GPIO_Speed_input | GPIO_Mode_IN_FLOAT,  0},  // USB
	{GPIOA, 12,  GPIO_Speed_input | GPIO_Mode_IN_FLOAT,  0},  // USB
};

struct s_PinDef g_PinsOut[] =
{
	{GPIOC, 13,  GPIO_Speed_10MHz | GPIO_Mode_OUT_PP,  1},  // LED
};


// all pins
const struct s_PinDefSet all_pins[] =
{
    GPIO_ARRAY(g_PinsUsb),
    GPIO_ARRAY(g_PinsOut),
};

#define pin_Led g_PinsOut[0]

#ifdef USB_CONNECT_PIN
	#define pin_UsbPullup g_PinsUsb[0]
#endif

//--------------------------------------------------------------------+


void init_all_gpios(void)
{
    for(int a = 0; a < NUMOFARRAY(all_pins); a++)
    {
        const struct s_PinDefSet *set = all_pins + a;
        for(int i = 0; i < set->num; i++)
        {
            const struct s_PinDef *p = set->pindef + i;

            int bit_pos = p->pin + (p->init_val ? 0 : 16);
            p->port->BSRR = 1 << bit_pos;
            GPIO_SET_MODEp(p, p->mod);
        }
    }
}



//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void board_stm32f1_clock_init(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  RCC_PeriphCLKInitTypeDef rccperiphclkinit = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  HAL_RCC_OscConfig(&oscinitstruct);

  /* USB clock selection */
  rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  rccperiphclkinit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2);
}




//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
uint32_t USB_HP_IRQ_cnt = 0;
uint32_t USB_LP_IRQ_cnt = 0;
uint32_t USBWakeUp_IRQ_cnt = 0;

void USB_HP_IRQHandler(void)
{
    tud_int_handler(0);
    USB_HP_IRQ_cnt++;
}

void USB_LP_IRQHandler(void)
{
    tud_int_handler(0);
    USB_LP_IRQ_cnt++;
}

void USBWakeUp_IRQHandler(void)
{
    tud_int_handler(0);
    USBWakeUp_IRQ_cnt++;
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
UART_HandleTypeDef UartHandle;

void board_init(void)
{

	board_stm32f1_clock_init();

    RCC->APB1ENR |= 0;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN;
    AFIO->MAPR   |= AFIO_MAPR_SWJ_CFG_1;        // disable jtag - make A15 as GPIO


	init_all_gpios();

	#if CFG_TUSB_OS == OPT_OS_NONE
		// 1ms tick timer
		SysTick_Config(SystemCoreClock / 1000);

	#endif
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

}
//--------------------------------------------------------------------+

#ifdef USB_CONNECT_PIN
	void dcd_disconnect(uint8_t rhport)
	{
		(void)rhport;
		GPIO_OUT_VAL(pin_UsbPullup, 0);
	}

	void dcd_connect(uint8_t rhport)
	{
		(void)rhport;
		//GPIO_OUT_VAL(pin_UsbPullup, 1);
	}
#endif
//--------------------------------------------------------------------+

void board_init_after_tusb(void)
{
	GPIO_OUT_VAL(pin_UsbPullup, 1);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	GPIO_OUT_VAL(pin_Led, !state);
}
//--------------------------------------------------------------------+

uint32_t board_button_read(void)
{
//    return BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
    return 0;
}
//--------------------------------------------------------------------+

//size_t board_get_unique_id(uint8_t id[], size_t max_len)
//{
//    (void) max_len;
//    volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
//    uint32_t* id32 = (uint32_t*) (uintptr_t) id;
//    uint8_t const len = 12;
//
//    id32[0] = stm32_uuid[0];
//    id32[1] = stm32_uuid[1];
//    id32[2] = stm32_uuid[2];
//
//    return len;
//}

//int board_uart_read(uint8_t *buf, int len)
//{
//    (void) buf;
//    (void) len;
//    return 0;
//}

//int board_uart_write(void const *buf, int len)
//{
//    HAL_UART_Transmit(&UartHandle, (uint8_t *) (uintptr_t) buf, len, 0xffff);
//    return len;
//}
//--------------------------------------------------------------------+
#if CFG_TUSB_OS == OPT_OS_NONE
	volatile uint32_t system_ticks = 0;

	void SysTick_Handler(void)
	{
		HAL_IncTick();
		system_ticks++;
		user_systick_handler(system_ticks);

	}

	uint32_t board_millis(void)
	{
		return system_ticks;
	}

#endif

//--------------------------------------------------------------------+

void HardFault_Handler(void)
{
    __asm("BKPT #0\n");
}
//--------------------------------------------------------------------+

#ifdef  USE_FULL_ASSERT
	void assert_failed(const char *file, uint32_t line)
	{
		/* USER CODE BEGIN 6 */
		/* User can add his own implementation to report the file name and line number,
		   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
		/* USER CODE END 6 */
	}
#endif /* USE_FULL_ASSERT */

//// Required by __libc_init_array in startup code if we are compiling using
//// -nostdlib/-nostartfiles.
//void _init(void)
//{
//}
