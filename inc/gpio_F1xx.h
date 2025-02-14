#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED


	#include "stm32f1xx.h"
	#include <stdint.h>

	// структура - описатель вывода GPIO
	struct s_PinDef
	{
		GPIO_TypeDef *port;           // GPIO
		uint8_t       pin;            // pin number
		uint8_t       mod;
		uint8_t       init_val;
	};

	struct s_PinDefSet
	{
		const struct s_PinDef *pindef;
		int                    num;
	};


	#define GPIO_ARRAY(a) {a, NUMOFARRAY(a)}


	enum
	{
		GPIO_Speed_input = 0,
		GPIO_Speed_10MHz = 1,
		GPIO_Speed_2MHz  = 2,
		GPIO_Speed_50MHz = 3
	};


	enum
	{
		// input modes
		GPIO_Mode_IN_ANALOG = 0x0,
		GPIO_Mode_IN_FLOAT = 0x04,
		GPIO_Mode_IN_PULL = 0x08,

		// output modes
		GPIO_Mode_OUT_PP = 0x0,
		GPIO_Mode_OUT_OD = 0x4,
		GPIO_Mode_AF_PP = 0x8,
		GPIO_Mode_AF_OD = 0xC
	};


	// макросы для работы с GPIO на STM32F1xx
	#define GPIO_RG2(reg, pin, val) reg = ((reg & ~((uint32_t)3  << ((pin) * 2))) | ((uint32_t)((val) & 3)  << ((pin) * 2)))
	#define GPIO_RG4(reg, pin, val) reg = ((reg & ~((uint32_t)15 << ((pin) * 4))) | ((uint32_t)((val) & 15) << ((pin) * 4)))

	#define GPIO_MODE(gpio, pin, mode) ((pin) < 8) ? (GPIO_RG4(gpio->CRL, pin, mode)) : (GPIO_RG4(gpio->CRH, (pin) & 7, mode))
	#define GPIO_AF(gpio, pin, af) ((pin) < 8) ? (GPIO_RG4(gpio->AFR[0], pin, af)) : (GPIO_RG4(gpio->AFR[1], (pin) & 7, af))
	#define GPIO_BSRR(pin, val)	1 << ((pin) + ((val) ? 0  : 16))
	#define GPIO_OUT_V(port, pin, val) port->BSRR = GPIO_BSRR(pin, val)

	//#define GPIO_PUPD(gpio, pin, mode) GPIO_RG2(gpio->PUPDR, pin, mode)

	#define GPIO_OUT_VAL(pd, val)  GPIO_OUT_V((pd).port,  (pd).pin, val)  // (pd).port->BSRR = 1 << ((pd).pin + ((val) ? 0  : 16))
	#define GPIO_OUT_VALp(pd, val) GPIO_OUT_V((pd)->port, (pd)->pin, val) // (pd)->port->BSRR = 1 << ((pd)->pin + ((val) ? 0  : 16))

	//#define GPIO_SET_ACTIVE(pd)   GPIO_OUT_VAL((pd),  (pd).active_value)
	//#define GPIO_SET_PASSIVE(pd)  GPIO_OUT_VAL((pd), !(pd).active_value)

	//#define GPIO_SET_ACTIVEp(pd)   (pd)->port->BSRR = 1 << ((pd)->pin + ((pd)->active_value ? 0  : 16))
	//#define GPIO_SET_PASSIVEp(pd)  (pd)->port->BSRR = 1 << ((pd)->pin + ((pd)->active_value ? 16 :  0))

	#define GPIO_SET_MODE(pd, mode) GPIO_MODE((pd).port, (pd).pin, (mode))
	#define GPIO_SET_MODEp(pd, mode) GPIO_MODE((pd)->port, (pd)->pin, (mode))
	//#define GPIO_SET_PUPD(pd, mode) GPIO_PUPD((pd).port, (pd).pin, (mode))
	#define GPIO_SET_AF(pd, af)     GPIO_AF  ((pd).port, (pd).pin, (af))


	#define GPIO_GET_PIN(pd) (((pd).port->IDR >> (pd).pin) & 1)

	#define GPIO_OUT_BSRR(gpio, val)   gpio->BSRR = (val)

	// макросы для работы с масками управляющих регистров
	// перед инициализацией создаётся два массива uint32_t[2], один для масок, второй для значений
	// затем они инициализируются всеми пинами порта, и в конце обновляются значения в регистрах GPIO
	#define GPIO_MODE_MASK4(mask, pin, mode) mask[pin >> 3] = (mask[pin >> 3] &  ~((uint32_t)15 << (((pin) & 7) * 4))) | ((uint32_t)((mode) & 15) << (((pin) & 7) * 4))
	#define GPIO_MODE_MASK4_HL(maskh, maskl, pin, mode) \
	        if(pin < 8) maskl = (maskl &  ~((uint32_t)15 << (((pin) & 7) * 4))) | ((uint32_t)((mode) & 15) << (((pin) & 7) * 4)); \
	        else        maskh = (maskh &  ~((uint32_t)15 << (((pin) & 7) * 4))) | ((uint32_t)((mode) & 15) << (((pin) & 7) * 4));
	#define REG_MASK_VAL(reg, mask, val) if(mask) reg = (reg & ~(mask)) | ((val) & (mask))




#endif /* GPIO_H_INCLUDED */
