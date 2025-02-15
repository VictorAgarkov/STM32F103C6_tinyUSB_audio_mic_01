#ifndef _LED_BLINK_H
	#define _LED_BLINK_H

	#include <stdint.h>

	typedef struct
	{
		const uint8_t  num;
		const uint8_t *set;
	}LedBlinkSet_t;

	/*
	Поле set структуры LedBlinkSet_t - набор задержек, в течении которых светик горит или не горит.
	Чётные задержки (0,2,4...) - в течении которых светик горит
	Нечётные задержки (1,3,5...) - в течении которых светик потушен
	Значения:
	0  - переход к следующей задержке без изменения состояния светика
	-1 - вечная задержка - постоянное состояние светика до прихода какого-то внешнего события

	*/

	typedef struct
	{
		int     idx;       // Номер паттерна в LedBlinkSet_t
		int     offs;      // Смещение внутри паттерна
		uint8_t time;      // текущее время от начала
		//int     curr_state;
	}LedBlinkState_t;


	#define LED_BLINK_ITEM(a) {NUMOFARRAY(a), a}

	#define LED_BLINK_SET(set,_idx) (set).idx = _idx; (set).offs = 0; (set).time = 0

#endif // _LED_BLINK_H
