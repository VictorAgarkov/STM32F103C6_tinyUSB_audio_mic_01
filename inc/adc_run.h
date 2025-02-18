#ifndef ADC_RUN_H_INCLUDED
	#define ADC_RUN_H_INCLUDED

	#include "tusb.h"

	#include "goods.h"

	#define ADC_CHN_NUM min(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, 10)

	void adc_init(void);

	extern int      g_AdcAvgBuffCnt;
	extern uint16_t g_AdcAvgBuff[CFG_TUD_AUDIO_EP_SZ_IN / 2];


#endif /* ADC_RUN_H_INCLUDED */
