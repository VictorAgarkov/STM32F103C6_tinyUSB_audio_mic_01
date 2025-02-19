#ifndef ADC_RUN_H_INCLUDED
	#define ADC_RUN_H_INCLUDED

	#include "tusb.h"

	#include "goods.h"

	#define ADC_CHN_NUM min(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, 10)

	void adc_init(void);
	int  adc_get_actual_sample_rate(void);
	void adc_set_buff_idx(unsigned int idx);

	extern unsigned int  g_AdvAvgBuffIdx;
	extern unsigned int  g_AdcAvgBuffCnt[2];
	extern int16_t       g_AdcAvgBuff[2][CFG_TUD_AUDIO_EP_SZ_IN / 2];
	extern uint32_t      g_AdcCtrl;


#endif /* ADC_RUN_H_INCLUDED */
