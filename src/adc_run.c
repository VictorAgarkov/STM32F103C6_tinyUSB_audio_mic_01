/*
	АЦП запускается от TIM3 с частотой 96кГц
	Оцифровывается нужно количество каналов
	По готовности отсчёты поканально суммируются. По мере накопления с частотой дискретизации USB-audio
	усреднённые отсчёты помещаются в выходной буфер

*/


#include "stm32f1xx_hal.h"
#include "adc_run.h"
#include "tusb_config.h"

uint16_t g_AdcDmaBuff[16];

uint32_t g_AdcAcc[10];
int      g_AdcAccCnt;


uint16_t g_AdcAvgBuff[CFG_TUD_AUDIO_EP_SZ_IN / 2];
int      g_AdcAvgBuffCnt = 0;

//// зависимость частоты дискредизации АЦП от кол-ва каналов
//// когда АЦП успевает всё оцифровать
//// указано для Fadc=9MHz, ADC_SMPR=1
//// и перемежением измерений при кол-ве каналов от 1 до 8,
//// без перемежения при кол-ве каналов 9 и 10
//const int AdcSampleRate[] =
//{
//	0,          // 0
//	96000,      // 1
//	96000,      // 2
//	48000,      // 3
//	48000,      // 4
//	24000,      // 5
//	24000,      // 6
//	24000,      // 7
//	24000,      // 8
//	48000,      // 9
//	24000,      // 10
//
//};

// указано для Fadc=9MHz, ADC_SMPR=2
// и без перемежения измерений
const int AdcSampleRate[] =
{
	0,          // 0
	96000,      // 1
	96000,      // 2
	96000,      // 3
	48000,      // 4
	48000,      // 5
	48000,      // 6
	48000,      // 7
	24000,      // 8
	24000,      // 9
	24000,      // 10
};

#define AVG_NUM (AdcSampleRate[ADC_CHN_NUM] / CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE)  // сколько усреднять выборок АЦП на 1 отсчёт USB-audio

//---------------------------------------------------------------------------------------------------------------------------------------------------
void adc_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

	// init ADC1

	#define ADC_SMPR 2
	#define ADC_ZERO_CHN 17

	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV8;  // ADCCLK = 72/8 = 9 MHz
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;             // start calibration
	while (!(ADC1->CR2 & ADC_CR2_CAL));   // wait calibration complete


	#if  (1) //ADC_CHN_NUM > 8
		#define ADC_DMA_NUM  (ADC_CHN_NUM)
		ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15) | (4 << 20) | (5 << 25);
		ADC1->SQR2 = (6 << 0) | (7 << 5) | (8 << 10) | (9 << 15);
		ADC1->SQR1 =                                                                         ((ADC_DMA_NUM - 1) << 20);
	#else //  ADC_CHN_NUM <= 8
		// перемежение измерений с каким-то стабильным каналом, для уменьшения влияния соседних каналов друг на друга
		#define ADC_DMA_NUM  (ADC_CHN_NUM * 2)
		ADC1->SQR3 = (ADC_ZERO_CHN << 0) | (0 << 5) | (ADC_ZERO_CHN << 10) | (1 << 15) | (ADC_ZERO_CHN << 20) | (2 << 25);
		ADC1->SQR2 = (ADC_ZERO_CHN << 0) | (3 << 5) | (ADC_ZERO_CHN << 10) | (4 << 15) | (ADC_ZERO_CHN << 20) | (5 << 25);
		ADC1->SQR1 = (ADC_ZERO_CHN << 0) | (6 << 5) | (ADC_ZERO_CHN << 10) | (7 << 15)                        | ((ADC_DMA_NUM - 1) << 20);
	#endif // ADC_CHN_NUM

	ADC1->SMPR1 = (ADC_SMPR << 27) | (ADC_SMPR << 24) | (ADC_SMPR << 21) | (ADC_SMPR << 18) | (ADC_SMPR << 15)
	            | (ADC_SMPR << 12) | (ADC_SMPR <<  9) | (ADC_SMPR <<  6) | (ADC_SMPR <<  3) | (ADC_SMPR <<  0);

	ADC1->SMPR2 = (ADC_SMPR << 27) | (ADC_SMPR << 24) | (ADC_SMPR << 21) | (ADC_SMPR << 18) | (ADC_SMPR << 15)
	            | (ADC_SMPR << 12) | (ADC_SMPR <<  9) | (ADC_SMPR <<  6) | (ADC_SMPR <<  3) | (ADC_SMPR <<  0);



	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON | (4 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG | ADC_CR2_ALIGN | ADC_CR2_TSVREFE;



	// init TIM3
	// инициализируем TIM3 как триггер
	// находим оптимальные значения периода и прескалера
	uint32_t period = SystemCoreClock / AdcSampleRate[ADC_CHN_NUM];
	int shift = 0;
	while(period > 0x00010000)
	{
		shift++;
		period >>= 1;
	}

	TIM3->PSC = (1 << shift) - 1;   // prescaler
	TIM3->ARR = period - 1;         // period
	TIM3->CR2 = TIM_CR2_MMS_1;      // UPDATE event as TRGO
	TIM3->CNT = 0;

	// инициализируем DMA1-CH1 для приема данных с АЦП
	DMA1_Channel1->CCR   = 0;
	DMA1_Channel1->CMAR  = (unsigned long)g_AdcDmaBuff;
	DMA1_Channel1->CPAR  = (unsigned long)&ADC1->DR;
	DMA1_Channel1->CNDTR = ADC_DMA_NUM;
	DMA1_Channel1->CCR   = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC;

	// подготавливаем к прерываниям
	DMA1->IFCR = 0x000f000f;  // Reset interrupt flag for 1 & 5 channels.
	DMA1_Channel1->CCR   |= DMA_CCR_TCIE;  // Enable CH1 interrupt

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	g_AdcAvgBuffCnt = g_AdcAccCnt = 0;

	// запускаем преобразование
	TIM3->CR1 = TIM_CR1_CEN;  // запускаем всё

}
//---------------------------------------------------------------------------------------------------------------------------------------------------
void DMA1_Channel1_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF1; // reset flag

	for(int i = 0; i < ADC_CHN_NUM; i++)
	{
		g_AdcAcc[i] += g_AdcDmaBuff[i];
	}

	g_AdcAccCnt++;


	if(g_AdcAccCnt >= AVG_NUM)
	{
		g_AdcAccCnt = 0;

		if(g_AdcAvgBuffCnt * ADC_CHN_NUM < CFG_TUD_AUDIO_EP_SZ_IN / 2)
		{
			uint16_t *p16 = g_AdcAvgBuff + g_AdcAvgBuffCnt * ADC_CHN_NUM;
			for(int i = 0; i < ADC_CHN_NUM; i++)
			{
				 p16[i] = (g_AdcAcc[i] / AVG_NUM) ^ 0x8000;
				 g_AdcAcc[i] = 0;
			}
			g_AdcAvgBuffCnt++;
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
