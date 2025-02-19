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


int16_t       g_AdcAvgBuff[2][CFG_TUD_AUDIO_EP_SZ_IN / 2];  // используем 2 буфера для накопления отсчётов АЦП: один заполняется, другой готов к передаче в USB
unsigned int  g_AdcAvgBuffCnt[2] = {0, 0}; // заполненности буферов АЦП
unsigned int  g_AdvAvgBuffIdx = 0;    // номер текущего буфера (0/1), в который накапливаются отсчёты АЦП
volatile int  g_AdcAvgNum = 0;  // счётчик усреднений выборок АЦП.
uint32_t      g_AdcCtrl = 3;
//volatile int g_AdcIntCnt = 0;  // debug


#define ADC_RATE(x) (((x) / CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE) * CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE) // при необходимости уменьшаем частоту выборок АЦП, что бы она была кратной частоте выборок аудиоустройства
#define ADC_AVG_NUM (AdcSampleRate[ADC_CHN_NUM] / CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE)  // сколько усреднять выборок АЦП на 1 отсчёт USB-audio

// зависимость частоты дискредизации АЦП от кол-ва каналов
// когда АЦП успевает всё оцифровать
// это значение кратно меньше 192кГц
#if CFG_ADC_ALTERNATION_EN
	// указано для Fadc=9MHz, ADC_SMPR=1
	// и перемежением измерений при кол-ве каналов от 2 до 8,
	// без перемежения при кол-ве каналов 1, 9 и 10
	const int AdcSampleRate[] =
	{
		ADC_RATE(0),          // 0
		ADC_RATE(192000),     // 1
		ADC_RATE(96000),      // 2
		ADC_RATE(48000),      // 3
		ADC_RATE(48000),      // 4
		ADC_RATE(38400),      // 5
		ADC_RATE(32000),      // 6
		ADC_RATE(32000),      // 7
		ADC_RATE(27428),      // 8
		ADC_RATE(32000),      // 9
		ADC_RATE(32000),      // 10

	};
#else // CFG_ADC_ALTERNATION_EN
	// указано для Fadc=9MHz, ADC_SMPR=2
	// и без перемежения измерений
	const int AdcSampleRate[] =
	{
		ADC_RATE(0),          // 0
		ADC_RATE(192000),     // 1
		ADC_RATE(96000),      // 2
		ADC_RATE(96000),      // 3
		ADC_RATE(64000),      // 4
		ADC_RATE(64000),      // 5
		ADC_RATE(48000),      // 6
		ADC_RATE(48000),      // 7
		ADC_RATE(38400),      // 8
		ADC_RATE(32000),      // 9
		ADC_RATE(32000),      // 10
	};
#endif // CFG_ADC_ALTERNATION_EN



//---------------------------------------------------------------------------------------------------------------------------------------------------
void adc_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

	// init ADC1


	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV8;  // ADCCLK = 72/8 = 9 MHz
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;             // start calibration
	while (!(ADC1->CR2 & ADC_CR2_CAL));   // wait calibration complete

	#if  (ADC_CHN_NUM > 1) && (ADC_CHN_NUM <= 8) && (CFG_ADC_ALTERNATION_EN)
		// перемежение измерений с каким-то стабильным каналом, для уменьшения влияния соседних каналов друг на друга
		#define ADC_DMA_NUM  (ADC_CHN_NUM * 2)
		#define ADC_SMPR 1
		#define ADC_ZERO_CHN 17
		ADC1->SQR3 = (ADC_ZERO_CHN << 0) | (0 << 5) | (ADC_ZERO_CHN << 10) | (1 << 15) | (ADC_ZERO_CHN << 20) | (2 << 25);
		ADC1->SQR2 = (ADC_ZERO_CHN << 0) | (3 << 5) | (ADC_ZERO_CHN << 10) | (4 << 15) | (ADC_ZERO_CHN << 20) | (5 << 25);
		ADC1->SQR1 = (ADC_ZERO_CHN << 0) | (6 << 5) | (ADC_ZERO_CHN << 10) | (7 << 15)                        | ((ADC_DMA_NUM - 1) << 20);
	#else //  ADC_CHN_NUM <= 8
		#define ADC_DMA_NUM  (ADC_CHN_NUM)
		#define ADC_SMPR 2
		ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15) | (4 << 20) | (5 << 25);
		ADC1->SQR2 = (6 << 0) | (7 << 5) | (8 << 10) | (9 << 15);
		ADC1->SQR1 =                                                                         ((ADC_DMA_NUM - 1) << 20);
	#endif // ADC_CHN_NUM

	ADC1->SMPR1 = (ADC_SMPR << 27) | (ADC_SMPR << 24) | (ADC_SMPR << 21) | (ADC_SMPR << 18) | (ADC_SMPR << 15)
	            | (ADC_SMPR << 12) | (ADC_SMPR <<  9) | (ADC_SMPR <<  6) | (ADC_SMPR <<  3) | (ADC_SMPR <<  0);

	ADC1->SMPR2 = (ADC_SMPR << 27) | (ADC_SMPR << 24) | (ADC_SMPR << 21) | (ADC_SMPR << 18) | (ADC_SMPR << 15)
	            | (ADC_SMPR << 12) | (ADC_SMPR <<  9) | (ADC_SMPR <<  6) | (ADC_SMPR <<  3) | (ADC_SMPR <<  0);

	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON | (4 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG /*| ADC_CR2_ALIGN*/ | ADC_CR2_TSVREFE;

	// init TIM3
	// инициализируем TIM3 как триггер
	// находим оптимальные значения периода и прескалера
	uint32_t period = SystemCoreClock / 192000; //AdcSampleRate[ADC_CHN_NUM];
	int shift = 0;
	while(period > 0x00010000)
	{
		shift++;
		period >>= 1;
	}

	TIM3->PSC = (1 << shift) - 1;   // prescaler
	TIM3->ARR = period - 1;         // period
	TIM3->CR2 = TIM_CR2_MMS_1;      // set 'UPDATE' event as 'TRGO'
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
	NVIC_SetPriority(DMA1_Channel1_IRQn, 1);

	g_AdcAvgBuffCnt[0] = g_AdcAvgBuffCnt[1] = g_AdcAccCnt = 0;

	// запускаем преобразование
	TIM3->CR1 = TIM_CR1_CEN;  // запускаем всё
	g_AdcAvgNum = ADC_AVG_NUM; // визуализация для отладки

}
//---------------------------------------------------------------------------------------------------------------------------------------------------
int adc_get_actual_sample_rate(void)
{
	return AdcSampleRate[ADC_CHN_NUM] / ADC_AVG_NUM;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------
// устанавливаем, какой буфер (0 или 1) будет использоваться для накопления отсчётов
void adc_set_buff_idx(unsigned int idx)
{
	if(idx < NUMOFARRAY(g_AdcAvgBuffCnt))
	{
		//disable_irq;
			g_AdvAvgBuffIdx = idx;
		//enable_irq;
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
int ccc = 0;
void DMA1_Channel1_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF1; // reset flag

	for(int i = 0; i < ADC_CHN_NUM; i++)
	{
		if(g_AdcCtrl & 4) g_AdcAcc[i] += g_AdcDmaBuff[i];  // averaging
		else              g_AdcAcc[i]  = g_AdcDmaBuff[i];  // single (last) sample
	}

	g_AdcAccCnt++;

	if(g_AdcAccCnt >= ADC_AVG_NUM)
	{
		g_AdcAccCnt = 0;
		unsigned int *pBuffCnt = g_AdcAvgBuffCnt + g_AdvAvgBuffIdx;

		if(*pBuffCnt * ADC_CHN_NUM < CFG_TUD_AUDIO_EP_SZ_IN / 2)
		{
			int16_t *p16 = g_AdcAvgBuff[g_AdvAvgBuffIdx] + *pBuffCnt * ADC_CHN_NUM;
			for(int i = 0; i < ADC_CHN_NUM; i++)
			{
				int vv;

				// Ctrl pin 1 - ADC signal or imitation
				if(g_AdcCtrl & 1)
				{
					vv = g_AdcAcc[i];  // real ADC signal
					if(g_AdcCtrl & 4) vv /= ADC_AVG_NUM;  // averaging summ
				}
				else  vv = (ccc++ & 1) ? 0 : 4095;     // flip-flop imitation

				// Ctrl pin 2 - full 16-bit scale or 12-bit ADC scale
				if(g_AdcCtrl & 2) vv = (vv << 4) ^ 0x8000;

				p16[i] = vv;

				g_AdcAcc[i] = 0;
			}
			(*pBuffCnt)++;
		}
	}
	//g_AdcIntCnt++;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------
