/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Reinhard Panhuber
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
 */

/* plot_audio_samples.py requires following modules:
 * $ sudo apt install libportaudio
 * $ pip3 install sounddevice matplotlib
 *
 * Then run
 * $ python3 plot_audio_samples.py
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "stm32f1xx_hal.h"
#include "tusb.h"

#include "led_blink.h"
#include "goods.h"
#include "adc_run.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// device_state:
// bit 0 - mounted or not
// bit 1 - idle or on record
uint32_t device_state = 0;

LedBlinkState_t blink_stt = {0,0};

#if CFG_USE_FEATURE_UNIT
	// Audio controls
	// Current states
	//bool       mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];       // +1 for master channel 0
	//int16_t    volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];     // +1 for master channel 0
	bool       mute;       // master channel 0
	int16_t    volume_dB;  // master channel 0
	int32_t    volume_k = 0x10000;
#endif //CFG_USE_FEATURE_UNIT

uint32_t   sampFreq;
uint8_t    clkValid;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX+1]; 			// Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; 						// Sample frequency range state

// Audio test data
//uint16_t test_buffer_audio[CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX];
//int16_t startVal = 0;
//int     bytes_to_send = 0;
char g_DeviceName[32];

void audio_task(void);
void led_blinking_routine(void);
uint32_t get_volume_from_dB(int dB);
void make_device_name(void);
uint32_t board_get_ctrl_pins(void);
//------------------------------------------------------------------------------------------------------------------------------
/*
  + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN

*/

/*------------- MAIN -------------*/
int main(void)
{

    board_init();

    adc_init();
    make_device_name();

    // init device stack on configured roothub port
    tusb_rhport_init_t dev_init =
    {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    // Init values
    sampFreq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    clkValid = 1;

    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;


    while (1)
    {
        tud_task(); // tinyusb device task
        audio_task();
    }
}
//------------------------------------------------------------------------------------------------------------------------------
void make_device_name(void)
{
	#if CFG_ALERNATION_2_CH_8
		strcpy(g_DeviceName, "TinyUSB MIC alt ");
	#else // CFG_ALERNATION_2_CH_8
		strcpy(g_DeviceName, "TinyUSB MIC ");
	#endif // CFG_ALERNATION_2_CH_8
	itoa(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, g_DeviceName + strlen(g_DeviceName), 10);
	strcat(g_DeviceName, "ch/");
	int real_sample_rate = adc_get_actual_sample_rate() / adc_get_ADC_AVG_NUM();
	if(real_sample_rate % 1000 == 0)
	{
		itoa(real_sample_rate / 1000, g_DeviceName + strlen(g_DeviceName), 10);
		strcat(g_DeviceName, "kHz");
	}
	else
	{
		itoa(real_sample_rate, g_DeviceName + strlen(g_DeviceName), 10);
		strcat(g_DeviceName, "Hz");
	}
	//
}
//------------------------------------------------------------------------------------------------------------------------------

void update_led_mode(void)
{
	const int led_sss[] = {0, 1, 0, 2};
	if(device_state < NUMOFARRAY(led_sss))
	{
		int new_mode = led_sss[device_state];

		if(new_mode != blink_stt.idx)
		{
			LED_BLINK_SET(blink_stt, new_mode);
		}
	}
}
//------------------------------------------------------------------------------------------------------------------------------
/*
	Получаем уровень громкости (коэфф. для умножения) из dB
	Возврат в формате 16.16
*/
uint32_t get_volume_from_dB(int dB)
{
	const uint32_t pow2_6[] =
	{
		1.0000 * 0x80000000, // 2^(0/6) === 84dB
		1.1224 * 0x80000000, // 2^(1/6)
		1.2599 * 0x80000000, // 2^(2/6)
		1.4142 * 0x80000000, // 2^(3/6)
		1.5874 * 0x80000000, // 2^(4/6)
		1.7817 * 0x80000000, // 2^(5/6)
	};

	if(dB >= +96) return 0xffffffff;
	if(dB <= -96) return 0;

	int shift;
	int idx;

	dB += 96; // all values to positive

	shift = 15 + 16 - dB / 6;
	idx = dB % 6;

	return pow2_6[idx] >> shift;
}
//------------------------------------------------------------------------------------------------------------------------------
//int adc_sample_rate = 0;
//uint32_t adc_last_cnt;
//extern volatile int g_AdcIntCnt;
void user_systick_handler(uint32_t tick_count)
{
	uint32_t tc10 = tick_count % 10;
	if(tc10 == 0)
	{
		led_blinking_routine();
	}
	else if(tc10 == 1)
	{
		uint32_t ctrl_pins = board_get_ctrl_pins();
		g_AdcCtrl = (ctrl_pins >> 0) & 0x07;
	}

//	if(tick_count % 1000 == 100)
//	{
//		uint32_t i = g_AdcIntCnt;
//		adc_sample_rate = i - adc_last_cnt;
//		adc_last_cnt = i;
//	}
}

//--------------------------------------------------------------------+
// Device callbacks
// Вызываются в следующем порядке:
// при подключении к USB:
//    1. tud_suspend_cb(0)
//    2. tud_resume_cb()
//    3. tud_mount_cb()
// при отключении от USB:
//    1. tud_suspend_cb(0)
// при повторном подключении к USB:
//    1. tud_resume_cb()
//    2. tud_suspend_cb(0)
//    3. tud_resume_cb()
//    4. tud_mount_cb()
//
// 'tud_umount_cb' не вызываются
//--------------------------------------------------------------------+
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when device is mounted
void tud_mount_cb(void)
{
    device_state = 1;
    update_led_mode();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    device_state = 0;
    update_led_mode();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}
//------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+
//------------------------------------------------------------------------------------------------------------------------------

void audio_task(void)
{
    // Yet to be filled - e.g. put meas data into TX FIFOs etc.
    // asm("nop");
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+
//------------------------------------------------------------------------------------------------------------------------------

//volatile uint32_t stat[512];
//int stat_cnt = 0;

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 100;

    (void) rhport;
    (void) pBuff;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void) channelNum;
    (void) ctrlSel;
    (void) ep;

    return false; 	// Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 101;
    (void) rhport;
    (void) pBuff;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void) channelNum;
    (void) ctrlSel;
    (void) itf;


    return false; 	// Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
    (void) rhport;


    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	#if CFG_USE_FEATURE_UNIT
		// Page 91 in UAC2 specification
		uint8_t channelNum = TU_U16_LOW(p_request->wValue);
		uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
		uint8_t itf = TU_U16_LOW(p_request->wIndex);
		uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

	//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 102;
	//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = entityID  ;
	//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = ctrlSel   ;
	//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = channelNum;
	//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = itf       ;

		// If request is for our feature unit
		if ( entityID == 2 )
		{
			switch ( ctrlSel )
			{
			case AUDIO_FU_CTRL_MUTE:
				// Request uses format layout 1
				TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

				//mute[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;
				if(channelNum == 0) mute =  ((audio_control_cur_1_t*) pBuff)->bCur;

				//TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
				return true;

			case AUDIO_FU_CTRL_VOLUME:
				// Request uses format layout 2
				TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

				//volume[channelNum] = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;
				if(channelNum == 0)
				{
					volume_dB = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;
					volume_k = get_volume_from_dB(volume_dB);
				}


				//TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
				return true;

			// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
			}
		}
	#endif //CFG_USE_FEATURE_UNIT
    return false;    // Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t ep         = TU_U16_LOW(p_request->wIndex);

//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 103;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = ep        ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = ctrlSel   ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = channelNum;
//


    (void) channelNum;
    (void) ctrlSel;
    (void) ep;

    //	return tud_control_xfer(rhport, p_request, &tmp, 1);

    return false; 	// Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 104;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = itf       ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = ctrlSel   ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = channelNum;


    (void) channelNum;
    (void) ctrlSel;
    (void) itf;

    return false; 	// Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t itf        = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);


//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 105;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = entityID  ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = ctrlSel   ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = channelNum;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = itf       ;

	(void) itf;
	(void) channelNum;

    // Input terminal (Microphone input)
    if (entityID == 1)
    {
        switch ( ctrlSel )
        {
        case AUDIO_TE_CTRL_CONNECTOR:
        {
            // The terminal connector control only has a get request with only the CUR attribute.
            audio_desc_channel_cluster_t ret;

            // Those are dummy values for now
            ret.bNrChannels = 1;
            ret.bmChannelConfig = (audio_channel_config_t) 0;
            ret.iChannelNames = 0;

            TU_LOG2("    Get terminal connector\r\n");

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));
        }
        break;

        // Unknown/Unsupported control selector
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

	#if CFG_USE_FEATURE_UNIT
		// Feature unit
		if (entityID == 2)
		{
			switch ( ctrlSel )
			{
			case AUDIO_FU_CTRL_MUTE:
				// Audio control mute cur parameter block consists of only one byte - we thus can send it right away
				// There does not exist a range parameter block for mute
				TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
				//return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);
				return tud_control_xfer(rhport, p_request, &mute, 1);

			case AUDIO_FU_CTRL_VOLUME:
				switch ( p_request->bRequest )
				{
				case AUDIO_CS_REQ_CUR:
					TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
					//return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));
					return tud_control_xfer(rhport, p_request, &volume_dB, sizeof(volume_dB));

				case AUDIO_CS_REQ_RANGE:
					TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

					// Copy values - only for testing - better is version below
					audio_control_range_2_n_t(1) ret;

					ret.wNumSubRanges = 1;
					ret.subrange[0].bMin = -50;   // -50 dB
					ret.subrange[0].bMax = +50;   // +50 dB
					ret.subrange[0].bRes = 1;     // 1 dB steps

					return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));

				// Unknown/Unsupported control
				default:
					TU_BREAKPOINT();
					return false;
				}
				break;

			// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
			}
		}
	#endif //CFG_USE_FEATURE_UNIT


    // Clock Source unit
    if ( entityID == 4 )
    {
        switch ( ctrlSel )
        {
        case AUDIO_CS_CTRL_SAM_FREQ:
            // channelNum is always zero in this case
            switch ( p_request->bRequest )
            {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Sample Freq.\r\n");
                return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Sample Freq. range\r\n");
                return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

        case AUDIO_CS_CTRL_CLK_VALID:
            // Only cur attribute exists for this request
            TU_LOG2("    Get Sample Freq. valid\r\n");
            return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false; 	// Yet not implemented
}
//------------------------------------------------------------------------------------------------------------------------------
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 106;
//	if(bytes_to_send < NUMOFARRAY(stat)) stat[bytes_to_send]++;
	(void) rhport;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;

	//tud_audio_write ((uint8_t *)test_buffer_audio, sizeof(test_buffer_audio));
	//tud_audio_write ((uint8_t *)test_buffer_audio, bytes_to_send);
	//bytes_to_send = 0;

	int buff_idx = g_AdvAvgBuffIdx;
	adc_set_buff_idx(!buff_idx);
	int fill_cnt = g_AdcAvgBuffCnt[buff_idx];

	tud_audio_write ((uint8_t *)g_AdcAvgBuff[buff_idx], fill_cnt * sizeof(g_AdcAvgBuff[0][0]) * ADC_CHN_NUM);

	g_AdcAvgBuffCnt[buff_idx] = 0;



	device_state |= 2;
	update_led_mode();

	return true;
}
//------------------------------------------------------------------------------------------------------------------------------
//volatile uint32_t last_bytes_to_send = 0;
bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 107;
    (void) rhport;
    (void) n_bytes_copied;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;

//    for (size_t cnt = 0; cnt < NUMOFARRAY(test_buffer_audio) / CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX; cnt++)
//    {
//    	int ii = cnt * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;
//    	for(uint32_t c = 0; c < CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX; c++)
//		{
//			#if CFG_USE_FEATURE_UNIT
//				if(mute) test_buffer_audio[ii + c] = 0;
//				else
//				{
//					int16_t v16 = startVal + c * 0x10000 * 11 / 16;
//					int64_t v64 = v16;
//					v64 *= volume_k;
//					v64 >>= 16;
//
//					const int lim15 = 32767;
//
//					test_buffer_audio[ii + c] = lim(-lim15, v64, lim15);
//				}
//			#else  //CFG_USE_FEATURE_UNIT
//				int16_t v16 = startVal + c * 0x10000 * 11 / 16;
//				test_buffer_audio[ii + c] = v16;
//			#endif //CFG_USE_FEATURE_UNIT
//
//		}

//    	if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX >= 1)    test_buffer_audio[ii + 0] =  startVal;
//    	if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX >= 2)    test_buffer_audio[ii + 1] = -startVal;
//    	if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX >= 3)    test_buffer_audio[ii + 2] =  startVal > 0 ? 32767 : -32767;
//    	if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX >= 4)    test_buffer_audio[ii + 3] =  startVal < 0 ? 32767 : -32767;

//		startVal++;
//    }


    return true;
}
//------------------------------------------------------------------------------------------------------------------------------

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;
//    (void) p_request;
//    startVal = 0;

//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = 108;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = p_request->bRequest ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = p_request->wValue ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = p_request->wIndex ;
//	if(stat_cnt < NUMOFARRAY(stat)) stat[stat_cnt++] = p_request->wLength ;

    device_state &= ~2;
    update_led_mode();

    return true;
}
//------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------+
// BLINKING TASK
// вызывать 100 раз/сек
//--------------------------------------------------------------------+
void led_blinking_routine(void)
{
	const uint8_t led_off[]     = {0, -1};  // светик потушен
	const uint8_t led_on[]      = {-1};     // светик горит
	const uint8_t led_blink_1[] = {25,25};   // часто мигает
	const uint8_t led_blink_2[] = {3, 150};   // редко вспыхивает

	const LedBlinkSet_t blink_sets[] =
	{
		LED_BLINK_ITEM(led_off),
		LED_BLINK_ITEM(led_on),
		LED_BLINK_ITEM(led_blink_1),
		LED_BLINK_ITEM(led_blink_2),
		//LED_BLINK_ITEM(),
	};

	if(blink_stt.idx >= NUMOFARRAY(blink_sets)) return;

	const LedBlinkSet_t *fset = blink_sets + blink_stt.idx;

	if(fset->set[blink_stt.offs] != -1)
	{
		while(blink_stt.time >= fset->set[blink_stt.offs])
		{
			if(++blink_stt.offs >= fset->num) blink_stt.offs = 0;
			blink_stt.time = 0;
		}
		blink_stt.time++;
	}

	int led_val = !(blink_stt.offs & 1);

	board_led_write(led_val);

}
//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------
