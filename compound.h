/********************************** (C) COPYRIGHT *******************************
* File Name          :Compound_Dev.C											*	
* Author             : WCH                                                      *
* Version            : V1.2                                                     *
* Date               : 2017/02/24                                               *
* Description        : A demo for USB compound device created by CH554, support *
					   keyboard , and HID-compliant device.                     *
********************************************************************************/


#ifndef	__COMPOUND_H__
#define __COMPOUND_H__

#define KEY_VALUES_OFFSET 0x40

// extern UINT16I 	TouchKeyButton;	

extern	void 	USBDeviceInit();
// extern	void 	HIDValueHandle();
void usb_send_key (char *p);
void compound_process_recv_data(UINT8 len);
void usb_send_keys_from_flash(void);

typedef enum {
	LEARNMATCH_STATE_IDLE,
	LEARNMATCH_STATE_ACTIVE
} learnmatch_state_t;

typedef enum {
	CLEANMATCH_STATE_IDLE,
	CLEANMATCH_STATE_ACTIVE
} cleanmatch_state_t;

extern volatile UINT8 g_data_ready;
extern volatile UINT8 g_data_len;
//extern volatile UINT8 g_compound_heartbeat_flag;
extern volatile UINT8 g_compound_heartbeat_timer;
extern volatile UINT16 g_learnmatch_timer;
extern volatile UINT16 g_cleanmatch_timer;
extern volatile learnmatch_state_t learnmatch_state;
extern volatile cleanmatch_state_t cleanmatch_state;

extern  UINT8 g_key_values[8];


	
#endif
/**************************** END *************************************/
