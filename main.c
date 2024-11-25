/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C											
* Author             : WCH                                                      
* Version            : V1.0                                                     
* Date               : 2017/03/15                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard and mouse, and HID compatible device.           
********************************************************************************/
#include 	".\Public\CH554.H"
#include	"Compound.h"
#include 	".\Public\debug.h"
#include 	"stdio.h"
#include    "key.h"
#include    "Timer.H"

sbit LED1 = P3^2;

extern UINT8 	FLAG;												  // Trans complete flag
extern UINT8 	EnumOK;												// Enum ok flag					

void soft_reset(void)
{
  UINT8 i = 0;
  for (i = 0; i <= 2; i++)
  {
  	LED1 = 0;
	mDelaymS(50);
	LED1 = 1;
	mDelaymS(50);
  }

  EA = 0;
  mDelaymS(10);

  SAFE_MOD = 0x55;
  SAFE_MOD = 0xAA;
  GLOBAL_CFG |= bSW_RESET;
}

void led_flash_handler(void)
{
    static UINT8 cnt = 0;

    if(++cnt >= 50) 
    {  
        cnt = 0;
        soft_reset();
    }
}

void handle_key_event(key_event_t event)
{
  switch(event)
  {
      case KEY_EVENT_PRESSED:
      {
          usb_send_key("P");
          break;
      }

      case KEY_EVENT_RELEASED:
      {
          usb_send_key("R");
          break;
      }
      
      case KEY_EVENT_NONE:
      {
          break;
      }

      default:
      {
          break;
      }

  }
}

void main(void)
{
  UINT8 i = 0;

  CfgFsys();                                //Configure sys
  mDelaymS(5);                              //
  mInitSTDIO( );                            // Init UART0

#if	DE_PRINTF
    printf( "Start config.\r\n" );
    printf( "Init USB device.\r\n" );
#endif

  USBDeviceInit();                          // Configure the USB module,the Endpoint and the USB Interrupt
  EA = 1;	
	UEP1_T_LEN = 0;                           // Reset the trans length register
  UEP2_T_LEN = 0;                                                  
  FLAG = 0;
	EnumOK = 0;

  key_init();
  register_key_cb(handle_key_event);

  timer0_init_10ms();
  timer0_register_cb(led_flash_handler);


  while(1)
  {

    	if (EnumOK)
      {	
          // HIDValueHandle();		 // 处理HID数据
          key_scan();
      }
      
      mDelaymS(10);
  }
}

/**************************** END *************************************/


