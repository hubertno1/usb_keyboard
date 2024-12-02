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

sbit LED1 = P1^6;

extern UINT8 	FLAG;												  // Trans complete flag
extern UINT8 	EnumOK;												// Enum ok flag

void wdog_init(void);


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
    // static UINT8 cnt = 0;

    // if(++cnt >= 50) 
    // {  
    //     cnt = 0;
    //     soft_reset();
    // }
    if(++g_compound_heartbeat_timer >= 50) 
    {  
        soft_reset();
    } 

}

void handle_key_event(key_event_t event)
{
  switch(event)
  {
      case KEY_EVENT_PRESSED:
      {
          usb_send_key("#");
		    LED1 = 0;
			mDelaymS(10);
			LED1 = 1;
			mDelaymS(10);


          break;
      }

      case KEY_EVENT_RELEASED:
      {
          usb_send_key("#");
		  	LED1 = 0;
			mDelaymS(10);
			LED1 = 1;
			mDelaymS(10);

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
  mDelaymS(10);                              //
  mInitSTDIO( );                            // Init UART0

  LED1 = 0;
  mDelaymS(5);
  LED1 = 1;
  mDelaymS(5);
  

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
  // timer0_register_cb(led_flash_handler);
  wdog_init();             // 初始化看门狗
  



  while(1)
  {

    	if (EnumOK)
      {
          if (g_data_ready)
          {
              compound_process_recv_data(g_data_len);
              g_data_ready = 0;
          }

          // HIDValueHandle();		 // 处理HID数据
          key_scan();
      }
      
        WDOG_COUNT = 0x6F;  // 重置看门狗计数器以防止超时复位

      mDelaymS(10);
  }
}

void wdog_init(void)
{
    // 启用安全模式，写寄存器使能
    SAFE_MOD = 0x55; // 进入安全模式
    SAFE_MOD = 0xAA;

    // 设置 WDOG_COUNT 初始值
    WDOG_COUNT = 0x49;  // 设置为约 10 秒的计数值 (主频 12MHz)

    // 使能看门狗
    GLOBAL_CFG |= bWDOG_EN;  // 允许看门狗复位
}

/**************************** END *************************************/


