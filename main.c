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
#include "DataFlash.h"

sbit LED1 = P1^6;			//  P1^6; P3^2;

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
            UINT8 key_vals[8] = {0};
            UINT8 read_len = ReadDataFlash(KEY_VALUES_OFFSET, 8, key_vals);
            if (read_len == 8)
            {
                // ??key_vals????0xFF(??????????????)
                UINT8 i;
                UINT8 is_empty = 1;
                for (i = 0; i < 8; i++)
                {
                    if (key_vals[i] != 0xFF)
                    {
                        is_empty = 0;
                        break;
                    }
                }

                if (is_empty)
                {
                    // Flash??????????,?????'#'??
                    usb_send_key("#");
                }
                else
                {
                    // Flash?????????,?????flash????????
                    usb_send_keys_from_flash();
                }
            }
            else
            {
                // ????,????:??'#'???????
                usb_send_key("#");
            }

            LED1 = 0;
            mDelaymS(10);
            LED1 = 1;
            mDelaymS(10);

            break;
      }

      case KEY_EVENT_RELEASED:
      {
				            UINT8 key_vals[8] = {0};
            UINT8 read_len = ReadDataFlash(KEY_VALUES_OFFSET, 8, key_vals);
            if (read_len == 8)
            {
                // ??key_vals????0xFF(??????????????)
                UINT8 i;
                UINT8 is_empty = 1;
                for (i = 0; i < 8; i++)
                {
                    if (key_vals[i] != 0xFF)
                    {
                        is_empty = 0;
                        break;
                    }
                }

                if (is_empty)
                {
                    // Flash??????????,?????'#'??
                    usb_send_key("#");
                }
                else
                {
                    // Flash?????????,?????flash????????
                    usb_send_keys_from_flash();
                }
            }
            else
            {
                // ????,????:??'#'???????
                usb_send_key("#");
            }
				
          //usb_send_key("#");
          //usb_send_keys_from_flash();
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
  mDelaymS(20);
  LED1 = 1;
  mDelaymS(20);

 ReadDataFlash(KEY_VALUES_OFFSET, 8, g_key_values);

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
  
mDelaymS(500);



  while(1)
  {
		static UINT8 my_cnt = 0;
    static UINT8 fail_cnt = 0;
		
//		if (!EnumOK)
//		{
//			soft_reset();
//		}

    	if (EnumOK)
      {
//				UINT8 i = 1;
//				do
//				{
//				LED1 = 1;
//				} while (i--);
          if (g_data_ready)
          {
              compound_process_recv_data(g_data_len);
              g_data_ready = 0;
          }

          // HIDValueHandle();		 // 处理HID数据
          key_scan();
      }
			else		/* 如果是一直枚举不起来，那就软复位，重新枚举 */
			{
				my_cnt ++;

//				LED1 = 1;
//				mDelaymS(10);
//				LED1 = 0;
//				mDelaymS(10);
				
				if (my_cnt >= 100)
				{
//					soft_reset();
					my_cnt = 0; 
					
					
					EA = 0;  
					UEP1_T_LEN = 0;  
					UEP2_T_LEN = 0;  
					FLAG = 0;
					EnumOK = 0;
					
					USBDeviceInit();  
					
					EA = 1;

          fail_cnt ++;

          if (fail_cnt >= 3)
          {
            soft_reset();
          }
					
				}
				
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


