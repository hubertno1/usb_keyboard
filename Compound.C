/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C												
* Author             : WCH                                                      
* Version            : V1.2                                                     
* Date               : 2017/02/24                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard , and HID-compliant device.                     
********************************************************************************/

#include 	".\Public\CH554.H"
#include 	".\Public\DEBUG.H"
#include 	"compound.h"
#include 	"key.h"

#include 	<stdio.h>
#include 	<stdlib.h>
#include 	<string.h>
#include 	"DataFlash.h"

#define 	THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define		BUFFER_SIZE				64
#define 	DUAL_BUFFER_SIZE		128
#define 	UsbSetupBuf     		((PUSB_SETUP_REQ)Ep0Buffer)
#define		L_WIN 					0X08
#define 	L_ALT 					0X04
#define		L_SHIFT					0X02
#define 	L_CTL					0X01
#define 	R_WIN 					0X80
#define 	R_ALT 					0X40
#define 	R_SHIFT					0X20
#define 	R_CTL					0X10
#define 	SPACE					0X2C
#define		ENTER					0X28

#define MOUSE 0

#pragma  NOAREGS

UINT8X  	Ep0Buffer[THIS_ENDP0_SIZE]  _at_ 0x0000;  								// Endpoint 0, buffer OUT/OUT，the address must be even.
UINT8X  	Ep1Buffer[BUFFER_SIZE] 		_at_ 0x000A;  								// Endpoint 1, buffer IN，the address must be even.
UINT8X 		Ep2Buffer[DUAL_BUFFER_SIZE]	_at_ 0x0050;  								// Endpoint 2, buffer OUT[64]+IN[64]，the address must be even.


/**************************** Global variable ********************************/	
UINT8   volatile	SetupReq, SetupLen, UsbConfig;
UINT8	volatile	EnumOK, FLAG;
PUINT8  	pDescr;                                                                	// USB enumerate complete flag.
USB_SETUP_REQ   					SetupReqBuf;                                   	// A buffer for Setup package.

char code sPath[] = "`  C:\\HID_RECV.EXE   \r   ";										// The path for the software. SPACE for delay
char *pStr = sPath;

UINT8 HIDCom[64] = {0};												
UINT8 HIDKey[8] = {0};

static UINT8 compound_received_data[BUFFER_SIZE] = {0};										// 存储从PC中接收的数据
static UINT8 compound_response_data[BUFFER_SIZE] = {0};
volatile UINT8 g_data_ready;
volatile UINT8 g_data_len;
#define PACKET_HEADER 				0xAA
#define USB_OP_HEARTBEAT 			0x10
#define USB_OP_WL_LEARNMATCH 		0x1F
#define USB_OP_WL_CLEANMATCH		0X20
#define USB_OP_WR_KEYVALUE 			0X2F

#define VERSION_STR 				"1.0.0"
#define DEVICE_VID_L     0x31    // VID低字节
#define DEVICE_VID_H     0x51    // VID高字节
#define DEVICE_PID_L     0x07    // PID低字节
#define DEVICE_PID_H     0x20    // PID高字节
static UINT8 first_hb_flag = 0;
extern void soft_reset(void);
extern void timer0_register_cb(void (*cb)(void));  // 声明timer注册函数
extern void led_flash_handler(void);               // 声明回调函数
// 1. 先声明函数原型
void compound_process_recv_data(UINT8 len);
UINT8 usb_check_heartbeat_packet(UINT8 *compound_data, UINT8 len);
volatile UINT8 g_compound_heartbeat_flag = 0;
volatile UINT8 g_compound_heartbeat_timer = 0;

// 定义学习状态

volatile learnmatch_state_t learnmatch_state = LEARNMATCH_STATE_IDLE;
volatile UINT16 g_learnmatch_timer = 0;	// 计时器，单位为10ms
volatile cleanmatch_state_t cleanmatch_state = CLEANMATCH_STATE_IDLE;
volatile UINT16 g_cleanmatch_timer = 0;	// 计时器，单位为10ms

sbit LED1 = P3^2;
// #define KEY_VALUES_OFFSET 0x40			/* Data flash从C000H - C0FFH，其中只有偶数地址有效，也就是只有128字节有效 */
UINT8 g_key_values[8] = {0};		/* 用于存储按键值 */

/**************************** Device Descriptor *************************************/
UINT8C DevDesc[18] = {																// Device Descriptor
	0x12,0x01,
	0x10,0x01,
	0x00,0x00,
	0x00,0x08,                      
	0x31,0x51,																		// Vendor ID   |  VID =  0X5131///413c
	0x07,0x20,																		// Product ID  |  PID = 0X2007 /// 2105
//	0x00,0x01,
//	0x05,0x21,
	0x00,0x11,																		// bcdDevice		
	0x00,0x00,
	0x00,0x01
};
/**************************** HID Report Descriptor *********************************/
UINT8C KeyRepDesc[65] = 															// Report Descriptor, DELL Keyboard
{
	0x05, 0x01, 		// Usage page Generatic Desktop
	0x09, 0x06, 		// Usage keyboard
	0xa1, 0x01, 		// Collation Application
	0x05, 0x07, 		// Usafe page (key code)
	0x19, 0xe0, 		// Usage Min ( E0 -->  L_CTL)
	0x29, 0xe7, 		// Usage MAX ( E7 --> R_GUI )
	0x15, 0x00, 		// Logical min
	0x25, 0x01, 		// Logical max
	0x95, 0x08, 		// Report count ( 8 )
	0x75, 0x01, 		// Report size	( 1 )
	0x81, 0x02, 		// Input ( Data, Variable, Absolute )
	0x95, 0x08, 		// Report count ( 8 )
	0x75, 0x01, 		// Report size	( 1 )
	0x81, 0x01, 		// Input ( const )
	0x05, 0x08, 		// Usage page( LED )
	0x19, 0x01, 		// Usage min ( 1 )
	0x29, 0x03, 		// Usage max ( 3 )
	0x95, 0x03, 		// Report count ( 3 )
	0x75, 0x01, 		// Report size ( 1 )
	0x91, 0x02, 		// Output ( Data, Variable, Absolute )
	0x95, 0x01, 		// Report count ( 1 )
	0x75, 0x05, 		// Report size ( 5 )
	0x91, 0x01, 		// Output ( const )
	0x05, 0x07, 		// Usage page ( key code )
	0x19, 0x00, 		// Usage min ( 0H )
	0x2a, 0xff, 0x00,	// Usage max ( FFH )
	0x15, 0x00,			// Logical min ( 0H )
	0x26, 0xff, 0x00,	// Logical max ( FFH )
	0x95, 0x06, 		// Report count ( 6 )
	0x75, 0x08, 		// Report size ( 8 )
	0x81, 0x00, 		// Input ( Data, Array, Absolute )
	0xc0				// End collection
};

UINT8C ComRepDesc[34] = 															// Report Descriptor, Composite device
{
	0x06, 0x00, 0xff, 	// Usage page Vendor defined
	0x09, 0x01, 		// Usage keyboard
	0xa1, 0x01, 		// Collation Application
	0x09, 0x02, 		// Mouse
	0x15, 0x00, 		// Logical min ( 0H )
	0x26, 0x00, 0xff,	// Logical max ( FFH )
	0x75, 0x08,  		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x81, 0x06,  		// Input ( Data, Relative, Wrap )
	0x09, 0x02, 		// Mouse
	0x15, 0x00,  		// Logical min ( 0H )
	0x26, 0x00, 0xff,	// Logical max ( FFH )
	0x75, 0x08, 		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x91, 0x06, 		// Output ( Data, Relative, Wrap )
	0xc0,
};


UINT8C CfgDesc[9+9+9+7+9+9+7+7] =
{
    0x09,0x02,0x42,0x00,0x02,0x01,0x00,0xA0,0x32,             						// Configure Descriptor
	     
//	0x09,0x02,0x22,0x00,0x01,0x01,0x00,0xA0,0x32,
	0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00,             						// Interface Descriptor, keyboard
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,sizeof(KeyRepDesc),0x00,             		// HID Descriptor
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a,                       						// Endpoint Descriptor, 50ms

//   0x09,0x02,0x29,0x00,0x01,0x01,0x00,0xA0,0x32,             						// Configure Descriptor	
//    0x09,0x04,0x00,0x00,0x02,0x03,0x00,0x00,0x00,             						// Interface Descriptor, Composite device， 2 EPs
	0x09,0x04,0x01,0x00,0x02,0x03,0x00,0x00,0x00,             						// Interface Descriptor, Composite device， 2 EPs
    0x09,0x21,0x10,0x01,0x21,0x01,0X22,sizeof(ComRepDesc),0x00,             		// HID Descriptor
    0x07,0x05,0x82,0x03,0x40,0x00,0x32,                        						// Endpoint Descriptor, 10ms, IN
	0x07,0x05,0x02,0x03,0x40,0x00,0x0a												// Endpoint Descriptor,	10ms, OUT
};

void compound_process_recv_data(UINT8 len);

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : Configure USB mode ，USB device init configure.Configure tie Endpoint, compound device, 
				           Endpoint 0 control trans, Endpoint 1/2 interrupt(IN).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00;                                                           // Device mode
	
    UEP1_DMA = Ep1Buffer;                                                      // Endpoint 1 uploard address
    UEP0_DMA = Ep0Buffer;                                                      // Endpoint 0 trans address
    UEP2_DMA = Ep2Buffer;                                                      // Endpoint 2 uploard address	
    UEP4_1_MOD = UEP4_1_MOD | bUEP1_TX_EN & ~(bUEP1_RX_EN | bUEP1_BUF_MOD | bUEP4_RX_EN | bUEP4_TX_EN); // Endpoint 1 sigle 64 byte send buffer, Endpoint 4 disable
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 // Endpoint 1 auto Toggle flag, In translation return NAK
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 // Endpoint 0 OUT trans return ACK, IN trans return NAK    

    UEP2_3_MOD = UEP2_3_MOD | bUEP2_TX_EN | bUEP2_RX_EN & ~bUEP2_BUF_MOD;      // Endpoint 2 sigle 64 byte send buffer OUT[64]+IN[64] (OUT first)
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 // Endpoint 2 auto Toggle flag, IN trans return NAK
		
	  USB_DEV_AD = 0x00;                                                         // Initial the device address
	  UDEV_CTRL = bUD_PD_DIS;                                                    // Disable DM/PM PULL_DOWN.
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // Enable inter PULL_UP. Auto return NAK, during interrupt routing, if the interrupt flag.
	  UDEV_CTRL |= bUD_PORT_EN;                                                  // Enable USB port.
	  USB_INT_FG = 0xFF;                                                         // Clear interrupt flag.
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;                  // Enable USB interrupt
	  IE_USB = 1;                                                                // Enable USB interrupt
}

/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB Device mode Endpoint 1 upload data, Interrupt mode
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void Enp1IntIn( void )
{			
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey));                              			// Upload data
    UEP1_T_LEN = sizeof(HIDKey);                                             			// Upload length
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                			// Return ACK
    while(( UEP1_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_ACK);                  			// Waiting upload complete, avoid overwriting	
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB Device mode Endpoint 2 upload data, Interrupt mode 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void Enp2IntIn( void )
{	
    memcpy( &Ep2Buffer[64], HIDCom, sizeof(HIDCom) );                         			// Upload data
    UEP2_T_LEN = sizeof(HIDCom);                                           				// Upload length
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                			// Return ACK
    while(( UEP2_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_ACK);                  			// Waiting upload complete, avoid overwriting	
}			

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH554USB ISR
*******************************************************************************/
void DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      			//USB ISR, Using register 1
{
    UINT8 len;			
	
    if(UIF_TRANSFER)                                                           			//USB trans flag
    {	
		switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP) )			
        {			
        case UIS_TOKEN_IN | 2:                                                 			// Endpoint 2 upload
			UEP2_T_LEN = 0;                                                     		// Reset length
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           		// Default return NAK
            break;                                                                         
		 case UIS_TOKEN_OUT | 2:
		 {
			len = USB_RX_LEN;
			if (len > BUFFER_SIZE)
			{
				len = BUFFER_SIZE;
			}

			memcpy(compound_received_data, Ep2Buffer, len);
			g_data_len = len;
			g_data_ready = 1;
			// compound_process_recv_data(len);
			
			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;           		// Default return ACK
			
			break;
			
			                                                 		// Eendpoint 2 download 
            // UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_T_RES_NAK;           		// Default return NAK
            // break;
		 }	                                                                       
        case UIS_TOKEN_IN | 1:                                                  		// Endpoint 1 upload
			UEP1_T_LEN = 0;                                                     		// Reset length
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           		// Default return NAK
            FLAG = 1;                                                           		// Upload complete flag
            break;		                                                                   
        case UIS_TOKEN_SETUP | 0:                                               		// SETUP packet
            len = USB_RX_LEN;
            if( len == (sizeof(USB_SETUP_REQ)) )
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    												// Total length less than 0X7F
                }
                len = 0;                                                         		// Default length is 0
                if( (UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD )
                {
					if( (UsbSetupBuf->bRequestType == 0x21)&&(UsbSetupBuf->bRequest == HID_SET_REPORT) )
					{			
						EnumOK = 1;														// Class require, SET REPORT, indicate the enumerate complete							
#if DE_PRINTF
	printf( "Init ok \r\n" );
#endif						
					}
					if( (UsbSetupBuf->bRequestType == 0x21)&&(UsbSetupBuf->bRequest == HID_SET_IDLE) )
					{					
						UEP0_T_LEN = 0;                                                 //Status stage complete, upload 0 data packet, end the control trans
	//					UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
					}
                    len = 0;                                                  		// Fail
                }
				else{
																						// Deal with the standard require
					SetupReq = UsbSetupBuf->bRequest;
					switch( SetupReq )                                                  // Require code
					{
					case USB_GET_DESCRIPTOR:
						switch( UsbSetupBuf->wValueH )
						{
						case 1:                                                       	// Device descriptor					
							pDescr = DevDesc;                                         	// Write device descriptor to buffer
							len = sizeof( DevDesc );	
							break;	
						case 2:                                                       	// Configuration descriptor					
							pDescr = CfgDesc;                                         	// Write configuration descriptor to buffer
							len = sizeof( CfgDesc );	
							break;	
						case 0x22:                                                    	// HID report descriptor						
							if( UsbSetupBuf->wIndexL == 0 )                           	// Interface index 0 HID report --> keyboard
							{						
								pDescr = KeyRepDesc;                                  	// Write to buffer
								len = sizeof( KeyRepDesc );	
							}	
							else 	
								if( UsbSetupBuf->wIndexL == 1 )                       	// Interface index 1 HID report --> Composite device
								{								
									pDescr = ComRepDesc;                              	// Write to buffer
									len = sizeof( ComRepDesc );
								}
								else
								{
									len = 0xff;                                         // Add interface index, if required
								}
//							if( UsbSetupBuf->wIndexL == 0 )                       	// Interface index 1 HID report --> Composite device
//							{								
//								pDescr = ComRepDesc;                              	// Write to buffer
//								len = sizeof( ComRepDesc );
//							}
//							else
//							{
//								len = 0xff;                                         // Add interface index, if required
//							}
								break;
						default:
							len = 0xff;                                               	// Length equal 0 if code error
							break;	
						}	
						if ( len == 0xff ){	
							break;                                                    	// Error code, jump out the swith sentence
						}											
						if ( SetupLen > len ){	
							SetupLen = len;                                           	// Limit the total length 
						}	
						len = SetupLen >= 8 ? 8 : SetupLen;                           	// Upload this length
						memcpy( Ep0Buffer, pDescr, len );                             	// Upload data
						SetupLen -= len;	
						pDescr += len;	
						break;	
							
					case USB_SET_ADDRESS:	
						SetupLen = UsbSetupBuf->wValueL;                              	// Save the device address					
						break;
					case USB_GET_CONFIGURATION:
		
						Ep0Buffer[0] = UsbConfig;
						if ( SetupLen >= 1 )
						{
							len = 1;
						}					
						break;
					case USB_SET_CONFIGURATION:
			
						UsbConfig = UsbSetupBuf->wValueL;				
						break;
					case 0x0A:
						break;
					case USB_CLEAR_FEATURE:                                            	//Clear Feature
			
						if ( (UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP ) 
						{
						   switch( UsbSetupBuf->wIndexL )
						   {
							  case 0x82:
								   UEP2_CTRL = UEP2_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
								   break;
							  case 0x81:
								   UEP1_CTRL = UEP1_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
								   break;
							  case 0x01:
								   UEP1_CTRL = UEP1_CTRL & ~ (bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
								   break;
							  default:
								   len = 0xFF;                                         	// Error endpoint
								   break;
							}
						 }
						else
						{
							len = 0xFF;                                                	// Deal with the error endpoint
						}
						break;
					case USB_SET_FEATURE:                                          		// Set Feature 					
						if( (UsbSetupBuf->bRequestType & 0x1F) == 0x00 )               
						{
							if( ( ((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
									 /* Awake the device */
								}
								else
								{
									len = 0xFF;                                        	// Error routing
								}
							}
							else
							{
								len = 0xFF;                                            	// Error routing
							}
						}
						else 
							if( (UsbSetupBuf->bRequestType & 0x1F) == 0x02 )            // Set endpoint
							{
								if( ( ((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL ) == 0x00 )
								{
									switch( ((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL )
									{
										case 0x82:
											UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* Endpoint 2 IN STALL */
											//timer0_register_cb(led_flash_handler);
										 
											break;

										case 0x02:
											UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* Endpoint 2 OUT STALL */
											//timer0_register_cb(led_flash_handler);
										 
											break;

										case 0x81:
											UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* Endpoint 1 IN STALL */
											//timer0_register_cb(led_flash_handler);
										 
											break;

										default:
											len = 0xFF;                                   	// Error routing
											break;
									}
								}
								else
								{
									len = 0xFF;                                      		// Error routing
								}
							}
							else
							{
								len = 0xFF;                                          		// Error routing
							}
							break;
					case USB_GET_STATUS:													// Get status
						 Ep0Buffer[0] = 0x00;
						 Ep0Buffer[1] = 0x00;
						 if ( SetupLen >= 2 )
						 {
							len = 2;
						 }
						 else
						 {
							len = SetupLen;
						 }
						 break;
					default:
						len = 0xff;                                                    		// Error routing
						break;
					}
				}
            }
            else
            {
                len = 0xff;                                                         		//Packet length error
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
 //               UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;	// Control endpoint should not return STALL, 
																							// if Send STALL, that means the device has no ability to communicate with the host
            }
            else if(len <= 8)                                                       		// Upload the data or return the 0 data packet
            {		
                UEP0_T_LEN = len;		
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;		// Default data packet is DATA1, and return ACK
            }		
            else		
            {		
                UEP0_T_LEN = 0;  															// Upload 0 data packet, incase the host go to status stage ahead, and get fault code
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;		// Default data packet is DATA1, and return ACK
            }		
            break;		
        case UIS_TOKEN_IN | 0:                                                      		// Endpoint 0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                                 		// Current length
                memcpy( Ep0Buffer, pDescr, len );                                   		// Upload the data
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                           			//Toggle Sysc flag 
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
//                UEP0_T_LEN = 0;                                                     		//Status stage complete, upload 0 data packet, end the control trans
//                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;		
                break;
            }
			if(len == 0xff)
			{
				UEP0_T_LEN = 0;                                                     		//Status stage complete, upload 0 data packet, end the control trans
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;		
			}
            break;
        case UIS_TOKEN_OUT | 0:  															// Endpoint 0 OUT
            len = USB_RX_LEN;
            if( SetupReq == 0x09 )
            {
                if( Ep0Buffer[0] )
                {
#if DE_PRINTF
                    printf("Light on Num Lock LED!\n");
#endif
                }
                else if( Ep0Buffer[0] == 0)
                {
#if DE_PRINTF
                    printf("Light off Num Lock LED!\n");
#endif
                }
            }
            UEP0_T_LEN = 0;  																// Upload 0 data packet, incase the host go to status stage ahead, and get fault code
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;										// Default data packet is DATA1, and return ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                           		// Reset the trans interrupt
    }
    if( UIF_BUS_RST )                                                                		// Device mode Bus reset
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                            		// Reset Reset interrupt
    }		
    if( UIF_SUSPEND )                                                               		// USB Suspended and Awake interrupt
    {		
        UIF_SUSPEND = 0;		
        if ( USB_MIS_ST & bUMS_SUSPEND )                                            		// Suspended
        {		
#ifdef DEBUG		
            printf( "Enter LPM\r\n" );                                              		// Enter LPM flag for debug
#endif
            while ( XBUS_AUX & bUART0_TX )
            {
			
                ;    																		// Waiting send complete ...
            }		
            SAFE_MOD = 0x55;		
            SAFE_MOD = 0xAA;		
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                               			// Action on USB Bus, or RXD0, will awake the device
            PCON |= PD;                                                           			// LPM
            SAFE_MOD = 0x55;		
            SAFE_MOD = 0xAA;		
            WAKE_CTRL = 0x00;		
        }		
    }		
    else {                                                                         			// Unexpected interrupt
        USB_INT_FG = 0xFF;                                                         			// Reset the interrupt
    }
}

/*******************************************************************************
* Function Name  : static SendKey( char *p)
* Description    : ASCII to key code
* Input          : char *p
* Output         : None
* Return         : None
*******************************************************************************/

static void SendKey ( char *p )
{

	char c = *p;
		
	if( (c >= 'a') && (c <= 'z' )){
		c = c - 'a' + 'A';
	}
	
	if( (c >= 'A') && (c <= 'Z' )){
		HIDKey[2] = c - 'A' + 4;
	}
	else
		if( c >= '1' && c <= '9' )
			HIDKey[2] = c - '1' + 0X1E;
		else
		{
		switch ( c ){
			case '`' :
				HIDKey[0] = 0X08;
				HIDKey[2] = 0X15;
				break;
			case '\\':
				HIDKey[2] = 0x31;
				break;
			case ' ':
				HIDKey[2] = L_SHIFT;
				break;
			case '\r':
				HIDKey[2] = ENTER;
				break;
			case ':':
				HIDKey[0] = 0x02;
				HIDKey[2] = 0x33;
				break;
			case '+':
				HIDKey[0] = 0x00;
				HIDKey[2] = 0x57;
				break;
			case '_':
				HIDKey[0] = 0X02;
				HIDKey[2] = 0X2D;
				break;
			case '/':
				HIDKey[0] = L_CTL + L_ALT;
				HIDKey[2] = 0X16;
				break;
			case '0':
				HIDKey[2] = 0X27;
				break;
			case '.':
				HIDKey[2] = 0X37;
				break;
			case '~':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X05;
				break;
			case '!':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X08;
				break;
			default:
				break;
		}
	}
	
	mDelaymS( 20 );																				//emulate the press down and pop up
	while(FLAG == 0);                                           								// Upload the key
	Enp1IntIn();						
	while(FLAG == 0);   																					
	mDelaymS( 20 );
	HIDKey[0] = 0X00;     						
	HIDKey[2] = 0X00;                                              								// Reset the key
	while(FLAG == 0);                                           						
	Enp1IntIn();			
	while(FLAG == 0); 
}

/*******************************************************************************
* Function Name  : static void UploadData(void)
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void UploadData(void)
{
	UINT8 i;
	for( i = 1; i < 64 ;i ++ )
		HIDCom[i] += rand();
}
/*******************************************************************************
* Function Name  : extern HIDValueHandle( void )
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
// extern void HIDValueHandle( void )
// {
// 	static UINT8 rotateFlag = 0;																// When upload keys complete, upload EP2 data
//   UINT16 KeyData = TouchKeyButton;
//   if(TKEY_CTRL&bTKC_IF)		                                    //query key
//   {
//      KeyData = TKEY_DAT;                                      //
// #ifdef DE_PRINTF
//      printf("B.=  %04x\n",KeyData&0x7FFF);		
// #endif
//   }		
// 	if (rotateFlag &&(KeyData < (TouchKeyButton-100)))	        //100 灵敏度调节					
//   {
// 		HIDCom[0] += 0x01;	
// 		UploadData();

// 		Enp2IntIn();	
// 	}			
// 	else if(rotateFlag == 0)
// 	{				
//         SendKey(pStr);																	     	 // Upload path
// 		pStr++;	
// 		if(*pStr == '\0')			
// 		{
// 			SendKey( "~" );																	     // Upload ALT+B
// 			mDelaymS( 200 );	
// 			rotateFlag = 1;
// 		}		
// 	}	

// }

/**************************** END *************************************/

void usb_send_key (char *p)
{

	char c = *p;
	
  memset(HIDKey, 0, sizeof(HIDKey)); 
		
	if( (c >= 'a') && (c <= 'z' )){
		c = c - 'a' + 'A';
	}
	
	if( (c >= 'A') && (c <= 'Z' )){
		HIDKey[2] = c - 'A' + 4;
	}
	else
		if( c >= '1' && c <= '9' )
			HIDKey[2] = c - '1' + 0X1E;
		else
		{
		switch ( c ){
			case '#':			/* f7 key */
				HIDKey[0] = 0x00;
				HIDKey[2] = 0x40;
				break;

			case '`' :
				HIDKey[0] = 0X08;
				HIDKey[2] = 0X15;
				break;
			case '\\':
				HIDKey[2] = 0x31;
				break;
			case ' ':
				HIDKey[2] = L_SHIFT;
				break;
			case '\r':
				HIDKey[2] = ENTER;
				break;
			case ':':
				HIDKey[0] = 0x02;
				HIDKey[2] = 0x33;
				break;
			case '+':
				HIDKey[0] = 0x000;
				HIDKey[2] = 0x57;
				break;
			case '_':
				HIDKey[0] = 0X02;
				HIDKey[2] = 0X2D;
				break;
			case '/':
				HIDKey[0] = L_CTL + L_ALT;
				HIDKey[2] = 0X16;
				break;
			case '0':
				HIDKey[2] = 0X27;
				break;
			case '.':
				HIDKey[2] = 0X37;
				break;
			case '~':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X05;
				break;
			case '!':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X08;
				break;
			default:
				break;
		}
	}
	
	mDelaymS( 20 );																				//emulate the press down and pop up
	while(FLAG == 0);                                           								// Upload the key
	Enp1IntIn();						
	while(FLAG == 0);   																					
	mDelaymS( 20 );
	HIDKey[0] = 0X00;     						
	HIDKey[2] = 0X00;                                              								// Reset the key
	while(FLAG == 0);                                           						
	Enp1IntIn();			
	while(FLAG == 0); 
}


void usb_send_keys_from_flash(void)
{
	UINT8 hid_report[8] = {0};
	UINT8 read_len = 0;

	// 从flash中读取8字节键值
	read_len = ReadDataFlash(KEY_VALUES_OFFSET, 8, hid_report);
	if (read_len != 8)
	{
		return;
	}

	// memcpy(HIDKey, g_key_values, 8);

	// 发送按键按下报告
	// 将hid_report内容复制到HIDKey
    memcpy(HIDKey, hid_report, 8);

    // 发送按键按下报告
    while(FLAG == 0);
    Enp1IntIn();
    while(FLAG == 0);

	// 延时模拟按下保持
    mDelaymS(20);


    // 松开按键
    memset(HIDKey, 0, 8);
    while(FLAG == 0);
    Enp1IntIn();
    while(FLAG == 0);


}


/**
 * @brief 计算从compound_data[1]开始的len - 1个字节的校验和
 * 
 * @param compound_data 
 * @param len 这里的len指的是从compound_data[2]中提取的长度值
 * @return UINT8 
 */
UINT8 usb_calculate_checksum(UINT8 *compound_data, UINT8 len)
{
	UINT8 checksum = 0;
	UINT8 i;

	for (i = 1; i < len; i++)
	{
		checksum += compound_data[i];
	}

	return checksum;
}

UINT8 usb_check_valid_packet(UINT8 *compound_data, UINT8 len)
{
	/* 注意：接收到的数据compound_data[0]是固定的，0x00，这一位不考虑；实际判断从compound_data[1]开始考虑 */

	/* 协议格式是 帧头（1 byte）+ 长度（1 byte）(计算的是从帧头到校验和的长度（包括）) + OPCODE (1 byte) + DATA（不固定） + Checksum (1 byte) */
	/* 例如：0xAA 0x04 0x01 0x0A  */
	/* 帧头是0xAA，长度是0x04，OPCODE是0x01，没有DATA，checksum是0XAA + 0x04 + 0x01 = 0xB0 */
	/* 判断缓冲区接收到的数据是否是一个合法的数据包 */

	/* 判断流程：1. 判断compound_data[1]是否是一个合法的帧头0xAA */
	/* 			2. 如果是一个合法的帧头，紧接着从compound_data[2]提取这个字节的长度 */
	/* 			3. 根据协议格式计算得到checksum是位于compound_data[x]的x是多少 */
	/* 			4. 再计算这包数据的checksum，与协议里面的checksum大小是否一致：如果一直，那就进入判断OPCODE的环节 */

	UINT8 expected_checksum = 0;
	UINT8 recved_checksum = 0;
	UINT8 length = 0;

	if (compound_data[1] != 0xAA)
	{
		return 0;
	}

	length = compound_data[2];

	expected_checksum = usb_calculate_checksum(compound_data, length);
	recved_checksum = compound_data[length];
	if (expected_checksum != recved_checksum)
	{
		return 0;
	}

	return 1;

}



/**
 * @brief 
 * 
 * @param len 
 */
void compound_process_recv_data(UINT8 len)
{
	/* 响应数据包格式 */
	/* compound_response_data[0] =  0x01 						1byte												*/
	/* compound_response_data[1] =  PACKET_HEADER 				1byte											    */
	/* compound_response_data[2] =  LENGTH 						1byte												*/
	/* compound_response_data[3] =  OPCODE 						1byte												*/
	/* compound_response_data[4] =  ERROCDE 					1byte												*/
	/* compound_response_data[5 ~ x] =  DATA 				    不固定，根据OPCODE不同，DATA内容不同			      */
	/* compound_response_data[x + 1] =  CHECKSUM 				1byte												*/

	if (usb_check_valid_packet(compound_received_data, len))	// 合法数据包
	{
		// 数据包合法，继续处理
		UINT8 opcode = compound_received_data[3];

		switch (opcode)
		{
			case USB_OP_HEARTBEAT:
			{
				// 心跳包:心跳包的组装的data是
				//VERSION_STR[0];
    			//VERSION_STR[2];
    			//VERSION_STR[4];
    			//DEVICE_VID_L;           
    			//DEVICE_VID_H;           
    			//DEVICE_PID_L;           
    			//DEVICE_PID_H;
				// ERRCODE = 0x00

				// 1. 业务逻辑，处理心跳包功能

				UINT8 checksum = 0;
				UINT8 length =  12;
				UINT8 i = 0;

//				g_compound_heartbeat_timer = 0;
				g_compound_heartbeat_flag = 1;
//				if (!first_hb_flag)
//				{
//					first_hb_flag = 1;
//					timer0_register_cb(led_flash_handler);
//				}

				// 2. 组装响应包并发送
				compound_response_data[0] = 0x01;		// 协议固定的，第一个字节得是0x01
				compound_response_data[1] = PACKET_HEADER;
				compound_response_data[2] = length;
				compound_response_data[3] = opcode;
				compound_response_data[4] = 0x00;	// ERRCODE = 0x00

				compound_response_data[5] = VERSION_STR[0];
				compound_response_data[6] = VERSION_STR[2];
				compound_response_data[7] = VERSION_STR[4];
				compound_response_data[8] = DEVICE_VID_L;
				compound_response_data[9] = DEVICE_VID_H;
				compound_response_data[10] = DEVICE_PID_L;
				compound_response_data[11] = DEVICE_PID_H;

				// 计算校验和，计算[1] ~ [11]的校验和
				for (i = 1; i < 12; i++)
				{
					checksum += compound_response_data[i];
				}
				compound_response_data[12] = checksum;

				// 填充剩余的字节为0x00，直到64字节
				memset(&compound_response_data[13], 0, 51);  // 64 - 13 = 51

				// 发送数据
				memcpy(&Ep2Buffer[BUFFER_SIZE], compound_response_data, 64);  
        		UEP2_T_LEN = 64;
        		UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;

				break;
			}
			case USB_OP_WL_LEARNMATCH:
			{
				// 学习匹配
				// 学习匹配没有data
				// ERRCODE = 0x00

				// 1. 业务逻辑，学习匹配无线按键控制USB键盘
				// 拉低P1^6 4s，进入学习模式，然后拉高
				// 首先考虑阻塞可不可以。如果阻塞4s，那么久不能处理其他的数据包，这会直接导致心跳包的处理失败，进而重启USB设备，这是不允许的
				// 所以，必须使用非阻塞方法。想法是利用原有的定时器，原来的定时器是定时10ms，那就是可以在这个主程序里面注册回调函数，然后在回调函数里面定时4s，然后就是主程序里4s后主程序里来一个全局变量置位，
				// 上位机下发学习命令，状态机接收到，立刻回复usb主机开始处理学习，然后进入学习状态：拉低P1.6 4s，然后拉高；这个怎么通过非阻塞的方法实现呢？

				UINT8 checksum = 0;
				UINT8 length = 5;	// header + length + opcode + ERRCODE + checksum
				UINT8 i = 0;


				if (learnmatch_state == LEARNMATCH_STATE_IDLE)
				{
					WL_LEARN_PIN = 0;
					learnmatch_state = LEARNMATCH_STATE_ACTIVE;
					g_learnmatch_timer = 0;
				}

				// 2. 组装响应包并发送
				compound_response_data[0] = 0x01;		// 协议固定的，第一个字节得是0x01
				compound_response_data[1] = PACKET_HEADER;
				compound_response_data[2] = length;
				compound_response_data[3] = opcode;
				compound_response_data[4] = 0x00;	// ERRCODE = 0x00,表示成功

				for (i = 1; i < 5; i++)
				{
					checksum += compound_response_data[i];
				}
				compound_response_data[5] = checksum;
				memset(&compound_response_data[6], 0, 58);  // 64 - 6 = 58

				// 发送数据
				memcpy(&Ep2Buffer[BUFFER_SIZE], compound_response_data, 64);  
				UEP2_T_LEN = 64;
				UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;

				break;
			}
			case USB_OP_WL_CLEANMATCH:
			{
				// 清空匹配
				// 清空匹配没有data
				// ERRCODE = 0x00
				// 清空匹配和学习匹配的区别是，清空匹配是拉低11s，然后拉高
				UINT8 checksum = 0;
				UINT8 length = 5;	// header + length + opcode + ERRCODE + checksum
				UINT8 i = 0;


				if (cleanmatch_state == CLEANMATCH_STATE_IDLE)
				{
					WL_LEARN_PIN = 0;
					cleanmatch_state = CLEANMATCH_STATE_ACTIVE;
					g_cleanmatch_timer = 0;
				}

				// 2. 组装响应包并发送
				compound_response_data[0] = 0x01;		// 协议固定的，第一个字节得是0x01
				compound_response_data[1] = PACKET_HEADER;
				compound_response_data[2] = length;
				compound_response_data[3] = opcode;
				compound_response_data[4] = 0x00;	// ERRCODE = 0x00,表示成功

				for (i = 1; i < 5; i++)
				{
					checksum += compound_response_data[i];
				}
				compound_response_data[5] = checksum;
				memset(&compound_response_data[6], 0, 58);  // 64 - 6 = 58

				// 发送数据
				memcpy(&Ep2Buffer[BUFFER_SIZE], compound_response_data, 64);  
				UEP2_T_LEN = 64;
				UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;

				break;
			}
			case USB_OP_WR_KEYVALUE:		/* 写入键值保存到flash，下一次从写入位置读取来发送键值 */
			{
				//UINT8 valid_keys = 0;
				UINT8 key_values[8] = {0};

				UINT8 saved_key_values[8] = {0};	/* 保存从flash中读取的键值，比较是否与将要写入的数据相同。如果相同，那就不写入了。如果不同，就更新需要写入的键值 */
				UINT8 read_len = 0;
				UINT8 erase_len = 0;
				UINT8 write_len = 0;
				UINT8 i = 0;
				UINT8 length = 13;	// header + length + opcode + ERRCODE + DATA + checksum, 这里的DATA是8字节的从flash中读取的键值
				UINT8 errcode = 0x00;	// success
				UINT8 checksum = 0;

				/* 1. 从协议包里解析出升级的键值，保存到了局部变量key_values里 */
				memcpy(key_values, &compound_received_data[4], 8);
				
				/* 2. 保存到data flash里面 */

				// 写之前先读取一次，比较是否与将要写入的数据相同
				read_len = ReadDataFlash(KEY_VALUES_OFFSET, 8, saved_key_values);
				if (read_len != 8)
				{
					break;
				}

				if (memcmp(key_values, saved_key_values, 8) == 0)
				{
					// 如果相同，那就不写入了
					break;
				}

				// 擦除dataflash对应区域
				erase_len = EraseDataFlash(KEY_VALUES_OFFSET, 8);
				if (erase_len != 8)
				{
					break;
				}

				// 写入新的键值
				write_len = WriteDataFlash(KEY_VALUES_OFFSET, key_values, 8);
				if (write_len != 8)
				{
					break;
				}

				// 写完后再次读取，比较是否写入成功
				read_len = ReadDataFlash(KEY_VALUES_OFFSET, 8, saved_key_values);
				if (read_len != 8)
				{
					break;
				}

				memcpy(g_key_values, saved_key_values, 8);	// 更新全局变量缓存

				// 新的键值写入之后，就需要写一个发送键值的函数了。因为是要考虑到发送组合按键的。

				/* 3. 回复写入成功的响应包 */
				// TODO: 组装写入成功的相应包，注意其他的都是失败的相应包
				compound_response_data[0] = 0x01;		// 协议固定的，第一个字节得是0x01
				compound_response_data[1] = PACKET_HEADER;
				compound_response_data[2] = length;
				compound_response_data[3] = opcode;
				compound_response_data[4] = 0x00;	// ERRCODE = 0x00,表示成功

				// 放入8字节data，即刚写入的键值
				memcpy(&compound_response_data[5], saved_key_values, 8);

				// 计算校验和，从[1]开始到[1+12=13 -1=12],即[1..12]
				for (i = 1; i < (1+length); i++) // i从1到13-1=12
				{
					checksum += compound_response_data[i];
				}
				// 将校验和放在[1+length=14-1=13]位置
				compound_response_data[5+8] = checksum; // 5+8=13，对应索引compound_response_data[13]

				// 清空剩余空间(如果需要，这里数据正好64字节足够)
				memset(&compound_response_data[14], 0, 64-14); 

				// 发送数据
				memcpy(&Ep2Buffer[BUFFER_SIZE], compound_response_data, 64);  
				UEP2_T_LEN = 64;
				UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;

				break;
			}

			default:
			{
				// 未知的OPCODE, 需要回复错误码
				// 未知的OPCODE的data是空的
				// ERRCODE = 0x01



				break;
			}

		}

	}
	else	// 非法数据包 echo模式
	{
        memcpy(compound_response_data, compound_received_data, len);
        memcpy(&Ep2Buffer[BUFFER_SIZE], compound_response_data, len);
        UEP2_T_LEN = len;
        UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
	}


}


