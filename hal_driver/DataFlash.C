/********************************** (C) COPYRIGHT *******************************
* File Name          : DataFlash.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH554 DataFlash字节读写函数定义   
*******************************************************************************/

#include "..\Public\CH554.H"                                                         
#include "..\Public\Debug.H"
#include "DataFlash.H"

/*******************************************************************************
* Function Name  : WriteDataFlash(UINT8 Addr,PUINT8 buf,UINT8 len)
* Description    : DataFlash写
* Input          : UINT8 Addr，PUINT16 buf,UINT8 len
* Output         : None
* Return         : UINT8 i 返回写入长度
*******************************************************************************/
UINT8 WriteDataFlash(UINT8 Addr,PUINT8 buf,UINT8 len)
{
    UINT8 i;
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bDATA_WE;                                                    //使能DataFlash写
    SAFE_MOD = 0;                                                              //退出安全模式	
		ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
    Addr <<= 1;
    for(i=0;i<len;i++)
	  {
        ROM_ADDR_L = Addr + i*2;
        ROM_DATA_L = *(buf+i);			
        if ( ROM_STATUS & bROM_ADDR_OK ) {                                     // 操作地址有效
           ROM_CTRL = ROM_CMD_WRITE;                                           // 写入
        }
        if((ROM_STATUS ^ bROM_ADDR_OK) > 0) return i;                          // 返回状态,0x00=success,  0x02=unknown command(bROM_CMD_ERR)
	  }
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG &= ~bDATA_WE;                                                   //开启DataFlash写保护
    SAFE_MOD = 0;                                                              //退出安全模式	
    return i;		
}

/*******************************************************************************
* Function Name  : ReadDataFlash(UINT8 Addr,UINT8 len,PUINT8 buf)
* Description    : 读DataFlash
* Input          : UINT8 Addr UINT8 len PUINT8 buf
* Output         : None
* Return         : UINT8 i 返回写入长度
*******************************************************************************/
UINT8 ReadDataFlash(UINT8 Addr,UINT8 len,PUINT8 buf)
{
    UINT8 i;
    ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
    Addr <<= 1;
    for(i=0;i<len;i++){
	  ROM_ADDR_L = Addr + i*2;                                                   //Addr必须为偶地址
	  ROM_CTRL = ROM_CMD_READ;
//     if ( ROM_STATUS & bROM_CMD_ERR ) return( 0xFF );                        // unknown command
	  *(buf+i) = ROM_DATA_L;
		}
    return i;
}


/*******************************************************************************
* Function Name  : EraseDataFlash(UINT8 Addr, UINT8 len)
* Description    : 擦除DataFlash指定区域
* Input          : UINT8 Addr：起始地址, UINT8 len：要擦除的长度(字节数) 
* Output         : None
* Return         : UINT8 erasedLen: 实际擦除的长度
*******************************************************************************/
UINT8 EraseDataFlash(UINT8 Addr, UINT8 len)
{
    UINT8 i;
    UINT8 erasedLen = 0;
    
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bDATA_WE;                                                    //使能DataFlash写
    SAFE_MOD = 0;                                                              //退出安全模式	
    ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
    
    Addr <<= 1;                                                                //将实际地址转为偶地址
    for(i=0; i<len; i++)
    {
        ROM_ADDR_L = Addr + i*2;
        ROM_DATA_L = 0xFF;                                                     //连续写入0xFF			
        if ( ROM_STATUS & bROM_ADDR_OK ) {                                     // 操作地址有效
           ROM_CTRL = ROM_CMD_WRITE;                                           // 启动写操作
           erasedLen++;                                                        //擦除长度加1
        } else {
           break;                                                              //地址无效时退出擦除
        }
    }
    
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG &= ~bDATA_WE;                                                   //禁止DataFlash写
    SAFE_MOD = 0;                                                              //退出安全模式
    
    return erasedLen;		
}
