#include "platform.h"
#include "w25qxx.h"
#include "stm32f4xx_hal.h"
#include "flash_if.h"
#include "string.h"
#include "lcd1602.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbutils.h"
#include "mbcrc.h"
#include "lwip.h"
/* ----------------------- system function ----------------------------------*/
static u8 DA_OUT(u8 ch,u8 rng,u8 data);
static void InitTLC5620(void);
static void Save_Out_State(void);
static void Get_Out_Addr(void);
/* ----------------------- system var ----------------------------------*/
static char device_name[30] = "Analog_board_V2_4";
/*
2 热插拔实现
3 增加定时检测lwip内存
*/
SYSINFO_TypeDef SysConfigure;        //系统配置信息

static uint32_t lastTick = 0;         //时间检测
static uint32_t buf_size = 0;         //数据包位置
static uint8_t  aucRtuBuf[256];       //透传Modbus rtu buf
static uint8_t  temp_value = 0;

static uint32_t out_save_addr = FLASH_ADDR_OUT_STATA;       //输出保存地址
static uint32_t data_addr = FLASH_DATA_ADDR;
static uint8_t  wwdg_time = 0;      //定时喂狗时间
static uint16_t ADC_Value[800] = {0};         //adc
static uint8_t reboot = 0;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern DAC_HandleTypeDef hdac;
extern RTC_HandleTypeDef hrtc;

#define huart485 huart3
#define huart_debug huart1
#define USART485 USART3
/* ----------------------- Holding register Defines ------------------------------------------*/

#define REG_HOLDING_START 1U
#define REG_HOLDING_NREGS 48

static unsigned short usRegHoldingStart = REG_HOLDING_START;
static unsigned short usRegHoldingBuf[REG_HOLDING_NREGS]={0};

/* ----------------------- input register Defines ------------------------------------------*/
#define REG_INPUT_START 1U
#define REG_INPUT_NREGS 1

static unsigned short usRegInputStart = REG_INPUT_START;
static unsigned short usRegInputBuf[REG_INPUT_NREGS]={0};


/* ----------------------- coils register Defines ------------------------------------------*/
#define REG_COILS_START     1U
#define REG_COILS_SIZE      1

static unsigned char ucRegCoilsBuf[REG_COILS_SIZE % 8 == 0 ? REG_COILS_SIZE / 8 : REG_COILS_SIZE / 8 +1]={0x00};	  //数量低于8个还需要验证一下，是否需要加1呢。

/* ----------------------- discrete register Defines ------------------------------------------*/
#define REG_DISC_START     1U
#define REG_DISC_SIZE      1

static unsigned char ucRegDiscBuf[REG_DISC_SIZE % 8 == 0 ? REG_DISC_SIZE / 8 : REG_DISC_SIZE / 8 +1] = { 0x00};	   //数量8的整数倍。

//更新继电器状态
static void updateRegCoilsCB()
{

}

//更新开关量输入状态
static void updateRegDiscreteCB()
{
}
//更新输出状态 0.00精度  24个
static void updateRegHoldingCB()
{
	int index = 24;
	HAL_GPIO_WritePin(D_OUT1_GPIO_Port,  D_OUT1_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT2_GPIO_Port,  D_OUT2_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT3_GPIO_Port,  D_OUT3_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT4_GPIO_Port,  D_OUT4_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	
	for(; index < 32; index++)
	{
		DA_OUT(index - 28 , 2, usRegHoldingBuf[index] / 100.0 * 25);
	}
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, usRegHoldingBuf[index++] / 100.0 * 3.3 / 20  * (4095 / 3.3));
}
//更新输入状态 0.00精度  24个
static void updateRegInputCB()
{
	uint8_t * chandle = (uint8_t *)&SysConfigure.In_Chandle_9;
	
	int index = 0;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN8_GPIO_Port, D_IN8_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN7_GPIO_Port, D_IN7_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN6_GPIO_Port, D_IN6_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN5_GPIO_Port, D_IN5_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN4_GPIO_Port, D_IN4_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN3_GPIO_Port, D_IN3_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN2_GPIO_Port, D_IN2_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN1_GPIO_Port, D_IN1_Pin) ? 0 : 1;
	
	volatile int adc_value = 0;
	int i, size;
	for(i = index; i < 16; i++)
	{
		adc_value = 0;
		for(size = 0; size < 100; size++)
		{
			adc_value = adc_value + ADC_Value[size * 8 + i - 8];
		}
		usRegHoldingBuf[i] = (adc_value * 3.3 / 4096)* 3.0769;
	}
	for(i = index; i < 16; i++)
	{
		switch(*(chandle + i - 8))
		{
			case 0:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] > 500 ? 0 : 1;
				break;
			case 1:
				usRegHoldingBuf[i] = usRegHoldingBuf[i];
				break;
			case 2:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] * 2;
				break;
			case 3:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] > 900 ? 0 : 1000 * 20 * usRegHoldingBuf[i] / 100 / (29.2 - 3 * usRegHoldingBuf[i] / 100);    //   100 * (100 * usRegHoldingBuf[i] / 100 - 144.5) / (14.45 - 20 * usRegHoldingBuf[i] / 100); //;usRegHoldingBuf[i] > 500 ? 0 : 10.0 * usRegHoldingBuf[i] / (14.45 - usRegHoldingBuf[i] / 100.0);
				break;
		}
	}
}
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
                /* Pass current register values to the protocol stack. */
            case MB_REG_READ:
							  updateRegInputCB();
                while( usNRegs > 0 )
                {
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] &
                                           0xFF );
                    iRegIndex++;
                    usNRegs--;
                }
                break;

                /* Update current register values with new values from the
                 * protocol stack. */
            case MB_REG_WRITE:
                while( usNRegs > 0 )
                {
                    usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                    usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                    iRegIndex++;
                    usNRegs--;
                }
								updateRegHoldingCB();
								Save_Out_State();
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
				updateRegInputCB();       //更新输入寄存器状态
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,( unsigned char )( iNCoils > 8 ? 8 : iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset, ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ), *pucRegBuffer++ );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
								updateRegCoilsCB();
								Save_Out_State();
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
				updateRegDiscreteCB(); //更新开关量输入值;
        usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                ( unsigned char )( iNDiscrete >
                                                   8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

//保存输出状态
static void Save_Out_State(void)
{ 
	if(out_save_addr % 4096 < sizeof(usRegHoldingBuf))
	{
		W25QXX_Erase_Sector(out_save_addr / 4096);         //擦除这个扇区
	}
	if(data_addr % 4096 < sizeof(data_addr))
	{
		W25QXX_Erase_Sector(data_addr / 4096);         //擦除这个扇区
	}
	
	W25QXX_Write((uint8_t *)usRegHoldingBuf, out_save_addr, sizeof(usRegHoldingBuf));
	W25QXX_Write((uint8_t *)&out_save_addr, data_addr, sizeof(out_save_addr));
	data_addr += sizeof(data_addr);
	out_save_addr += sizeof(usRegHoldingBuf);
	
	if(out_save_addr + sizeof(usRegHoldingBuf) >= FLASH_ADDR_END)
	{
		out_save_addr = FLASH_ADDR_OUT_STATA;
	}
	if(data_addr + sizeof(data_addr) >= FLASH_ADDR_OUT_STATA)
	{
		data_addr = FLASH_DATA_ADDR;
	}
}
//还原输出状态
static void Get_Out_Addr(void)
{
	volatile int i = 0;
	uint32_t value = 0;

	while(1)
	{
		if(FLASH_DATA_ADDR + i* 4 >= FLASH_ADDR_OUT_STATA)
		{
			data_addr = FLASH_DATA_ADDR;
			out_save_addr = FLASH_ADDR_OUT_STATA;
			break;
		}
		W25QXX_Read((uint8_t *)&value, FLASH_DATA_ADDR + (i++)* 4, sizeof(value));
		if(value == 0xFFFFFFFF)
		{
			if(i < 2)
			{
				data_addr = FLASH_DATA_ADDR;
				out_save_addr = FLASH_ADDR_OUT_STATA;
				break;
			}
			data_addr = FLASH_DATA_ADDR + (i - 2) * 4;
			
			W25QXX_Read((uint8_t *)&out_save_addr, data_addr, sizeof(out_save_addr));
			break;
		}
	}
	if(data_addr != FLASH_DATA_ADDR)
	{
		W25QXX_Read((uint8_t *)usRegHoldingBuf, out_save_addr, sizeof(usRegHoldingBuf));
		out_save_addr += sizeof(usRegHoldingBuf);
		
		updateRegHoldingCB();
	}
}

void RS485_Send(uint8_t * send_buf, uint16_t size)
{
	//先读取数据
	while(__HAL_UART_GET_FLAG(&huart485, UART_FLAG_RXNE) == SET) {
		uint8_t value;  HAL_UART_Receive(&huart485, &value, 1, 0xFF);
	}
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart485, send_buf, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART485)
	{
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(&huart485, &temp_value, 1);       //重新启动接收数据
	}
}

void RS485_Recive(uint8_t ** recive_buf, uint8_t *size)
{
	if(((HAL_GetTick() - lastTick) >= 3) && buf_size >= 4)
	{
		*size = buf_size;
		*recive_buf = &aucRtuBuf[0];
		buf_size = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART485)
	{
		if(((HAL_GetTick() - lastTick) > 5 && buf_size >= 4) || buf_size > 255)
		{
			buf_size = 0;
		}
		lastTick = HAL_GetTick();
		aucRtuBuf[buf_size++] = temp_value;
		
		HAL_UART_Receive_IT(&huart485, &temp_value, 1);
	}
}


int fputc(int ch, FILE *f)
{
//  RS485_Send((uint8_t *)&ch, 1);
//	HAL_Delay(1);
  return ch;
}

int fgetc(FILE *f)
{
	uint8_t  ch;     
	//HAL_UART_Receive(&huart_debug,(uint8_t *)&ch, 1, 0xFFFF);
	return  ch;
}

#define TLC5620_SCLK(SET) HAL_GPIO_WritePin(DA_CLK_GPIO_Port, DA_CLK_Pin, (GPIO_PinState)(SET))	  // TLC5620时钟管脚
#define TLC5620_SDAA(SET) HAL_GPIO_WritePin(DA_DATA_GPIO_Port, DA_DATA_Pin, (GPIO_PinState)(SET))		// TLC5620数据管教
#define TLC5620_LOAD(SET) HAL_GPIO_WritePin(DA_LOAD_GPIO_Port, DA_LOAD_Pin, (GPIO_PinState)(SET))	  // TLC5620 LOAD管教
#define TLC5620_LDAC(SET) HAL_GPIO_WritePin(DA_LD_GPIO_Port, DA_LD_Pin, (GPIO_PinState)(SET))	// TLC5620 LDAC管教

#define TLC5620_OUTA 0x0000
#define TLC5620_OUTB 0x4000
#define TLC5620_OUTC 0x8000
#define TLC5620_OUTD 0xc000

#define TLC5620_OUT_one    0x0000
#define TLC5620_OUT_double 0x2000

static void InitTLC5620(void)
{
	TLC5620_SCLK(0);
	TLC5620_SDAA(0);
	TLC5620_LOAD(1);
	TLC5620_LOAD(1);
}
//TLC 5620电压输出
//
static void  TLC5620_Conversion (u16 tlc5620_data)
{
    u8   m=0;
    u16  n;
    for(; m<0x0b; m++)
    {
        TLC5620_SCLK(1);
        n=tlc5620_data;
        n=n&0x8000;
        if(n==0x8000)
            TLC5620_SDAA(1);
        else
            TLC5620_SDAA(0);
        TLC5620_SCLK(0);
        tlc5620_data<<=1;
    }
    TLC5620_LOAD(0);
    TLC5620_LOAD(1);
    TLC5620_LDAC(0);
    TLC5620_LDAC(1);
}
//ch = 0 TLC5620_OUTA;1 TLC5620_OUTB;2 TLC5620_OUTC;3 TLC5620_OUTD;other err;
//rng= 0 TLC5620_OUT_one;other TLC5620_OUT_double;
//VO(DACA|B|C|D)+REF*(CODE/256)*(1+RNG)
static u8 DA_OUT(u8 ch,u8 rng,u8 data)
{
    u16 out_data=0;
    
    out_data |= data;
    out_data <<= 5;
    if(ch == 0){
        out_data |= TLC5620_OUTA;
    }else if(ch == 1){
        out_data |= TLC5620_OUTB;
    }else if(ch == 2){
        out_data |= TLC5620_OUTC;
    }else if(ch == 3){
        out_data |= TLC5620_OUTD;
    }else{
        return 0;
    }
    
    if(rng == 1){
        out_data |= TLC5620_OUT_one;
    }else if(rng == 2){
        out_data |= TLC5620_OUT_double;
    }else{
        out_data |= TLC5620_OUT_one;
    }
    
    TLC5620_Conversion(out_data);
    return 1;
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
	if(wwdg_time <= 200)
	{
		HAL_WWDG_Refresh(hwwdg);
		wwdg_time++;
	}
}

void WWDG_Refresh(void)
{
	wwdg_time = 0;
	if(reboot > 0 && reboot++ == 200)
		Soft_Reset();
}

void Soft_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}
//http
const char* Upload_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	 return "/success.shtml";
}

const char* GetSet_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
		return "/set.shtml";
}

const char* Reboot_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	  reboot = 1;
		return "/success.shtml";
}
const char* Clear_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
		ClearSysInfo();
		return "/success.shtml";
}
const char* Set_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	int i = 0;
	static int buf[10] = {0};
	for (i = 0; i < iNumParams; i++)
	{
		 //ip
		 if (strcmp(pcParam[i] , "ip") == 0)
     {
				sscanf(pcValue[i],"%d.%d.%d.%d", &buf[0], &buf[1], &buf[2], &buf[3]);
				SysConfigure.IP[0] = buf[0];
				SysConfigure.IP[1] = buf[1];
				SysConfigure.IP[2] = buf[2];
				SysConfigure.IP[3] = buf[3];
     }   

		 if (strcmp(pcParam[i] , "mask") == 0)
     {
				sscanf(pcValue[i],"%d.%d.%d.%d", &buf[0], &buf[1], &buf[2], &buf[3]);
				SysConfigure.Net_Mask[0] = buf[0];
				SysConfigure.Net_Mask[1] = buf[1];
				SysConfigure.Net_Mask[2] = buf[2];
				SysConfigure.Net_Mask[3] = buf[3];
     } 
		 if (strcmp(pcParam[i] , "mac") == 0)
     {
				sscanf(pcValue[i],"%02X.%02X.%02X.%02X.%02X.%02X", &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5]);
				SysConfigure.MACAddr[0] = buf[0];
				SysConfigure.MACAddr[1] = buf[1];
				SysConfigure.MACAddr[2] = buf[2];
				SysConfigure.MACAddr[3] = buf[3];
				SysConfigure.MACAddr[4] = buf[4];
				SysConfigure.MACAddr[5] = buf[5];
     }
		 if (strcmp(pcParam[i] , "addr") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.Modbus_Addr = buf[0];
     } 
			if (strcmp(pcParam[i] , "band") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.RS485_BaudRate = buf[0];
     }
		 if (strcmp(pcParam[i] , "stop") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.RS485_STOPBITS = buf[0];
     }
		 if (strcmp(pcParam[i] , "length") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.RS485_WordLength = buf[0];
     }
		 if (strcmp(pcParam[i] , "parity") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.RS485_Parity = buf[0];
     }
		 if (strcmp(pcParam[i] , "com9") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_9 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com10") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_10 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com11") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_11 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com12") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_12 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com13") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_13 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com14") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_14 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com15") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_15 = buf[0];
     }
		 if (strcmp(pcParam[i] , "com16") == 0)
     {
				sscanf(pcValue[i],"%d", &buf[0]);
				SysConfigure.In_Chandle_16 = buf[0];
     }
	}   
	//写入数据
	while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)&SysConfigure, sizeof(SysConfigure)) != 0 ){}
	return "/success.shtml";
}
const char* Login_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	int i = 0;
	for (i = 0; i < iNumParams; i++)
	{
		if (strcmp(pcParam[i] , "userName") == 0) {
			if(strcmp(pcValue[i], "root") != 0)
				return "";
		}
		if (strcmp(pcParam[i] , "passWord") == 0) {
			if(strcmp(pcValue[i], "hkzk") != 0)
				return "";
		}
	}
	return "/success.shtml";
}
const char* GetData_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	updateRegInputCB();
	return "/data.shtml";
}

const char* SetData_CGI_Handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
	int i = 0 , j = 0;
	int tempValue = 0;
	char tempchar[20] ;
	
	for (i = 0; i < iNumParams; i++)
	{
		for( j = 0; j < 13; j++)
		{
			sprintf(tempchar, "outcom%d", j + 1);
			if (strcmp(pcParam[i] , tempchar) == 0) {
				sscanf(pcValue[i]," %d", &tempValue);
				usRegHoldingBuf[24 + j] = tempValue;
			}
		}
	}
	updateRegHoldingCB();
	Save_Out_State();
	return "/success.shtml";
}

static const tCGI ppcURLs[]= //cgi程序
{
    {"/update.cgi",Upload_CGI_Handler},
		{"/getSet.cgi",GetSet_CGI_Handler},
		{"/reboot.cgi",Reboot_CGI_Handler},
		{"/clear.cgi",Clear_CGI_Handler},
		{"/set.cgi",Set_CGI_Handler},
		{"/login.cgi",Login_CGI_Handler},
		{"/getData.cgi",GetData_CGI_Handler},
		{"/setData.cgi",SetData_CGI_Handler},
};

/* we will use character "t" as tag for CGI */
static const char *TAGS[]=  //SSI的Tag
{   
  "ip",
  "mask",
  "mac", 
	"version",
	"addr",
	"band",
	"stop",
	"length",
	"parity",
	"com9",
	"com10",
	"com11",
	"com12",
	"com13",
	"com14",
	"com15",
	"com16",
	"incom1",  //17
	"incom2",
	"incom3",
	"incom4",
	"incom5",
	"incom6",
	"incom7",
	"incom8",
	"incom9",
	"incom10",
	"incom11",
	"incom12",
	"incom13",
	"incom14",
	"incom15",
	"incom16", 
	"outcom1", //33
	"outcom2",
	"outcom3",
	"outcom4",
	"outcom5",
	"outcom6",
	"outcom7",
	"outcom8",
	"outcom9",//41
};

static u16_t Handler(int iIndex, char *pcInsert, int iInsertLen)
{
	switch(iIndex)
	{
		case 0:
			sprintf(pcInsert, "%d.%d.%d.%d", SysConfigure.IP[0], SysConfigure.IP[1],SysConfigure.IP[2], SysConfigure.IP[3]);
			break;
		case 1:
			sprintf(pcInsert, "%d.%d.%d.%d", SysConfigure.Net_Mask[0], SysConfigure.Net_Mask[1],SysConfigure.Net_Mask[2], SysConfigure.Net_Mask[3]);
			break;
		case 2:
			sprintf(pcInsert, "%02X.%02X.%02X.%02X.%02X.%02X", SysConfigure.MACAddr[0], SysConfigure.MACAddr[1],SysConfigure.MACAddr[2], SysConfigure.MACAddr[3],
			SysConfigure.MACAddr[4], SysConfigure.MACAddr[5]);
			break;
		case 3:
			sprintf(pcInsert, "%s", device_name);
			break;
		case 4:
			sprintf(pcInsert, "%d", SysConfigure.Modbus_Addr);
			break;
		case 5:
			sprintf(pcInsert, "%d", SysConfigure.RS485_BaudRate);
			break;
		case 6:
			sprintf(pcInsert, "%d", SysConfigure.RS485_STOPBITS);
			break;
		case 7:
			sprintf(pcInsert, "%d", SysConfigure.RS485_WordLength);
			break;
		case 8:
			sprintf(pcInsert, "%d", SysConfigure.RS485_Parity);
			break;
		case 9:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_9);
			break;
		case 10:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_10);
			break;
		case 11:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_11);
			break;
		case 12:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_12);
			break;
		case 13:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_13);
			break;
		case 14:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_14);
			break;
		case 15:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_15);
			break;
		case 16:
			sprintf(pcInsert, "%d", SysConfigure.In_Chandle_16);
			break;
  }
	if(iIndex >=17 && iIndex < 33)
		sprintf(pcInsert, "%d", usRegHoldingBuf[iIndex - 17]);
	if(iIndex >=33 && iIndex < 42)
		sprintf(pcInsert, "%d", usRegHoldingBuf[iIndex - 9]);
	
  return strlen(pcInsert);
}

void my_http_init(void)
{
	httpd_init();
	
  http_set_ssi_handler(Handler, (char const **)TAGS, (sizeof(TAGS) / sizeof(char *)));
	http_set_cgi_handlers(ppcURLs, (sizeof(ppcURLs) / sizeof(tCGI)));
}

void SysConfig(void )
{
	//端口初始化
	HAL_GPIO_WritePin(D_EN1_GPIO_Port,  D_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN1_GPIO_Port,  A_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN2_GPIO_Port,  D_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN2_GPIO_Port,  A_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN3_GPIO_Port,  D_EN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN3_GPIO_Port,  A_EN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN4_GPIO_Port,  D_EN4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN4_GPIO_Port,  A_EN4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN5_GPIO_Port,  D_EN5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN5_GPIO_Port,  A_EN5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN6_GPIO_Port,  D_EN6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN6_GPIO_Port,  A_EN6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN7_GPIO_Port,  D_EN7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN7_GPIO_Port,  A_EN7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN8_GPIO_Port,  D_EN8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN8_GPIO_Port,  A_EN8_Pin, GPIO_PIN_RESET);
	
	if(SysConfigure.In_Chandle_9 == 0 || SysConfigure.In_Chandle_9 == 3) 
		HAL_GPIO_WritePin(D_EN1_GPIO_Port,  D_EN1_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_9 == 2) 
		HAL_GPIO_WritePin(A_EN1_GPIO_Port,  A_EN1_Pin, GPIO_PIN_SET);

	if(SysConfigure.In_Chandle_10 == 0 || SysConfigure.In_Chandle_10 == 3)
		HAL_GPIO_WritePin(D_EN2_GPIO_Port,  D_EN2_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_10 == 2)
		HAL_GPIO_WritePin(A_EN2_GPIO_Port,  A_EN2_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_11 == 0 || SysConfigure.In_Chandle_11 == 3)
		HAL_GPIO_WritePin(D_EN3_GPIO_Port,  D_EN3_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_11 == 2)
		HAL_GPIO_WritePin(A_EN3_GPIO_Port,  A_EN3_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_12 == 0 || SysConfigure.In_Chandle_12 == 3)
		HAL_GPIO_WritePin(D_EN4_GPIO_Port,  D_EN4_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_12 == 2)
		HAL_GPIO_WritePin(A_EN4_GPIO_Port,  A_EN4_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_13 == 0 || SysConfigure.In_Chandle_13 == 3)
		HAL_GPIO_WritePin(D_EN5_GPIO_Port,  D_EN5_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_13 == 2)
		HAL_GPIO_WritePin(A_EN5_GPIO_Port,  A_EN5_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_14 == 0 || SysConfigure.In_Chandle_14 == 3)
		HAL_GPIO_WritePin(D_EN6_GPIO_Port,  D_EN6_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_14 == 2)
		HAL_GPIO_WritePin(A_EN6_GPIO_Port,  A_EN6_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_15 == 0 || SysConfigure.In_Chandle_15 == 3)
		HAL_GPIO_WritePin(D_EN7_GPIO_Port,  D_EN7_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_15 == 2)
		HAL_GPIO_WritePin(A_EN7_GPIO_Port,  A_EN7_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_16 == 0 || SysConfigure.In_Chandle_16 == 3)
		HAL_GPIO_WritePin(D_EN8_GPIO_Port,  D_EN8_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_16 == 2)
		HAL_GPIO_WritePin(A_EN8_GPIO_Port,  A_EN8_Pin, GPIO_PIN_SET);
	
}
//init
void PLATFORM_Init(void )
{
	W25QXX_Init();
	
	memset((u8*)&SysConfigure, 0, sizeof(SysConfigure));
  memcpy((void *)&SysConfigure, (void *)FLASH_ADDR_CONFIG, sizeof(SysConfigure));//读取所有配置信息
	
	if(SysConfigure.Sysinfo_errror != 0x55) {
		SysConfigure.Modbus_Addr = 1;
		SysConfigure.IP[0] = 192;
		SysConfigure.IP[1] = 168;
		SysConfigure.IP[2] = 1;
		SysConfigure.IP[3] = 30;
		
		SysConfigure.Net_Mask[0] = 255;
		SysConfigure.Net_Mask[1] = 255;
		SysConfigure.Net_Mask[2] = 255;
		SysConfigure.Net_Mask[3] = 0;
		
		SysConfigure.MACAddr[0] = 0x00;
		SysConfigure.MACAddr[1] = 0x80;
		SysConfigure.MACAddr[2] = 0xE1;
		SysConfigure.MACAddr[3] = 0x00;
		SysConfigure.MACAddr[4] = 0x00;
		SysConfigure.MACAddr[5] = 0x11;
		
		SysConfigure.RS485_BaudRate = 9600;
		SysConfigure.RS485_STOPBITS = 1;
		SysConfigure.RS485_WordLength = 8;
		SysConfigure.RS485_Parity = 0;
		SysConfigure.APP_Update = 0;
		SysConfigure.APP_Size = 0;
		SysConfigure.APP_Version = 0;
		//输入通道
		SysConfigure.In_Chandle_9 = 0;
		SysConfigure.In_Chandle_10 = 0;
		SysConfigure.In_Chandle_11 = 0;
		SysConfigure.In_Chandle_12 = 0;
		SysConfigure.In_Chandle_13 = 0;
		SysConfigure.In_Chandle_14 = 0;
		SysConfigure.In_Chandle_15 = 0;
		SysConfigure.In_Chandle_16 = 0;
		
		SysConfigure.Sysinfo_errror = 0x55;
		
		while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)&SysConfigure, sizeof(SysConfigure)) != 0 ){}
	}

	lastTick = HAL_GetTick();
	//485初始化
  huart485.Init.BaudRate = SysConfigure.RS485_BaudRate;
  huart485.Init.WordLength = SysConfigure.RS485_WordLength == 8 ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
  huart485.Init.StopBits = SysConfigure.RS485_STOPBITS == 1 ? UART_STOPBITS_1 : UART_STOPBITS_2;
	
	if(SysConfigure.RS485_Parity == 0)  {
		huart485.Init.Parity = UART_PARITY_NONE;
	} else if(SysConfigure.RS485_Parity == 1){
		huart485.Init.Parity = UART_PARITY_EVEN;
	} else {
		huart485.Init.Parity = UART_PARITY_ODD;
	}
  if (HAL_UART_Init(&huart485) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	HAL_UART_Receive_IT(&huart485, &temp_value, 1);        //开启485
	
	SysConfig();   //根据配置文件初始化采集类型
	//ADC初始化
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_Value, 800);   //开启DMA
	//dac init
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	
	InitTLC5620();
	
	//io init
	Get_Out_Addr();
	//lcd init
	LCD1602_Init();
	//UpdateLcdShow();
}
void UpdateLcdShow(void)
{
	static char str[20];
	sprintf(str, "%d.%d.%d.%d", SysConfigure.IP[0], SysConfigure.IP[1], SysConfigure.IP[2], SysConfigure.IP[3]);
	LCD1602_Show_Str(0, 0, (u8 *)str);
	//
	sprintf(str, "%d:%d:%d:%d:%d:%d", SysConfigure.RS485_BaudRate, SysConfigure.RS485_STOPBITS, SysConfigure.RS485_WordLength, SysConfigure.RS485_Parity, SysConfigure.Modbus_Addr, SysConfigure.APP_Version);
	LCD1602_Show_Str(0, 1, (u8 *)str);
}

void UpdateInOutShow(void)
{
	static char str[20];
	sprintf(str, "%d%d%d%d%d%d%d%d", (ucRegDiscBuf[0] >> 7) & 0x01, (ucRegDiscBuf[0] >> 6) & 0x01, (ucRegDiscBuf[0] >> 5) & 0x01, (ucRegDiscBuf[0] >> 4) & 0x01,
	(ucRegDiscBuf[0] >> 3) & 0x01, (ucRegDiscBuf[0] >> 2) & 0x01, (ucRegDiscBuf[0] >> 1) & 0x01,(ucRegDiscBuf[0] >> 0) & 0x01);
	LCD1602_Show_Str(0, 0, (u8 *)str);
	sprintf(str, "%d%d%d%d", (ucRegCoilsBuf[0] >> 2) & 0x01, (ucRegCoilsBuf[0] >> 1) & 0x01, (ucRegCoilsBuf[0] >> 0) & 0x01, (ucRegCoilsBuf[0] >> 3) & 0x01);
	LCD1602_Show_Str(0, 0, (u8 *)str);
}

void ClearSysInfo(void )
{
	static uint8_t temp[sizeof(SysConfigure)] = {0};
	while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)temp, sizeof(SysConfigure)) != 0){}
}

