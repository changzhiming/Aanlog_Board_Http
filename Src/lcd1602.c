
#include "lcd1602.h"
void DATAOUT(u8 x)
{
	HAL_GPIO_WritePin(LCD_D0_GPIO_Port,  LCD_D0_Pin, (GPIO_PinState)((x >> 0) & 0x01));
	HAL_GPIO_WritePin(LCD_D1_GPIO_Port,  LCD_D1_Pin, (GPIO_PinState)((x >> 1) & 0x01));
	HAL_GPIO_WritePin(LCD_D2_GPIO_Port,  LCD_D2_Pin, (GPIO_PinState)((x >> 2) & 0x01));
	HAL_GPIO_WritePin(LCD_D3_GPIO_Port,  LCD_D3_Pin, (GPIO_PinState)((x >> 3) & 0x01));
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port,  LCD_D4_Pin, (GPIO_PinState)((x >> 4) & 0x01));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port,  LCD_D5_Pin, (GPIO_PinState)((x >> 5) & 0x01));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port,  LCD_D6_Pin, (GPIO_PinState)((x >> 6) & 0x01));
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port,  LCD_D7_Pin, (GPIO_PinState)((x >> 7) & 0x01));
}
void LCD1602_Wait_Ready(void)
{
	u8 sta;
	
	DATAOUT(0xff);
	LCD_RS_Clr();
	LCD_RW_Set();
	do
	{
		LCD_EN_Set();
		HAL_Delay(5);
 		sta = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);//��ȡ״̬��
		LCD_EN_Clr();
	}while(sta & 0x80);//bit7����1��ʾҺ����æ���ظ����ֱ�������0Ϊֹ
}

/* ��LCD1602Һ��д��һ�ֽ����cmd-��д������ֵ */
void LCD1602_Write_Cmd(u8 cmd)
{
	LCD1602_Wait_Ready();
	LCD_RS_Clr();
	LCD_RW_Clr();
	DATAOUT(cmd);
	LCD_EN_Set();
	LCD_EN_Clr();
}

/* ��LCD1602Һ��д��һ�ֽ����ݣ�dat-��д������ֵ */
void LCD1602_Write_Dat(u8 dat)
{
	LCD1602_Wait_Ready();
	LCD_RS_Set();
	LCD_RW_Clr();
	DATAOUT(dat);
	LCD_EN_Set();
	LCD_EN_Clr();
}

/* ���� */
void LCD1602_ClearScreen(void)
{
	LCD1602_Write_Cmd(0x01);
}

/* ������ʾRAM��ʼ��ַ���༴���λ�ã�(x,y)-��Ӧ��Ļ�ϵ��ַ����� */
void LCD1602_Set_Cursor(u8 x, u8 y)
{
	u8 addr;
	
	if (y == 0)
		addr = 0x00 + x;
	else
		addr = 0x40 + x;
	LCD1602_Write_Cmd(addr | 0x80);
}

/* ��Һ������ʾ�ַ�����(x,y)-��Ӧ��Ļ�ϵ���ʼ���꣬str-�ַ���ָ�� */
void LCD1602_Show_Str(u8 x, u8 y, u8 *str)
{
	LCD1602_Set_Cursor(x, y);
	while(*str != '\0')
	{
		LCD1602_Write_Dat(*str++);
	}
}

/* ��ʼ��1602Һ�� */
void LCD1602_Init(void)
{
	LCD1602_Write_Cmd(0x38);	//16*2��ʾ��5*7����8λ���ݿ�
	LCD1602_Write_Cmd(0x0c);	//����ʾ�����ر�
	LCD1602_Write_Cmd(0x06);	//���ֲ�������ַ�Զ�+1
	LCD1602_Write_Cmd(0x01);	//����
}

