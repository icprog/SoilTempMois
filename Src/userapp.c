
#include <string.h>
#include "userapp.h"
#include "usart.h"

uint8_t Rs485Addr = 0;

/*
*RS485： 初始化485
*参数：  无
*返回值：无
*/
void Rs485Init(void)
{
	RS485_TO_RX(  );
	Rs485Addr = 0x02;
}

/*
*Rs485Handler： Rs485数据处理
*参数：  			  无
*返回值：			  无
*/
void Rs485Handler(void)
{
	if(UART_RX_LPUART1.Rx_State  && (HAL_GetTick(  ) - UART_RX_LPUART1.Rxtime > 50))
	{
		UART_RX_LPUART1.Rx_State = false;
		Rs485RevceHandle(  );
	}
	else
		__WFI();
}

/*
*Rs485RevceHandle： 初始化485
*参数：  			 			无
*返回值：			 			无
*/
void Rs485RevceHandle(void)
{
	if(CalcCRC16(UART_RX_LPUART1.USART_RX_BUF,UART_RX_LPUART1.USART_RX_Len) == 0)
	{		
		///判断Rs485功能码
		switch(UART_RX_LPUART1.USART_RX_BUF[1])
		{
			case 0x03:  ///1：广播处理 0xfe	0x03	0x04	0x00	0x00	0x00	0x00	0xf5	0x3c
					if(UART_RX_LPUART1.USART_RX_BUF[0] == 0xFE)
					{
						///0xfe	0x03	0x04	Rs485Addr	0x00	0x00	0x00	0xf4	0x84
							RS485_TO_TX(  );

					}
					else if(UART_RX_LPUART1.USART_RX_BUF[0] == Rs485Addr) ///2：获取数据指令：温度、湿度
					{
						///0x02	0x03	0x04	0xff	0xdd	0x01	0x64	0x69	0x66 (温度、湿度)
					}
			
				break;
			
			
			case 0xA5: ///3：直接输出电压
				break;
			
			case 0x06: ///修改地址
				if(UART_RX_LPUART1.USART_RX_BUF[0] == Rs485Addr)	
				{
					///Rs485Addr	0x06	0x04	0x00	0x00	0x00	new	0x7b	0xa7
					
					///Rs485Addr = new
				}
			
				break;
			
			default:
				break;		
		}
	}
		
	memset(UART_RX_LPUART1.USART_RX_BUF, 0, UART_RX_LPUART1.USART_RX_Len);
	UART_RX_LPUART1.USART_RX_Len = 0;
	
}


/*
 *	CalcCRC16:	计算CRC16校验值
 *	data:		数据指针
 *	len:		数据长度
 *	返回值：	16位的CRC校验值
 */
uint16_t CalcCRC16(uint8_t *data, uint8_t len)
{
	uint16_t result = 0xffff;
	uint8_t i, j;

	for (i=0; i<len; i++)
	{
		result ^= data[i];
		for (j=0; j<8; j++)
		{
			if ( result&0x01 )
			{
					result >>= 1;
					result ^= 0xa001;
			}
			else
			{
					result >>= 1;
			}
		}
	}
	GET_CRC(&(data[len]), result);
	return result;
}
