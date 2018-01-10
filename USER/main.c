#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "dma.h"



//ALIENTEK Ì½Ë÷ÕßSTM32F407¿ª·¢°å ÊµÑé4
//´®¿ÚÍ¨ÐÅÊµÑé -¿âº¯Êý°æ±¾
//¼¼ÊõÖ§³Ö£ºwww.openedv.com
//ÌÔ±¦µêÆÌ£ºhttp://eboard.taobao.com
//¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾  
//×÷Õß£ºÕýµãÔ­×Ó @ALIENTEK
uint8_t motor_model[14]                =  {0};   /* Ö±ÐÐµç»ú²Ù×÷Ä£Ê½Ñ¡ÔñÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_outline[14]              =  {0};   /* Ö±ÐÐµç»úÔË¶¯ÂÖÀªÑ¡ÔñÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_disable[14]              =  {0};   /* Ö±ÐÐµç»ú¿ØÖÆ×Ö¹ØÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_enable[14]               =  {0};   /* Ö±ÐÐµç»ú¿ØÖÆ×Ö¿ªÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_velocity[14]             =  {0};   /* Ö±ÐÐµç»úÉèÖÃËÙ¶ÈÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_start[14]                =  {0};   /* Ö±ÐÐµç»ú¿ªÊ¼ÔË¶¯ÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_clearerror[14]           =  {0};   /* Ö±ÐÐµç»úÇå´íÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t motor_stop[14]                 =  {0};   /* Ö±ÐÐµç»úÍ£Ö¹ÔË¶¯ÃüÁî´®¿Ú·¢ËÍ»º´æÇø */
uint8_t recv[10] = {0};
uint8_t recv_node1[10]                 =  {0};   /* 1ºÅÖ±ÐÐµç»úÍ¨Ñ¶·µ»ØÊý¾Ý»º´æÇø */
uint8_t recv_node2[10]                 =  {0};   /* 2ºÅÖ±ÐÐµç»úÍ¨Ñ¶·µ»ØÊý¾Ý»º´æÇø */
uint8_t recv_node3[10]                 =  {0};   /* 3ºÅÖ±ÐÐµç»úÍ¨Ñ¶·µ»ØÊý¾Ý»º´æÇø */
uint8_t recv_node4[10]                 =  {0};   /* 4ºÅÖ±ÐÐµç»úÍ¨Ñ¶·µ»ØÊý¾Ý»º´æÇø */
u8 flag =0;
u8 fun_motor = 0;
uint16_t CalcFieldCRC(uint16_t* pDataArray, uint16_t numberOfWords)
{
	uint16_t shifter, c;
	uint16_t carry;
	uint16_t CRC_MAXON=0;
	//Calculate pDataArray Word by Word
	while(numberOfWords--)
	{
		shifter = 0x8000;
		c = *pDataArray++;
		do
		{
			carry = CRC_MAXON & 0x8000;
			CRC_MAXON <<= 1;
			if(c & shifter) 
				CRC_MAXON++;
			if(carry)
				CRC_MAXON ^= 0x1021;
			shifter >>=1;
		} 
		while(shifter);
	}
		return CRC_MAXON;
}

void StraightMotorInit(uint16_t node)
{
	uint16_t i = 0;
	uint16_t motor_datarray[7] = {0};
	
	uint8_t node_tmp = node>>8;
	node = node<<8;
	node += node_tmp;
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6040;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x0080;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_clearerror[0]  = motor_datarray[0]>>8;
	motor_clearerror[1]  = motor_datarray[0];
	motor_clearerror[2]  = motor_datarray[1];
	motor_clearerror[3]  = motor_datarray[1]>>8;
	motor_clearerror[4]  = motor_datarray[2];
	motor_clearerror[5]  = motor_datarray[2]>>8;
	motor_clearerror[6]  = motor_datarray[3];
	motor_clearerror[7]  = motor_datarray[3]>>8;
	motor_clearerror[8]  = motor_datarray[4];
	motor_clearerror[9]  = motor_datarray[4]>>8;
	motor_clearerror[10] = motor_datarray[5];
	motor_clearerror[11] = motor_datarray[5]>>8;
	motor_clearerror[12] = motor_datarray[6];
	motor_clearerror[13] = motor_datarray[6]>>8;
	
//	HAL_UART_Receive_IT(&huart1, recv, 10);

//	HAL_UART_Transmit_DMA(&huart1, motor_clearerror, 14);
//	HAL_Delay(10);
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä£
	for(i=0;i<14;i++)
	{
		USART_SendData(USART1, motor_clearerror[i]);
		delay_ms(1);
	}
	delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) 
				flag=1;
	else flag =101;
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6060;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x0003;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_model[0]  = motor_datarray[0]>>8;
	motor_model[1]  = motor_datarray[0];
	motor_model[2]  = motor_datarray[1];
	motor_model[3]  = motor_datarray[1]>>8;
	motor_model[4]  = motor_datarray[2];
	motor_model[5]  = motor_datarray[2]>>8;
	motor_model[6]  = motor_datarray[3];
	motor_model[7]  = motor_datarray[3]>>8;
	motor_model[8]  = motor_datarray[4];
	motor_model[9]  = motor_datarray[4]>>8;
	motor_model[10] = motor_datarray[5];
	motor_model[11] = motor_datarray[5]>>8;
	motor_model[12] = motor_datarray[6];
	motor_model[13] = motor_datarray[6]>>8;
	
//	HAL_UART_Receive_IT(&huart1, recv, 10);
//	HAL_UART_Transmit_DMA(&huart1, motor_model, 14);
//	HAL_Delay(10);	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä£
	for(i=0;i<14;i++)
	{	
		USART_SendData(USART1, motor_model[i]);
		delay_ms(1);
	}
	delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=2;
	else flag = 101;
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6086;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x0000;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_outline[0]  = motor_datarray[0]>>8;
	motor_outline[1]  = motor_datarray[0];
	motor_outline[2]  = motor_datarray[1];
	motor_outline[3]  = motor_datarray[1]>>8;
	motor_outline[4]  = motor_datarray[2];
	motor_outline[5]  = motor_datarray[2]>>8;
	motor_outline[6]  = motor_datarray[3];
	motor_outline[7]  = motor_datarray[3]>>8;
	motor_outline[8]  = motor_datarray[4];
	motor_outline[9]  = motor_datarray[4]>>8;
	motor_outline[10] = motor_datarray[5];
	motor_outline[11] = motor_datarray[5]>>8;
	motor_outline[12] = motor_datarray[6];
	motor_outline[13] = motor_datarray[6]>>8;
	
//	HAL_UART_Receive_IT(&huart1, recv, 10);
//	HAL_UART_Transmit_DMA(&huart1, motor_outline, 14);
//	HAL_Delay(10);	
		memset(recv,0,10);
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä£
	for(i=0;i<14;i++)
		{
			USART_SendData(USART1, motor_outline[i]);
			delay_ms(1);	
		}
		delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=3;
	else flag = 102;
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6040;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x0006;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_disable[0]  = motor_datarray[0]>>8;
	motor_disable[1]  = motor_datarray[0];
	motor_disable[2]  = motor_datarray[1];
	motor_disable[3]  = motor_datarray[1]>>8;
	motor_disable[4]  = motor_datarray[2];
	motor_disable[5]  = motor_datarray[2]>>8;
	motor_disable[6]  = motor_datarray[3];
	motor_disable[7]  = motor_datarray[3]>>8;
	motor_disable[8]  = motor_datarray[4];
	motor_disable[9]  = motor_datarray[4]>>8;
	motor_disable[10] = motor_datarray[5];
	motor_disable[11] = motor_datarray[5]>>8;
	motor_disable[12] = motor_datarray[6];
	motor_disable[13] = motor_datarray[6]>>8;
	
//	HAL_UART_Receive_IT(&huart1, recv, 10);
//	HAL_UART_Transmit_DMA(&huart1, motor_disable, 14);
//	HAL_Delay(10);
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä£
	for(i=0;i<14;i++)
	{
		USART_SendData(USART1, motor_disable[i]);
		delay_ms(1);
	}
	delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=4;
	else flag = 103;
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6040;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x000f;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_enable[0]  = motor_datarray[0]>>8;
	motor_enable[1]  = motor_datarray[0];
	motor_enable[2]  = motor_datarray[1];
	motor_enable[3]  = motor_datarray[1]>>8;
	motor_enable[4]  = motor_datarray[2];
	motor_enable[5]  = motor_datarray[2]>>8;
	motor_enable[6]  = motor_datarray[3];
	motor_enable[7]  = motor_datarray[3]>>8;
	motor_enable[8]  = motor_datarray[4];
	motor_enable[9]  = motor_datarray[4]>>8;
	motor_enable[10] = motor_datarray[5];
	motor_enable[11] = motor_datarray[5]>>8;
	motor_enable[12] = motor_datarray[6];
	motor_enable[13] = motor_datarray[6]>>8;
	
//	HAL_UART_Receive_IT(&huart1, recv, 10);
//	HAL_UART_Transmit_DMA(&huart1, motor_enable, 14);
//	HAL_Delay(10);
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä£
	for(i=0;i<14;i++)
	{
		USART_SendData(USART1, motor_enable[i]);
		delay_ms(1);
	}
	delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=4;
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
}
/*************************
Function:    StraightMotorSetSpeed
Description: ÉèÖÃÖ±ÐÐµç»ú×ªËÙ£¬²¢Ê¹ÆäÔË×ª
Input:       node-ËùÐèÒªÉèÖÃµÄµç»úµØÖ·
History:
*************************/
void StraightMotorSetSpeed(uint16_t node)
{
	uint16_t motor_datarray[7] = {0};
	uint32_t motor_node1_rev = 0x03e8;
	uint8_t i;
	uint8_t node_tmp = node>>8;
	node = node<<8;
	node += node_tmp;
	
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
//	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x60ff;
	motor_datarray[2] = node;
	if(fun_motor==4)
	{
		if(node == 0x0200 || node == 0x0400)
		{
			motor_datarray[3] = 0x0190;
			motor_datarray[4] = 0x0000;
		}
		else if(node == 0x0100 || node == 0x0300)
		{
			motor_datarray[3] = 0xfe6f;
			motor_datarray[4] = 0xffff;
		}
	}
	else if(fun_motor == 2)
	{
		 if(node == 0x0100 || node == 0x0300)
		{
			motor_datarray[3] = 0x0190;
			motor_datarray[4] = 0x0000;
		}
		else if(node == 0x0200 || node == 0x0400)
		{
			motor_datarray[3] = 0xfe6f;
			motor_datarray[4] = 0xffff;
		}
	}
	
	
//		if(node==0x0200||node==0x0400)
//	{
//		motor_node1_rev = 0xf830;
//	}
//	motor_datarray[3] = motor_node1_rev; 
//	motor_datarray[4] = (motor_node1_rev>>16);
//	motor_datarray[5] = 0x0000;
//	switch(node)
//	{
//		case(0x0100):motor_datarray[3] = motor_node1_rev; 
//								 motor_datarray[4] = (motor_node1_rev>>16);
//								 break;
//	
//		default:     break;
//	}
//	if(node==0x0200||node==0x0400)
//	{
////		motor_node1_rev = 0xe830;
//////		motor_datarray[3] = 0xe830;
//////		motor_datarray[4] = (0xe830>>16);
//////		motor_datarray[5] = 0xffff;
////		motor_datarray[3] = motor_node1_rev; 
////		motor_datarray[4] = (motor_node1_rev>>16);
//		motor_datarray[5] = 0xffff;
//	}
		
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_velocity[0]  = motor_datarray[0]>>8;
	motor_velocity[1]  = motor_datarray[0];
	motor_velocity[2]  = motor_datarray[1];
	motor_velocity[3]  = motor_datarray[1]>>8;
	motor_velocity[4]  = motor_datarray[2];
	motor_velocity[5]  = motor_datarray[2]>>8;
	motor_velocity[6]  = motor_datarray[3];
	motor_velocity[7]  = motor_datarray[3]>>8;
	motor_velocity[8]  = motor_datarray[4];
	motor_velocity[9]  = motor_datarray[4]>>8;
	motor_velocity[10] = motor_datarray[5];
	motor_velocity[11] = motor_datarray[5]>>8;
	motor_velocity[12] = motor_datarray[6];
	motor_velocity[13] = motor_datarray[6]>>8;
	
	//HAL_UART_Receive_IT(&huart1, recv, 10);
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä?
	for(i=0;i<14;i++)
	{
		USART_SendData(USART1, motor_velocity[i]);
		delay_ms(1);
	}
	delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=5;
	
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
//	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤

	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6040;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x000f;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_start[0]  = motor_datarray[0]>>8;
	motor_start[1]  = motor_datarray[0];
	motor_start[2]  = motor_datarray[1];
	motor_start[3]  = motor_datarray[1]>>8;
	motor_start[4]  = motor_datarray[2];
	motor_start[5]  = motor_datarray[2]>>8;
	motor_start[6]  = motor_datarray[3];
	motor_start[7]  = motor_datarray[3]>>8;
	motor_start[8]  = motor_datarray[4];
	motor_start[9]  = motor_datarray[4]>>8;
	motor_start[10] = motor_datarray[5];
	motor_start[11] = motor_datarray[5]>>8;
	motor_start[12] = motor_datarray[6];
	motor_start[13] = motor_datarray[6]>>8;
	
	
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä?
	for(i=0;i<14;i++)
		{
			USART_SendData(USART1, motor_start[i]);			
			delay_ms(1);
		}
		delay_ms(10);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//µÈ´ýDMA2_Steam7´«ÊäÍê³É
				{ 
					DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
					break; 
				}	
	}
	if(recv[0] == 0x4f && recv[1] == 0x4f &&
				 recv[2] == 0x00 && recv[3] ==0x01 &&
				 recv[4] == 0x00 && recv[5] ==0x00 &&
				 recv[6] == 0x00 && recv[7] ==0x00 &&
				 recv[8] == 0x51 && recv[9] ==0xaa) flag=6;
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÅœÔ®à š1Ö„DMA×¢Ì     
//	MYDMA_Enable(DMA2_Stream2,10);     //ßªÊ¼Ò»ÕŽDMAÔ«Ë¤

}
void StraightMotorStop(uint16_t node)
{
	uint16_t motor_datarray[7] = {0};
	
	uint8_t i;
	uint8_t node_tmp = node>>8;
	node = node<<8;
	node += node_tmp;
	
	motor_datarray[0] = 0x1103;
	motor_datarray[1] = 0x6040;
	motor_datarray[2] = node;
	motor_datarray[3] = 0x010f;
	motor_datarray[4] =	0x0000;
	motor_datarray[5] = 0x0000;
	motor_datarray[6] = 0x4f4f;
	motor_datarray[5] = CalcFieldCRC((uint16_t*)motor_datarray,0x06);
	
	motor_stop[0]  = motor_datarray[0]>>8;
	motor_stop[1]  = motor_datarray[0];
	motor_stop[2]  = motor_datarray[1];
	motor_stop[3]  = motor_datarray[1]>>8;
	motor_stop[4]  = motor_datarray[2];
	motor_stop[5]  = motor_datarray[2]>>8;
	motor_stop[6]  = motor_datarray[3];
	motor_stop[7]  = motor_datarray[3]>>8;
	motor_stop[8]  = motor_datarray[4];
	motor_stop[9]  = motor_datarray[4]>>8;
	motor_stop[10] = motor_datarray[5];
	motor_stop[11] = motor_datarray[5]>>8;
	motor_stop[12] = motor_datarray[6];
	motor_stop[13] = motor_datarray[6]>>8;
	
	memset(recv,0,10);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //Ê¹ÄÜ´®¿Ú1µÄDMA·¢ËÍ     
	MYDMA_Enable(DMA2_Stream2,10);     //¿ªÊ¼Ò»´ÎDMA´«Êä?
	for(i=0;i<14;i++)
		{
			USART_SendData(USART1, motor_stop[i]);			
			delay_ms(1);
		}
		delay_ms(10);
}
int main(void)
{ 
 
	u8 t;
	u8 i;
	u8 len=1;	
	u16 times=0; 
	char buf[]={0x11,0x13};	

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ÉèÖÃÏµÍ³ÖÐ¶ÏÓÅÏÈ¼¶·Ö×é2
	MYDMA_Config(DMA2_Stream2,DMA_Channel_4,(u16)&USART1->DR,(u16)recv,10);
	delay_init(168);		//ÑÓÊ±³õÊ¼»¯ 
	uart_init(115200);	//´®¿Ú³õÊ¼»¯²¨ÌØÂÊÎª115200
	LED_Init();		  		//³õÊ¼»¯ÓëLEDÁ¬½ÓµÄÓ²¼þ½Ó¿Ú  
	KEY_Init();
	StraightMotorInit(0x0001);
	StraightMotorInit(0x0002);
	StraightMotorInit(0x0003);
	StraightMotorInit(0x0004);
	while(1)
	{
			
	fun_motor = KEY_Scan(0);	
	if(fun_motor==WKUP_PRES)		
	{
		StraightMotorSetSpeed(0x0001);
		StraightMotorSetSpeed(0x0002);
		StraightMotorSetSpeed(0x0003);
		StraightMotorSetSpeed(0x0004);
	}
	if(fun_motor==KEY0_PRES)		
	{
		StraightMotorStop(0x01);
		StraightMotorStop(0x02);
		StraightMotorStop(0x03);
		StraightMotorStop(0x04);
	}
		if(fun_motor==KEY1_PRES)		
	{
		StraightMotorSetSpeed(0x0001);
		StraightMotorSetSpeed(0x0002);
		StraightMotorSetSpeed(0x0003);
		StraightMotorSetSpeed(0x0004);
	}
//				else 
//									printf("ÕýµãÔ­×Ó@ALIENTEK\r\n\r\n\r\n");
	
//		if(len ==0 )
//		{
//			StraightMotorInit(0x0100);
//			len = 1;
//		}
////		if(USART_RX_STA&0x8000)
////		{					   
////			len=USART_RX_STA&0x3fff;//µÃµ½´Ë´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È
////			if(USART_RX_BUF[0] == 0x4f && USART_RX_BUF[1] == 0x4f &&
////				 USART_RX_BUF[2] == 0x00 && USART_RX_BUF[3] ==0x01 &&
////				 USART_RX_BUF[4] == 0x00 && USART_RX_BUF[5] ==0x00 &&
////				 USART_RX_BUF[6] == 0x00 && USART_RX_BUF[7] ==0x00 &&
////				 USART_RX_BUF[8] == 0x51 && USART_RX_BUF[9] ==0xaa) len=2;
//			for(t=0;t<len;t++)
//			{
//				USART_RX_BUF[0]
//				USART_SendData(USART1, USART_RX_BUF[t]);         //Ïò´®¿Ú1·¢ËÍÊý¾Ý
//				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//µÈ´ý·¢ËÍ½áÊø
//			}
//			printf("\r\n\r\n");//²åÈë»»ÐÐ
//			USART_RX_STA=0;
//		}
//	else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\nALIENTEK Ì½Ë÷ÕßSTM32F407¿ª·¢°å ´®¿ÚÊµÑé\r\n");
//				printf("ÕýµãÔ­×Ó@ALIENTEK\r\n\r\n\r\n");
//			}
//			if(times%200==0)printf("ÇëÊäÈëÊý¾Ý,ÒÔ»Ø³µ¼ü½áÊø\r\n");  
//			if(times%30==0)LED0=!LED0;//ÉÁË¸LED,ÌáÊ¾ÏµÍ³ÕýÔÚÔËÐÐ.
//			delay_ms(10);   
//		}
	}
}

