#include "usart.h"
#include "BSP_usart.h"
#include "DBUS.h"
#include "STMGood.h"
#include "ramp.h"
#include "GimbalControl.h"
#include "stdio.h"
#include "camera.h"
#include "Judge.h"
uint8_t uart5_buff[1]={0};
uint8_t Rxdata[1]={0};
uint8_t Usart3buff[100]={0};
uint8_t Usart6buff[100]={0};
uint8_t Usart2buff[100]={0};
uint8_t Usart1buff[100]={0};
uint8_t Mdata[8];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
/**
 * @brief Enable USART3
 * @param None
 * @return None
 * @attention None
 */
void USART3_Enable(void)
{
	HAL_UART_Receive_IT(&huart3,Usart3buff,1);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_ERR);	
}
/**
 * @brief Enable USART2
 * @param None
 * @return None
 * @attention None
 */
void USART2_Enable(void)
{
	HAL_UART_Receive_IT(&huart2,Usart2buff,1);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_ERR);	
}
/**
 * @brief callbackfunction  
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		Dealdata(Usart2buff[0]);
		__HAL_UART_CLEAR_PEFLAG(&huart2);
		HAL_UART_Receive_IT(&huart2,Usart2buff,1);	
	}
	if(huart->Instance == USART6)
	{
//		printf("uart3 work\r\n");
		JudgeData(Usart6buff[0]);
		__HAL_UART_CLEAR_PEFLAG(&huart6);
		HAL_UART_Receive_IT(&huart6,Usart6buff,1);	
	}

}
/**
 * @brief Error Callback function
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //清除错误标志位，清空SR、DR寄存器
	}
}


/**
 * @brief Redirect function for printf
 * @param None
 * @return None
 * @attention  The printf function could not be usedwithout this function
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0); 
	USART2->DR = (uint8_t) ch;      
	return ch;
}

/**
 * @brief Enable the Usart1
 * @param None
 * @return None
 * @attention  None
 */
void USART1_Enable(void)
{
	 HAL_DMA_Start(&hdma_usart1_rx, (uint32_t)huart1.Instance->DR, (uint32_t)Remotebuffer,18);
	 huart1.Instance->CR3 |= USART_CR3_DMAR;								/*!<DMA Enable Receiver         */
	 __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);								/*!<使能串口的中断为空闲中断    */
	 HAL_UART_Receive_DMA(&huart1,Remotebuffer,18);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(&huart1,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART1_IDLE_IRQ(void)
{
	
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);	//清除标志位和SR，DR寄存器
			HAL_UART_DMAStop(&huart1);
			HAL_UART_Receive_DMA(&huart1,Remotebuffer,18);//函数中包括重新配置DMA
		}
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART3_IDLE_IRQ(void)
{
	
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET)
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);	//清除标志位和SR，DR寄存器
		}
}

/**
 * @brief Enable the Usart6
 * @param None
 * @return None
 * @attention  None
 */
//void USART6_Enable(void)
//{
//	 HAL_DMA_Start(&hdma_usart6_rx, (uint32_t)huart6.Instance->DR, (uint32_t)uart6_buff,23);
//	 huart6.Instance->CR3 |= USART_CR3_DMAR;								/*!<DMA Enable Receiver         */
//	 __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);								/*!<使能串口的中断为空闲中断    */
//	 HAL_UART_Receive_DMA(&huart6,uart6_buff,23);								/*!<DMA Receive data            */
//	 __HAL_UART_ENABLE_IT(&huart6,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
//}

void USART6_Enable(void)
{
	HAL_UART_Receive_IT(&huart6,Usart6buff,1);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_ERR);	
}













