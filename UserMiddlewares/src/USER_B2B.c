#include "USER_B2B.h"
#include "usart.h"
#include "cmsis_os.h"
#include "UserFreertos.h"
#include "Moto.h"
#include "judge.h"
#include "USER_Detcet.h"
#include "imu_temp_ctrl.h"
#include <string.h>
#include "PowerCtrl.h"


extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t rs485_isvalid = 0;

#define B2B_FRAME_LEN 64U


/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;
uint8_t FEEDBACK = 0;


uint32_t receive_times;

/* 需要用到的发送变量*/
uint8_t txbuffer[64] = {0};

void B2B_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void B2B_ParseUsart() // 先发低字节
{
	if (usart2RxBuf[0] == 0xAA && usart2RxBuf[B2B_FRAME_LEN - 1U] == 0xFE)
	{
		/*先解析接收内容*/
		for (uint8_t i = 0; i < 4; i++)
		{
				chassis.M3508[i].targetSpeed = (int16_t)usart2RxBuf[1 + i * 2] | (int16_t)usart2RxBuf[1 + i * 2 + 1] << 8;
		} //轮电机目标速度	1-8
		move.maxVx = (int16_t)((uint16_t)usart2RxBuf[9] |((uint16_t)usart2RxBuf[10] << 8)) / 1000.0f;
		move.maxVy = (int16_t)((uint16_t)usart2RxBuf[11] |((uint16_t)usart2RxBuf[12] << 8)) / 1000.0f;
		move.xSlope.value = (int16_t)((uint16_t)usart2RxBuf[13] |((uint16_t)usart2RxBuf[14] << 8)) / 1000.0f;
		move.ySlope.value = (int16_t)((uint16_t)usart2RxBuf[15] |((uint16_t)usart2RxBuf[16] << 8)) / 1000.0f;
		move.fastMode = usart2RxBuf[17]; //快速模式 0-普通模式 1-快速模式
		STOPFLAG = usart2RxBuf[62];


		
		/* 发送    */
		txbuffer[0] = 0xAB;

		for (uint8_t i = 0; i < 4; i++)
		{
				txbuffer[1 + i * 2] = chassis.M3508[i].speed;
				txbuffer[1 + i * 2 + 1] = chassis.M3508[i].speed >> 8;
		} // 1-8 轮电机当前速度
		txbuffer[9] = SET_WHEELSPEED_MAX;
		txbuffer[10] = SET_WHEELSPEED_MAX >> 8; //9-10 轮电机速度上限

		{
			float v = whell_power.InputPower;
			uint8_t *p = (uint8_t *)&v;
			txbuffer[11] = p[0];
			txbuffer[12] = p[1];
			txbuffer[13] = p[2];
			txbuffer[14] = p[3];
		}//11-14 轮电机功率上限
		memcpy(&txbuffer[15], &USER_JudgeData, sizeof(JudgeData_t)); //15-43 裁判系统数据
		
		txbuffer[62] = FEEDBACK;
		txbuffer[B2B_FRAME_LEN - 1U] = 0xFD;	
		rs485_isvalid = 1;
		HAL_UART_Transmit_DMA(&huart2, txbuffer, sizeof(txbuffer));
	}
}
	
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2)
	{
		if (Size == B2B_FRAME_LEN)
		{
			receive_times ++;
			B2B_ParseUsart();
			Detect_Update(DeviceID_B2B);
			detectList[DeviceID_B2B].isLost = 0;
		}
	}
	if(huart == &huart1)
	{
			Judge_Receive();
			Detect_Update(DeviceID_Judge);	
	}
}


void Task_B2B_Callback()
{
	/**********特殊情况处理*********************/
	if (STOPFLAG == 1)
	{
		FEEDBACK = 1;
		B2B_ParseUsart();
		osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务
	}
}
int times;
/************************freertos任务****************************/

void OS_Board2BoardCallback(void const *argument)
{
	B2B_Init();
	for (;;)
	{
		times ++;
		if(times >= 100)
		{
			times = 0;
			if(receive_times <= 5)
			{
				HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
				__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT); 
			}
			receive_times = 0;
		}
		Task_B2B_Callback();
		osDelay(1);
	}
}

