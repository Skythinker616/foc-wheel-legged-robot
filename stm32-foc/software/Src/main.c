/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "MagneticSensor.h" 
#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "FlashStorage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//用于蜂鸣的音符
enum
{
	//不响
	T_None=0,
	//低八度
  T_L1=3822,
  T_L2=3405,
  T_L3=3034,
  T_L4=2863,
  T_L5=2551,
  T_L6=2272,
  T_L7=2052,
	//中八度
  T_M1=1911,
  T_M2=1703,
  T_M3=1517,
  T_M4=1432,
  T_M5=1276,
  T_M6=1136,
  T_M7=1012,
	//高八度
  T_H1=956,
  T_H2=851,
  T_H3=758,
  T_H4=716,
  T_H5=638,
  T_H6=568,
  T_H7=506
};

//卡尔曼滤波结构体
typedef struct{
	float X_last;
	float X_mid;
	float X_now;
	float P_mid;
	float P_now;
	float P_last;
	float kg;
	float A;
	float Q;
	float R;
	float H;
} Kalman;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VOLT_SUPPLY 12 //供电母线电压
#define MAX_VOLT 6.9f  //限制供电电压(平均值)，最高为VOLT_SUPPLY/√3; 4010电机:6.9V, 2804电机:4V
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float targetVotage = 0;    //当前目标电压
uint8_t beepPlaying = 0;   //当前是否在蜂鸣状态
uint8_t motorID = 1;       //初始电机ID
uint8_t ledBlink = 1;      //led是否处于正常闪烁模式
float speed = 0;           //传感器作差数据所得转速
float filteredAngle = 0;   //经滤波得到的转子角度
uint32_t lastRecvTime = 0; //上次收到CAN数据帧的时间
Kalman angleFilter; //卡尔曼滤波结构体
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) { return ch; }

//单片机软件复位
void System_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

/*************卡尔曼滤波*************/

void Kalman_Init(Kalman* p,float T_Q,float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
	p->A = 1;
	p->H = 1;
	p->X_mid = p->X_last;
}

float Kalman_Filter(Kalman* p,float dat)
{
	if(!p) return 0;
	p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
	p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
	p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
	p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
	p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
	p->P_last = p->P_now;
	p->X_last = p->X_now;
	return p->X_now;
}

/*************CAN通信*************/

//CAN外设初始化
void CAN_Init()
{
	CAN_FilterTypeDef filter;
	filter.FilterActivation = ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan, &filter);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//发送一个反馈数据帧
void CAN_SendState(float angle, float speed)
{
	CAN_TxHeaderTypeDef header;
	header.StdId = motorID + 0x100;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 8;
	
	uint8_t data[8];
	memcpy(data,&(int32_t){angle*1000},4); //角度数据放在前四个字节
	memcpy(&data[4],&(int16_t){speed*10},2); //转速数据放在第5-6字节
	
	uint32_t mailbox;
	HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

//CAN收到数据的中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t rxData[8];
	
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxData) != HAL_OK)
		return;
	
	if(header.StdId == 0x100 && motorID <= 4) //ID=1~4接收0x100数据帧
	{
		targetVotage = *(int16_t*)&rxData[(motorID-1)*2] / 1000.0f;
		lastRecvTime = HAL_GetTick();
	}
	else if(header.StdId == 0x200 && motorID > 4) //ID=5~8接收0x200数据帧
	{
		targetVotage = *(int16_t*)&rxData[(motorID-5)*2] / 1000.0f;
		lastRecvTime = HAL_GetTick();
	}
}

/*************FOC*************/

void SimpleFOC_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //开启三个PWM通道输出
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	MagneticSensor_Init(); //初始化磁传感器
	
	voltage_power_supply=VOLT_SUPPLY; //设定FOC所需参数
	voltage_limit=MAX_VOLT;
	voltage_sensor_align=voltage_limit;
	targetVotage=0;
	
	Motor_init(); //初始化电机信息
	Motor_initFOC(); //初始化FOC参数
	
	motorID = Flash_ReadMotorID(); //从Flash读取电机ID
	motorID = motorID ? motorID : 1;
}

/*************蜂鸣*************/

//根据蜂鸣周期配置并触发定时器
void Beep_Play(uint16_t period)
{
	if(period!=0)
	{
		__HAL_TIM_SetAutoreload(&htim1,period/2);
		HAL_TIM_Base_Start_IT(&htim1);
		beepPlaying = 1;
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		beepPlaying = 0;
	}
}

//播放一串音符
void Beep_PlayNotes(uint8_t num, uint16_t notes[][2])
{
	for(uint8_t i=0; i<num; i++)
	{
		Beep_Play(notes[i][0]);
		HAL_Delay(notes[i][1]);
	}
	Beep_Play(0);
}

//蜂鸣中断处理，在定时器中断回调中调用
void Beep_IRQHandler()
{
	static uint8_t flipFlag = 0;
	if(targetVotage == 0)
	{
		setPhaseVoltage(voltage_limit/2, 0, _PI/3 * flipFlag); //使磁场方向在0-PI/3间震荡
		flipFlag = !flipFlag;
	}
}

/*************各个定时任务*************/

//非阻塞按键处理
void Key_Process()
{
	static uint32_t downTime = 0;
	static uint8_t lastKeyState = 0;
	uint8_t keyState = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1 : 0;
	if(keyState && !lastKeyState) //按下
	{
		downTime = HAL_GetTick();
		ledBlink = 0;
	}
	else if(keyState && lastKeyState) //按住
	{
		uint32_t pressTime = HAL_GetTick() - downTime;
		if(pressTime < 500*8) //闪8下，每次500ms
		{
			if(pressTime%500 < 100)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else if(pressTime < 500*12)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //闪8下之后常亮
	}
	else if(!keyState && lastKeyState) //松开
	{
		uint32_t pressTime = HAL_GetTick() - downTime;
		if(pressTime > 50 && pressTime <500*8) //闪8下以内松开，设置ID
		{
			motorID = pressTime/500 + 1;
			Flash_SaveMotorID(motorID);
		}
		else if(pressTime >= 500*8 && pressTime < 500*12) //闪8下后松开，清空Flash后复位重新校准
		{
			Flash_EraseMotorParam();
			System_Reset();
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		ledBlink = 1;
	}
	lastKeyState = keyState;
}

//非阻塞LED定时任务，闪烁次数表示电机ID
void Led_Process()
{
	if(!ledBlink) return;
	uint32_t period = 1000 + (100+200)*motorID;
	uint32_t mod = HAL_GetTick() % period;
	if(mod < (100+200)*motorID && mod%(100+200) < 100)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

//电机转速计算
void Motor_SpeedCalcProcess()
{
	const uint8_t speedLpfLen = 5;
	static float speedLpfBuf[speedLpfLen] = {0}; //存放5个filteredAngle
	
	float angle = filteredAngle;
	float curSpeed = (angle-speedLpfBuf[0])*1000/speedLpfLen/_2PI*60;
	speed = curSpeed;
	
	for(uint8_t i=0; i<speedLpfLen-1; i++)
		speedLpfBuf[i] = speedLpfBuf[i+1];
	speedLpfBuf[speedLpfLen-1] = angle;
}

//CAN离线检测，500ms没收到CAN信号则停机
void Motor_OfflineCheckProcess()
{
	static uint8_t isOffline = 0;
	if(HAL_GetTick() - lastRecvTime > 500)
	{
		if(!isOffline)
		{
			targetVotage = 0;
			isOffline = 1;
		}
	}
	else
		isOffline = 0;
}

//在滴答定时器中调用，1ms周期，处理各定时任务
void SysTick_UserExec()
{
	Key_Process();
	Led_Process();
	Motor_SpeedCalcProcess();
	Motor_OfflineCheckProcess();
	if(HAL_GetTick()%2)
		CAN_SendState(shaft_angle, speed);
	filteredAngle = Kalman_Filter(&angleFilter, shaft_angle);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	CAN_Init(); //CAN外设初始化
	SimpleFOC_Init(); //FOC初始化
	Beep_PlayNotes(3,(uint16_t[][2]){{T_H1,200},{T_H3,200},{T_H5,500}}); //播放开机音效
	
	Kalman_Init(&angleFilter, 0.0005f, 0.1f); //设定卡尔曼滤波系数
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		setTargetVotage(targetVotage); //设定FOC电压
		loopFOC(); //运行FOC算法
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
