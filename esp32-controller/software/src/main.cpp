/* main.c
 * 主代码文件，包含所有主要代码逻辑
 * by L.B.W 2023
 * */

#include <Arduino.h>
#include <PID.h>
#include "driver/twai.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "leg_pos.h"
#include "lqr_k.h"
#include "leg_conv.h"
#include "leg_spd.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <debug.h>

/*******************宏定义*******************/

//蓝牙各UUID及设备名称
#define UUID_SERVICE "4c9a0001-fb2e-432e-92ec-c6f5316b4689" //服务UUID
#define UUID_RX_CHARA "4c9a0002-fb2e-432e-92ec-c6f5316b4689" //接收特征UUID
#define UUID_TX_CHARA "4c9a0003-fb2e-432e-92ec-c6f5316b4689" //发送特征UUID
#define BLE_NAME "BalanceBot"

//计时工具，计算运行周期或经过时间
uint32_t timeCnt[6] = {0}, lastTimeCnt[6] = {0}, passTimeCnt[6] = {0};
#define TIME_COUNTER(num) do{timeCnt[num] = micros()-lastTimeCnt[num]; lastTimeCnt[num] = micros();}while(0)
#define PASS_TIME_START(num) uint32_t passTimeStart##num = micros()
#define PASS_TIME_STOP(num) passTimeCnt[num] = micros() - passTimeStart##num

/*******************结构体类型和全局变量定义*******************/

//IMU对象及数据
MPU6050 mpu;
struct IMUData
{
	float yaw, pitch, roll;			 // rad
	float yawSpd, pitchSpd, rollSpd; // rad/s
	float zAccel; // m/s^2
} imuData;

//电机结构体
struct Motor
{
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float voltage, maxVoltage; // V
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	float dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // function to calculate reverse voltage
} leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象

//腿部姿态结构体
struct LegPos
{
	float angle, length;   // rad, m
	float dAngle, dLength; // rad/s, m/s
	float ddLength;		   // m/s^2
} leftLegPos, rightLegPos; //左右腿部姿态

//状态变量结构体
struct StateVar
{
	float theta, dTheta;
	float x, dx;
	float phi, dPhi;
} stateVar;

//目标量结构体
struct Target
{
	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
} target = {0, 0, 0, 0, 0, 0, 0.07f};

//触地检测数据结构体
struct GroundDetector
{
	float leftSupportForce, rightSupportForce;
	bool isTouchingGround, isCuchioning;
} groundDetector = {10, 10, true, false};

//站立过程状态枚举量
enum StandupState {
	StandupState_None,
	StandupState_Prepare,
	StandupState_Standup,
} standupState = StandupState_None;

CascadePID legAnglePID, legLengthPID; //腿部角度和长度控制PID
CascadePID yawPID, rollPID; //机身yaw和roll控制PID

//蓝牙相关全局对象指针
BLECharacteristic *chara_tx, *chara_rx;
BLEServer *server;
BLEService *service;

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
float sourceVoltage = 12; //当前电源电压

/*******************函数声明*******************/

void setup();
void loop();

void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir);
float Motor_CalcRevVolt4010(float speed);
float Motor_CalcRevVolt2804(float speed);
void Motor_InitAll();
void Motor_Update(Motor *motor, uint8_t *data);
void Motor_SetTorque(Motor *motor, float torque);
void Motor_UpdateVoltage(Motor *motor);
void Motor_SendTask(void *arg);

void CAN_RecvCallback(uint32_t id, uint8_t *data);
void CAN_RecvTask(void *arg);
void CAN_Init();
void CAN_SendFrame(uint32_t id, uint8_t *data);

void IMU_Task(void *arg);
void IMU_Init();

void Ctrl_TargetUpdateTask(void *arg);
void LegPos_UpdateTask(void *arg);
void Ctrl_StandupPrepareTask(void *arg);
void Ctrl_Task(void *arg);
void Ctrl_Init();

void BT_SendSampleTask(void *arg);
void BT_Init();

void Serial_Task(void *pvParameters);
void Serial_Init();

void ADC_Task(void *pvParameters);
void ADC_Init();

/*******************各模块函数定义*******************/

/******* 电机模块 *******/

//初始化一个电机对象
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed))
{
	motor->speed = motor->angle = motor->voltage = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->calcRevVolt = calcRevVolt;
}

//4010电机反电动势计算函数(输入速度，输出反电动势)
float Motor_CalcRevVolt4010(float speed)
{
	return 0.00008f * speed * speed * speed - 0.0035f * speed * speed + 0.2322f * speed;
}

//2804电机反电动势计算函数(输入速度，输出反电动势)
float Motor_CalcRevVolt2804(float speed)
{
	return 0.000004f * speed * speed * speed - 0.0003f * speed * speed + 0.0266f * speed;
}

//初始化所有电机对象
void Motor_InitAll()
{
	Motor_Init(&leftJoint[0], 1.431, 7, 0.0316f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&leftJoint[1], -7.76, 7, 0.0317f, 1, Motor_CalcRevVolt4010);
	Motor_Init(&leftWheel, 0, 4.0f, 0.0096f, 1, Motor_CalcRevVolt2804);
	Motor_Init(&rightJoint[0], 0.343, 7, 0.0299f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&rightJoint[1], -2.446, 7, 0.0321f, -1, Motor_CalcRevVolt4010);
	Motor_Init(&rightWheel, 0, 4.0f, 0.0101f, 1, Motor_CalcRevVolt2804);
	xTaskCreate(Motor_SendTask, "Motor_SendTask", 2048, NULL, 5, NULL);
}

//从CAN总线接收到的数据中解析出电机角度和速度
void Motor_Update(Motor *motor, uint8_t *data)
{
	motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
	motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, float torque)
{
	motor->torque = torque;
}

//由设置的目标扭矩和当前转速计算补偿后的电机电压并进行限幅
void Motor_UpdateVoltage(Motor *motor)
{
	float voltage = motor->torque / motor->torqueRatio * motorOutRatio;
	if (motor->speed >= 0)
		voltage += motor->calcRevVolt(motor->speed);
	else if (motor->speed < 0)
		voltage -= motor->calcRevVolt(-motor->speed);
	if (voltage > motor->maxVoltage)
		voltage = motor->maxVoltage;
	else if (voltage < -motor->maxVoltage)
		voltage = -motor->maxVoltage;
	motor->voltage = voltage * motor->dir;
}

//电机指令发送任务
void Motor_SendTask(void *arg)
{
	uint8_t data[8] = {0};
	Motor* motorList[] = {&leftJoint[0], &leftJoint[1], &leftWheel, &rightJoint[0], &rightJoint[1], &rightWheel};
	while (1)
	{
		for (int i = 0; i < 6; i++)
			Motor_UpdateVoltage(motorList[i]); //计算补偿后的电机电压
		
		*(int16_t *)&data[0] = ((int16_t)(leftJoint[0].voltage * 1000));
		*(int16_t *)&data[2] = ((int16_t)(leftJoint[1].voltage * 1000));
		*(int16_t *)&data[4] = ((int16_t)(leftWheel.voltage * 1000));
		CAN_SendFrame(0x100, data);
		*(int16_t *)&data[0] = ((int16_t)(rightJoint[0].voltage * 1000));
		*(int16_t *)&data[2] = ((int16_t)(rightJoint[1].voltage * 1000));
		*(int16_t *)&data[4] = ((int16_t)(rightWheel.voltage * 1000));
		CAN_SendFrame(0x200, data);
		vTaskDelay(2);
	}
}

/******* CAN通信模块 *******/

//CAN收到数据后进入的回调函数
void CAN_RecvCallback(uint32_t id, uint8_t *data)
{
	switch (id) //根据CAN ID更新各电机数据
	{
	case 0x101:
		Motor_Update(&leftJoint[0], data);
		break;
	case 0x102:
		Motor_Update(&leftJoint[1], data);
		break;
	case 0x103:
		Motor_Update(&leftWheel, data);
		break;
	case 0x105:
		Motor_Update(&rightJoint[0], data);
		break;
	case 0x106:
		Motor_Update(&rightJoint[1], data);
		break;
	case 0x107:
		Motor_Update(&rightWheel, data);
		break;
	}
}

//CAN数据帧轮询接收任务
void CAN_RecvTask(void *arg)
{
	twai_message_t msg;
	twai_status_info_t status;
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		twai_get_status_info(&status);
		for(uint8_t i = 0; i < status.msgs_to_rx; i++)
		{
			if(twai_receive(&msg, 0) == ESP_OK)
				CAN_RecvCallback(msg.identifier, msg.data);
		}
		vTaskDelayUntil(&xLastWakeTime, 2); //2ms轮询一次
	}
}

//CAN通信外设初始化
void CAN_Init()
{
	twai_general_config_t twai_conf = {
		.mode = TWAI_MODE_NORMAL,
		.tx_io = GPIO_NUM_6,
		.rx_io = GPIO_NUM_7,
		.clkout_io = TWAI_IO_UNUSED,
		.bus_off_io = TWAI_IO_UNUSED,
		.tx_queue_len = 5,
		.rx_queue_len = 10,
		.alerts_enabled = TWAI_ALERT_NONE,
		.clkout_divider = 0,
		.intr_flags = ESP_INTR_FLAG_LEVEL1};

	twai_timing_config_t twai_timing = TWAI_TIMING_CONFIG_1MBITS();

	twai_filter_config_t twai_filter = {
		.acceptance_code = 0x00000000,
		.acceptance_mask = 0xFFFFFFFF,
		.single_filter = true};

	twai_driver_install(&twai_conf, &twai_timing, &twai_filter);
	twai_start();
	xTaskCreate(CAN_RecvTask, "CAN_RecvTask", 2048, NULL, 5, NULL);
}

//发送一帧CAN数据(data为8字节数据)
void CAN_SendFrame(uint32_t id, uint8_t *data)
{
	twai_message_t msg;
	msg.flags = 0;
	msg.identifier = id;
	msg.data_length_code = 8;
	memcpy(msg.data, data, 8);
	twai_transmit(&msg, 100);
}

/******* 陀螺仪模块 *******/

//陀螺仪数据获取任务
void IMU_Task(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint8_t fifoBuffer[64]; //dmp数据接收区
	int16_t yawRound = 0; //统计yaw转过的整圈数
	float lastYaw = 0;
	
	while (1)
	{
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
		{
			//获取陀螺仪角速度
			int16_t gyroData[3];
			mpu.getRotation(&gyroData[0], &gyroData[1], &gyroData[2]);
			imuData.rollSpd = gyroData[1] / 16.4f * M_PI / 180.0f;
			imuData.pitchSpd = -gyroData[0] / 16.4f * M_PI / 180.0f;
			imuData.yawSpd = gyroData[2] / 16.4f * M_PI / 180.0f;
			
			//获取陀螺仪欧拉角
			float ypr[3];
			Quaternion q;
			VectorFloat gravity;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			float yaw = -ypr[0];
			imuData.pitch = -ypr[2];
			imuData.roll = -ypr[1];

			if (yaw - lastYaw > M_PI)
				yawRound--;
			else if (yaw - lastYaw < -M_PI)
				yawRound++;
			lastYaw = yaw;
			imuData.yaw = yaw + yawRound * 2 * M_PI; //imuData.yaw为累计转角

			//获取陀螺仪Z轴加速度
			VectorInt16 rawAccel;
			mpu.dmpGetAccel(&rawAccel, fifoBuffer);
			VectorInt16 accel;
			mpu.dmpGetLinearAccel(&accel, &rawAccel, &gravity);
			imuData.zAccel = accel.z / 8192.0f * 9.8f;
		}
		vTaskDelayUntil(&xLastWakeTime, 5); //5ms轮询一次
	}
}

//陀螺仪模块初始化
void IMU_Init()
{
	//初始化IIC
	Wire.begin(5, 4);
	Wire.setClock(400000);

	//初始化陀螺仪
	mpu.initialize();
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	Serial.println("Address: 0x" + String(mpu.getDeviceID(), HEX));
	mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);
	while(mpu.dmpInitialize() != 0);
	mpu.setXAccelOffset(-668);
	mpu.setYAccelOffset(-1632);
	mpu.setZAccelOffset(998);
	mpu.setXGyroOffset(283);
	mpu.setYGyroOffset(80);
	mpu.setZGyroOffset(42);
	// mpu.CalibrateAccel(6); //测量偏移数据
	// mpu.CalibrateGyro(6);
	// mpu.PrintActiveOffsets();
	mpu.setDMPEnabled(true);

	//开启陀螺仪数据获取任务
	xTaskCreate(IMU_Task, "IMU_Task", 2048, NULL, 5, NULL);
}

/******* 运动控制模块 *******/

//目标量更新任务(根据蓝牙收到的目标量计算实际控制算法的给定量)
void Ctrl_TargetUpdateTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.003f;
	while (1)
	{
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		speedSlopeStep = -(legLength - 0.07f) * 0.05f + 0.003f;

		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}

		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;

		//限制速度目标在当前速度的±0.3m/s内
		if(target.speed - stateVar.dx > 0.3f)
			target.speed = stateVar.dx + 0.3f;
		else if(target.speed - stateVar.dx < -0.3f)
			target.speed = stateVar.dx - 0.3f;

		//计算yaw方位角目标
		target.yawAngle += target.yawSpeedCmd * 0.004f;
		
		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

//腿部姿态更新任务(根据关节电机数据计算腿部姿态)
void LegPos_UpdateTask(void *arg)
{
	const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		float legPos[2], legSpd[2];

		//计算左腿位置
		leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
		leftLegPos.length = legPos[0];
		leftLegPos.angle = legPos[1];

		//计算左腿速度
		leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
		leftLegPos.dLength = legSpd[0];
		leftLegPos.dAngle = legSpd[1];

		//计算左腿腿长加速度
		leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
		lastLeftDLength = leftLegPos.dLength;

		//计算右腿位置
		leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
		rightLegPos.length = legPos[0];
		rightLegPos.angle = legPos[1];

		//计算右腿速度
		leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
		rightLegPos.dLength = legSpd[0];
		rightLegPos.dAngle = legSpd[1];

		//计算右腿腿长加速度
		rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
		lastRightDLength = rightLegPos.dLength;

		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

//站立准备任务(将机器人从任意姿态调整到准备站立前的劈叉状态)
void Ctrl_StandupPrepareTask(void *arg)
{
	standupState = StandupState_Prepare;

	//将左腿向后摆
	Motor_SetTorque(&leftJoint[0], 0.2f);
	Motor_SetTorque(&leftJoint[1], 0.2f);
	while(leftLegPos.angle < M_3PI_4)
		vTaskDelay(5);
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	vTaskDelay(1000);

	//将右腿向前摆
	Motor_SetTorque(&rightJoint[0], -0.2f);
	Motor_SetTorque(&rightJoint[1], -0.2f);
	while(rightLegPos.angle > M_PI_4)
		vTaskDelay(5);
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	vTaskDelay(1000);

	//完成准备动作，关闭电机结束任务
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	Motor_SetTorque(&leftWheel, 0);
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	Motor_SetTorque(&rightWheel, 0);
	
	standupState = StandupState_Standup;

	vTaskDelete(NULL);
}

//主控制任务
void Ctrl_Task(void *arg)
{
	const float wheelRadius = 0.026f; //m，车轮半径
	const float legMass = 0.05f; //kg，腿部质量

	TickType_t xLastWakeTime = xTaskGetTickCount();

	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
						{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.07f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;

	while (1)
	{
		//计算状态变量
		stateVar.phi = imuData.pitch;
		stateVar.dPhi = imuData.pitchSpd;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imuData.pitch;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

		//如果正在站立准备状态，则不进行后续控制
		if(standupState == StandupState_Prepare)
		{
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}

		//计算LQR反馈矩阵
		float kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
		if(groundDetector.isTouchingGround) //正常触地状态
		{
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 2; j++)
					k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
			}
		}
		else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
		{
			memset(k, 0, sizeof(k));
			k[1][0] = kRes[1] * -2;
			k[1][1] = kRes[3] * -10;
		}

		//准备状态变量
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//与给定量作差
		x[2] -= target.position;
		x[3] -= target.speed;

		//矩阵相乘，计算LQR输出
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

		//计算yaw轴PID输出
		PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);
		
		//设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
		if(groundDetector.isTouchingGround) //正常接地状态
		{
			Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
			Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
		}
		else //腿部离地状态，关闭车轮电机
		{
			Motor_SetTorque(&leftWheel, 0);
			Motor_SetTorque(&rightWheel, 0);
		}

		//根据离地状态修改目标腿长，并计算腿长PID输出
		PID_CascadeCalc(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.12f, legLength, dLegLength);
		//计算roll轴PID输出
		PID_CascadeCalc(&rollPID, target.rollAngle, imuData.roll, imuData.rollSpd);
		//根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
		float leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6-rollPID.output : 0);
		float rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6+rollPID.output : 0);
		if(leftLegPos.length > 0.12f) //保护腿部不能伸太长
			leftForce -= (leftLegPos.length - 0.12f) * 100;
		if(rightLegPos.length > 0.12f)
			rightForce -= (rightLegPos.length - 0.12f) * 100;
		
		//计算左右腿的地面支持力
		groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imuData.zAccel);
		groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imuData.zAccel);
		//更新离地检测器数据
		static uint32_t lastTouchTime = 0;
		bool isTouchingGround = groundDetector.leftSupportForce > 3 && groundDetector.rightSupportForce > 3; //判断当前瞬间是否接地
		if(!isTouchingGround && millis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
			isTouchingGround = true;
		if(!groundDetector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
		{
			target.position = stateVar.x;
			groundDetector.isCuchioning = true;
			lastTouchTime = millis();
		}
		if(groundDetector.isCuchioning && legLength < target.legLength) //缓冲状态直到腿长压缩到目标腿长结束
			groundDetector.isCuchioning = false;
		groundDetector.isTouchingGround = isTouchingGround;

		//计算左右腿角度差PID输出
		PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
		
		//计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
		float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
		float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
		
		//使用VMC计算各关节电机输出扭矩
		float leftJointTorque[2]={0};
		leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		float rightJointTorque[2]={0};
		leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
		
		//保护腿部角度不超限
		float leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
		float rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
		#define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
								   rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
								   imuData.pitch > M_PI_4 || imuData.pitch < -M_PI_4) //腿部角度超限保护条件
		if(PROTECT_CONDITION) //当前达到保护条件
		{
			if(standupState == StandupState_None) //未处于起立过程中
			{
				//关闭所有电机
				Motor_SetTorque(&leftWheel, 0);
				Motor_SetTorque(&rightWheel, 0);
				Motor_SetTorque(&leftJoint[0], 0);
				Motor_SetTorque(&leftJoint[1], 0);
				Motor_SetTorque(&rightJoint[0], 0);
				Motor_SetTorque(&rightJoint[1], 0);
				//阻塞等待腿部角度回到安全范围，再等待4s后恢复控制(若中途触发了起立则在起立准备完成后直接跳出)
				while(PROTECT_CONDITION && standupState == StandupState_None)
				{
					leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
					rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
					vTaskDelay(100);
				}
				if(standupState == StandupState_None)
					vTaskDelay(4000);
				//退出保护后设定目标位置和yaw角度为当前值
				target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
				target.yawAngle = imuData.yaw;
				continue;
			}
			if(standupState == StandupState_Standup && (leftTheta < -M_PI_4 || rightTheta > M_PI_4))
				standupState = StandupState_None;
		}
		else
		{
			if(standupState == StandupState_Standup) //未达到保护条件且处于起立过程中，说明起立完成，退出起立过程
				standupState = StandupState_None;
		}

		//设定关节电机输出扭矩
		Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
		Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
		Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
		Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}

//控制模块初始化
void Ctrl_Init()
{
	//初始化各个PID参数
	PID_Init(&yawPID.inner, 0.01, 0, 0, 0, 0.1);
	PID_Init(&yawPID.outer, 10, 0, 0, 0, 2);
	PID_Init(&rollPID.inner, 1, 0, 5, 0, 5);
	PID_Init(&rollPID.outer, 20, 0, 0, 0, 3);
	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
	PID_Init(&legLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
	PID_Init(&legLengthPID.outer, 5.0f, 0, 0.0f, 0.0f, 0.5f);
	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
	PID_Init(&legAnglePID.inner, 0.04, 0, 0, 0, 1);
	PID_Init(&legAnglePID.outer, 12, 0, 0, 0, 20);
	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);

	//触发各个控制任务
	xTaskCreate(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL);
	xTaskCreate(LegPos_UpdateTask, "LegPos_UpdateTask", 4096, NULL, 2, NULL);
	vTaskDelay(2);
	xTaskCreate(Ctrl_Task, "Ctrl_Task", 4096, NULL, 1, NULL);
}

/******* 蓝牙模块 *******/

//通过蓝牙notify发送数据
extern "C" void BT_Send(uint8_t *data, uint32_t len)
{
	chara_tx->setValue(data, len);
	chara_tx->notify();
}

//通过蓝牙发送采样数据的任务
void BT_SendSampleTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t runDuration = (*(uint8_t *)arg) * 1000;
	uint32_t startTime = millis();
	while (1)
	{
		String str = "";
		str += String(target.speedCmd, 3) + "," + String(target.speed, 3) + "," + String(stateVar.dx, 3)  + "," + String(stateVar.phi) + "," + String(stateVar.theta) + "\r\n";
		// str += String(target.yawAngle, 3) + "," + String(imuData.yaw, 3) + "," + String(imuData.yawSpd, 3) + "\r\n";
		BT_Send((uint8_t *)str.c_str(), str.length());

		if (runDuration > 0 && millis() - startTime > runDuration)
			vTaskDelete(NULL);

		vTaskDelayUntil(&xLastWakeTime, 100);
	}
}

//重写蓝牙服务回调类，处理连接和断连事件
class MyBleServerCallbacks : public BLEServerCallbacks
{
public:
	void onConnect(BLEServer *server)
	{
		Serial.println("onConnect");
		server->getAdvertising()->stop();
	}
	void onDisconnect(BLEServer *server)
	{
		Serial.println("onDisconnect");
		server->getAdvertising()->start();
	}
};

//重写蓝牙特征回调类，处理特征读写事件
class MyBleCharaCallbacks : public BLECharacteristicCallbacks
{
public:
	void onWrite(BLECharacteristic *chara) //特征写入事件，收到蓝牙数据
	{
		uint32_t len = chara->getLength();
		uint8_t *data = chara->getData();

		// Serial.print("onWrite: " + String(len) + " bytes: "); //测试蓝牙收发
		// for (int i = 0; i < len; i++)
		// 	Serial.print(String(data[i], HEX) + " ");
		// Serial.println();
		// BT_Send(data, len);

		if(data[0] == 0xAA && len == 4) //0xAA开头，表示目标值控制指令
		{
			target.speedCmd = (data[2] / 100.0f - 1) * 1.0f;
			target.yawSpeedCmd = -(data[1] / 100.0f - 1) * 1.5f;
			target.legLength = (data[3] / 200.0f) * 0.02f + 0.07f;
		}
		else if(data[0] == 0xAB) //0xAB开头，表示触发起立过程
		{
			xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
		}
		else if(data[0] == 0xAC && len == 2) //0xAC开头，表示触发一次数据采样
		{
			xTaskCreate(BT_SendSampleTask, "BT_SendSampleTask", 4096, &data[1], 1, NULL);
		}
		else if(data[0] == 0xDB) //0xDB开头，表示是调试指令
		{
			Debug_SerialRecv(data, len);
		}
	}
};

//蓝牙模块初始化，创建设备、服务、特征并启动广播
void BT_Init()
{
	BLEDevice::init(BLE_NAME);
	server = BLEDevice::createServer();
	server->setCallbacks(new MyBleServerCallbacks());
	service = server->createService(UUID_SERVICE);
	chara_tx = service->createCharacteristic(UUID_TX_CHARA, BLECharacteristic::PROPERTY_NOTIFY);
	chara_tx->addDescriptor(new BLE2902());
	chara_rx = service->createCharacteristic(UUID_RX_CHARA, BLECharacteristic::PROPERTY_WRITE_NR);
	chara_rx->setCallbacks(new MyBleCharaCallbacks());
	service->start();
	server->getAdvertising()->start();
}

/******* 串口模块 *******/

//串口定时发送任务(调试用)
void Serial_Task(void *pvParameters)
{
	Serial.begin(115200);
	Serial.setTimeout(10);
	while (1)
	{
		// Serial.printf("%f,%f",leftJoint[0].angle, leftJoint[1].angle);
		// Serial.printf("%f,%f,%f,%f\r\n",leftLegPos.length, leftLegPos.angle, rightLegPos.length, rightLegPos.angle);
		// Serial.printf("%f,%f,%f\n",imuData.yaw, imuData.pitch, imuData.roll);
		// Serial.printf("%f,%f,%f,%f,%f,%f\r\n",leftJoint[0].angle, leftJoint[1].angle, leftWheel.angle, rightJoint[0].angle, rightJoint[1].angle, rightWheel.angle);
		// Serial.printf("%f,%f,%f,%f,%f,%f\r\n",stateVar.theta,stateVar.dTheta,stateVar.x,stateVar.dx,stateVar.phi,stateVar.dPhi);
		// Serial.printf("%f,%f\r\n", stateVar.theta, stateVar.dTheta);
		// Serial.printf("%f,%f\r\n",imuData.pitch,imuData.pitchSpd);
		// Serial.printf("%p\r\n", &speedPID.kp);
		// Serial.printf("source voltage: %f\r\n", analogRead(0) / 4095.0f * 3.3f * 11 * 12 / 13.5f);
		vTaskDelay(50);
	}
}

//串口模块初始化
void Serial_Init()
{
	xTaskCreate(Serial_Task, "Serial_Task", 4096, NULL, 1, NULL);
}

/******* ADC模块 *******/

//ADC定时采样任务，采样电池电压并计算电机输出比例
void ADC_Task(void *pvParameters)
{
	while (1)
	{
		sourceVoltage = analogRead(0) / 4095.0f * 3.3f * 11 * 12 / 13.5f;
		if(sourceVoltage < 8)
			sourceVoltage = 12;
		motorOutRatio = (12 - sourceVoltage) / 10.0f + 0.7f;
		vTaskDelay(100);
	}
}

//ADC模块初始化
void ADC_Init()
{
	xTaskCreate(ADC_Task, "ADC_Task", 4096, NULL, 5, NULL);
}

/*******************主函数*******************/

void setup()
{
	pinMode(9, INPUT_PULLUP); //按钮引脚
	pinMode(10, OUTPUT); //LED引脚

	//上电后等待5s
	digitalWrite(10, HIGH);
	vTaskDelay(5000);
	digitalWrite(10, LOW);

	//初始化所有模块
	Serial_Init();
	ADC_Init();
	CAN_Init();
	IMU_Init();
	Motor_InitAll();
	Ctrl_Init();
	BT_Init();
}

void loop()
{
	//LED闪烁次数表示电池电压
	for(int i = 0; i < (sourceVoltage - 11) / 0.5f + 1; i++)
	{
		digitalWrite(10, HIGH);
		vTaskDelay(100);
		digitalWrite(10, LOW);
		vTaskDelay(200);
	}
	digitalWrite(10, HIGH);
	vTaskDelay(100);
	digitalWrite(10, LOW);
	vTaskDelay(600);

	//检测到按钮按下，根据按下时长执行不同的操作
	if(digitalRead(9) == LOW)
	{
		int cmdCnt = 0;
		vTaskDelay(2000);
		while(digitalRead(9) == LOW) //等待按钮松开，闪烁LED并计算按下时长
		{
			cmdCnt++;
			digitalWrite(10, HIGH);
			vTaskDelay(300);
			digitalWrite(10, LOW);
			vTaskDelay(500);
		}
		vTaskDelay(2000);

		if(cmdCnt == 1) //功能1：设定目标航向角为当前值
		{
			target.yawAngle = imuData.yaw;
		}
		else if(cmdCnt == 2) //功能2：进入起立过程
		{
			xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
		}
		else if(cmdCnt == 3) //功能3：向前运动2s
		{
			target.speedCmd = 1.0f;
			vTaskDelay(2000);
			target.speedCmd = 0.0f;
		}
		else if(cmdCnt == 4) //功能4：向前运动5s
		{
			target.speedCmd = 1.0f;
			vTaskDelay(5000);
			target.speedCmd = 0.0f;
		}
		else if(cmdCnt == 5) //功能5：旋转90°
		{
			target.yawAngle += M_PI;
		}
	}
}

//真正的入口函数，调用setup和loop，模仿Arduino结构
extern "C" void app_main()
{
	setup();
	while (1)
	{
		loop();
	}
}
