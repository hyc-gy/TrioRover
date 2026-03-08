/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float Kp;//比例系数
	float Ki;//积分项系
	float Kd;//微分项系
	float SP;//用户设定
	uint64_t last_time;//上次运行PID的时闿
	float last_err;//上次的误巿
	float last_err_int;//上次误差的积分项
	float upper;//输出上限
	float lower;//输出下限	
}PID_Typedef;

typedef struct
{
	uint32_t pre_time;
	int32_t pre_cnt;
	int32_t update_cnt;
	float speed;
	uint32_t last_deltat;
	float deltas;
}MOTOR;
typedef struct {
	float yaw;
}angle_q;
typedef struct {
	float fb_yaw;
}yaw_q;
typedef struct {
	float SP_L;
	float SP_R;
}pid_q;
typedef struct {
	float target_x;
	float target_y;
}position_q;
typedef enum {
	Cmd_Char,
	Cmd_Coord
}CMD_Type;
typedef struct {
	CMD_Type type;
	union {
		char ch;
		char coord[16];
	}data;
}cmd_q;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t Ustick = 0;
volatile float ax, ay, az, temperature, gx, gy, gz;
volatile float pitch, roll;
//volatile float yaw=0;
MOTOR motor_l = { 0 };
MOTOR motor_r = { 0 };
uint8_t rx_byte;
volatile uint16_t BLE_Speed = 400;
volatile uint8_t get_ready = 0;
volatile int period_l = 0;
volatile int period_r = 0;
PID_Typedef PID_L = { 0 };
PID_Typedef PID_R = { 0 };
PID_Typedef PID_Turn = { 0 };
PID_Typedef PID_Move = { 0 };
//volatile float diff;
//volatile float distance;
volatile float vbat;
volatile float cur_x = 0;
volatile float cur_y = 0;
//volatile float cur_theta=0;
volatile float omega;
volatile float speed;
char cmd[16] = { 0 };
volatile uint8_t mode = 0, par_suc = 0;
//volatile float target_x=0,target_y=0;
TaskHandle_t Init_Handle, Sensor_Handle, Comm_Handle, PIDControl_Handle;
QueueHandle_t PID_Queue, Cmd_Queue, Angle_Queue, Position_Queue, Yaw_Queue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	static uint8_t i = 0;
	BaseType_t taskwoken = pdFALSE;
	if (huart->Instance == USART3)
	{
		if (mode == 0)
		{
			get_ready = 1;
			cmd_q order = { .type = Cmd_Char,.data.ch = rx_byte };
			xQueueOverwriteFromISR(Cmd_Queue, &order, &taskwoken);
			portYIELD_FROM_ISR(taskwoken);
		}
		else
		{
			if (rx_byte != ')')
			{
				cmd[i++] = rx_byte;
			}
			else
			{
				cmd[i++] = rx_byte;
				cmd[i] = '\0';
				i = 0;
				cmd_q order = { .type = Cmd_Coord };
				//				for(int a=0;a<15;a++)
				//				{
				//					order.data.coord[a]=cmd[a];
				//				}
				strcpy(order.data.coord, cmd);
				xQueueOverwriteFromISR(Cmd_Queue, &order, &taskwoken);
				portYIELD_FROM_ISR(taskwoken);
				get_ready = 1;
			}
		}
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM2)
	{
		Ustick += 65536;
	}
}

uint32_t GetUs(void)
{
	uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim2);
	return cnt + Ustick;
}

void reg_write(uint8_t reg, uint8_t value)
{
	uint8_t send_data[] = { reg,value };
	HAL_I2C_Master_Transmit(&hi2c1, 0xd0, send_data, 2, HAL_MAX_DELAY);
}

uint8_t reg_receive(uint8_t reg)
{
	HAL_I2C_Master_Transmit(&hi2c1, 0xd0, &reg, 1, HAL_MAX_DELAY);
	uint8_t receive_data;
	HAL_I2C_Master_Receive(&hi2c1, 0xd0, &receive_data, 1, HAL_MAX_DELAY);
	return receive_data;
}


void MPU_update(void)
{
	int16_t ax_raw = (int16_t)((reg_receive(0x3b) << 8) + reg_receive(0x3c));
	int16_t ay_raw = (int16_t)((reg_receive(0x3d) << 8) + reg_receive(0x3e));
	int16_t az_raw = (int16_t)((reg_receive(0x3f) << 8) + reg_receive(0x40));
	int16_t temper = (int16_t)((reg_receive(0x41) << 8) + reg_receive(0x42));
	int16_t gx_raw = (int16_t)((reg_receive(0x43) << 8) + reg_receive(0x44));
	int16_t gy_raw = (int16_t)((reg_receive(0x45) << 8) + reg_receive(0x46));
	int16_t gz_raw = (int16_t)((reg_receive(0x47) << 8) + reg_receive(0x48));
	ax = ax_raw * 6.1035e-5f;
	ay = ay_raw * 6.1035e-5f;
	az = az_raw * 6.1035e-5f;
	gx = gx_raw * 6.1035e-2f;
	gy = gy_raw * 6.1035e-2f;
	gz = gz_raw * 6.1035e-2f;
	temperature = temper / 333.87f + 21.0f;
}

float Eular_Compute(float yaw)
{
	//	static uint32_t nxt=0;
	//	if (HAL_GetTick()<nxt) return;
	MPU_update();
	//if (gz>3.5||gz<-3.5)
	yaw = yaw + gz * 0.01f;
	float pitch_g = pitch + gx * 0.01f;
	float roll_g = roll - gy * 0.01f;
	float pitch_a = atan2(ay, az) * 180.0f / 3.1415927f;
	float roll_a = atan2(ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.1415927f;
	pitch = 0.95238f * pitch_g + 0.04762f * pitch_a;
	roll = 0.95238f * roll_g + 0.04762f * roll_a;
	//printf("%f,%f,%f\n",yaw,roll,pitch);
	//	nxt+=5;
	return yaw;
	//注：左转为正，前边抬起为正，右边抬起为正   
}

//void gyro_compute(void)
//{
//	static uint32_t nxt=0;
//	if (HAL_GetTick()<nxt) return;
//	MPU_update();	
//	yaw=yaw+gz*0.005;
//	pitch=pitch+gx*0.005;
//	roll=roll-gy*0.005;
//	printf("%f,%f,%f\n",yaw,pitch,roll);
//	nxt+=5;
//}

//void acce_compute(void)
//{
//	yaw=0.0f;
//	pitch=atan2(ay,az)*180.0f/3.1415927f;
//	roll=atan2(ax,sqrtf(ay*ay+az*az))*180.0f/3.1415927f;
//}

int fputc(int ch, FILE* f)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 1000);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// 确认是否为KEY1按下
	BaseType_t taskwoken = pdFALSE;
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) {
		// 翻转绿灯 	
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if (mode == 0)
		{
			mode = 1;
			cur_x = 0.0;
			cur_y = 0.0;
			position_q position = { .target_x = 0,.target_y = 0 };
			xQueueOverwriteFromISR(Position_Queue, &position, &taskwoken);
			portYIELD_FROM_ISR(taskwoken);
		}
		else mode = 0;
		// 等待KEY1松开
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0);
	}
}


void Update_Speed_L(void)
{
	int32_t cur_cnt = __HAL_TIM_GET_COUNTER(&htim3);
	motor_l.deltas = ((int16_t)cur_cnt - (int16_t)motor_l.update_cnt) * 14.45132f / 530.4f;
	motor_l.update_cnt = cur_cnt;
	uint16_t deltac = abs((int16_t)cur_cnt - (int16_t)motor_l.pre_cnt);
	uint32_t cur_time = GetUs();
	if (deltac >= 2)
	{
		uint32_t deltat = cur_time - motor_l.pre_time;
		if (deltat == 0)deltat = 1;
		motor_l.last_deltat = deltat;
		float speed = (float)deltac * 360.0f * 1.0e6 / (530.4f * (float)deltat);
		//		float speed=(float)deltac*6283185.2f*0.023f/(1060.8f*(float)deltat);
		if (cur_cnt - motor_l.pre_cnt < 0)
		{
			speed = -speed;
		}
		motor_l.pre_cnt = cur_cnt;
		motor_l.pre_time = cur_time;
		motor_l.speed = 0.2 * speed + 0.8 * motor_l.speed;
	}
	else
	{
		if (deltac == 0)motor_l.speed = 0;
		else
		{
			if ((cur_time - motor_l.pre_time) > motor_l.last_deltat)
			{
				motor_l.speed = motor_l.speed * motor_l.last_deltat / (cur_time - motor_l.pre_time) / 2.0f;
			}
			else motor_l.speed = motor_l.speed / 2.0f;
		}
	}
}

void Update_Speed_R(void)
{
	int32_t cur_cnt = __HAL_TIM_GET_COUNTER(&htim4);
	motor_r.deltas = ((int16_t)cur_cnt - (int16_t)motor_r.update_cnt) * 14.45132f / 530.4f;
	motor_r.update_cnt = cur_cnt;
	uint32_t deltac = labs((int16_t)cur_cnt - (int16_t)motor_r.pre_cnt);
	uint32_t cur_time = GetUs();
	if (deltac >= 2)
	{
		uint32_t deltat = cur_time - motor_r.pre_time;
		motor_r.last_deltat = deltat;
		float speed = (float)deltac * 360.0f * 1.0e6 / (530.4f * (float)deltat);
		//		float speed=(float)deltac*6283185.2f*0.023f/(1060.8f*(float)deltat);
		if (cur_cnt - motor_r.pre_cnt < 0)
		{
			speed = -speed;
		}
		motor_r.pre_cnt = cur_cnt;
		motor_r.pre_time = cur_time;
		motor_r.speed = 0.2 * speed + 0.8 * motor_r.speed;
	}
	else
	{
		if (deltac == 0)motor_r.speed = 0;
		else
		{
			if ((cur_time - motor_r.pre_time) > motor_r.last_deltat)
			{
				motor_r.speed = motor_r.speed * motor_r.last_deltat / (cur_time - motor_r.pre_time) / 2.0f;
			}
			else motor_r.speed = motor_r.speed / 2.0f;
		}
	}
}

float PID_Compute(PID_Typedef* PID, float FB)
{
	float err = PID->SP - FB;
	uint64_t now = GetUs();
	float deltaT = (now - PID->last_time) * 1.0e-6;
	float err_dev = 0.0f;
	float err_int = 0.0f;
	if (PID->last_time)
	{
		err_dev = (err - PID->last_err) / deltaT;
		err_int = PID->last_err_int + (err + PID->last_err) * deltaT * 0.5;
		if (err_int > PID->upper)err_int = PID->upper;
		if (err_int < PID->lower)err_int = PID->lower;
	}
	float COp = PID->Kp * err;
	float COi = PID->Ki * err_int;
	float COd = PID->Kd * err_dev;
	float CO = COp + COi + COd;
	if (CO > PID->upper)CO = PID->upper;
	if (CO < PID->lower)CO = PID->lower;
	PID->last_time = now;
	PID->last_err = err;
	PID->last_err_int = err_int;
	return CO;
}
float PID_Compute2(PID_Typedef* PID, float err)
{
	uint64_t now = GetUs();
	float deltaT = (now - PID->last_time) * 1.0e-6;
	float err_dev = 0.0f;
	float err_int = 0.0f;
	if (PID->last_time)
	{
		err_dev = (err - PID->last_err) / deltaT;
		err_int = PID->last_err_int + (err + PID->last_err) * deltaT * 0.5;
		if (err_int > PID->upper)err_int = PID->upper;
		if (err_int < PID->lower)err_int = PID->lower;
	}
	float COp = PID->Kp * err;
	float COi = PID->Ki * err_int;
	float COd = PID->Kd * err_dev;
	float CO = COp + COi + COd;
	if (CO > PID->upper)CO = PID->upper;
	if (CO < PID->lower)CO = PID->lower;
	PID->last_time = now;
	PID->last_err = err;
	PID->last_err_int = err_int;
	return CO;
}
void Set_PWM_L(float duty)//pcb板画线改亿
{
	if (duty < 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fabsf(duty));
}
void Set_PWM_R(float duty)
{
	if (duty < 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fabsf(duty));
}
void PID_Pro(PID_Typedef* PID_l, float FB_l, PID_Typedef* PID_r, float FB_r)
{
	//	static uint32_t nxt2=0;
	//	if (HAL_GetTick()<nxt2) return;
	period_l = PID_Compute(PID_l, FB_l);
	period_r = PID_Compute(PID_r, FB_r);
	Set_PWM_L(period_l);
	Set_PWM_R(period_r);
	//	nxt2+=5;
}
float Get_Voltage(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	float voltage = HAL_ADC_GetValue(&hadc1) * 0.008833f;   /*3.3*11/4096*/
	return voltage;
}
void Update_SP(float target_x, float target_y, float yaw)
{
	float deltax = target_x - cur_x;
	float deltay = target_y - cur_y;
	float distance = sqrtf(deltax * deltax + deltay * deltay);
	float target_theta = atan2(deltay, deltax);
	float cur_theta = (yaw + 0.042) * 3.1415926f / 180.0f;
	float delta_theta = target_theta - cur_theta;
	//	printf("%f\n",cur_theta);
	while (delta_theta > 3.1415926f)delta_theta -= 6.2831852f;
	while (delta_theta < -3.1415926f)delta_theta += 6.2831852f;
	//  if(distance>50)
	//	{
	//		if(fabs(delta_theta)>0.5f)
	//		{
	//			speed=0.1f;
	//			omega=PID_Compute2(&PID_Turn,delta_theta);
	//		}
	//		else 
	//		{
	//			speed=0.24f;
	//			omega=PID_Compute2(&PID_Turn,delta_theta);
	//		}
	//	}
	//	else if(distance>10)
	//	{
	//		float radius=distance/(2*sin(fabs(delta_theta)/2.0f));
	//		if(radius<15)
	//		{
	//			speed=0.15;
	//			omega=(speed/0.15)*(delta_theta>0?1.0f:-1.0f);
	//		}
	//	}
	//	else
	//		{
	//			speed=PID_Compute2(&PID_Move,distance);
	//			omega=PID_Compute2(&PID_Turn,delta_theta);
	//			if(distance<2)
	//			{
	//				speed=0;
	//				omega=0;
	//			}
	//	  }
	//	
	//	if(delta_theta>0.05f)
	//	{
	//		speed=0;
	//		omega=0.2;
	//	}
	//	else
	//	{
	//		omega=0;
	//		if(distance>20)
	//		speed=0.2;
	//		else
	//		speed=PID_Compute2(&PID_Move,distance);
	//	}
	//  if(distance>30)
	//	{
	//		speed=0.1f;
	//		omega=PID_Compute2(&PID_Turn,delta_theta);
	//	}
	//	else if(distance>5)
	//	{
	//		speed=PID_Compute2(&PID_Move,distance);
	//		if(delta_theta>0.1f)
	//		{
	//			omega=PID_Compute2(&PID_Turn,delta_theta);
	//		}
	//	}
	//	else 
	//	{
	//		speed=0;
	//	}
	//	if(delta_theta<0.1)
	//	{
	//		omega=0;
	//	}
	if (fabsf(delta_theta) > 1.0f)
	{
		speed = 0;
		omega = PID_Compute2(&PID_Turn, delta_theta);
	}
	else
	{
		if (distance > 20)
			speed = 0.24;
		else {
			if (distance > 2)
				speed = PID_Compute2(&PID_Move, distance);
			else speed = 0.0f;
		}
		//			if(fabsf(delta_theta)>0.7)
		//			{
		//			PID_Turn.Kp=2;
		//			omega=PID_Compute2(&PID_Turn,delta_theta);
		//			}
		if (fabsf(delta_theta) > 0.1)
		{
			omega = PID_Compute2(&PID_Turn, delta_theta);
		}
		else
		{
			omega = 0;
		}
	}
	PID_L.SP = (speed - 0.067f * omega) * 360.0f / (3.1415926f * 0.046f);
	PID_R.SP = (speed + 0.067f * omega) * 360.0f / (3.1415926f * 0.046f);
}

void printf_test(void)
{
	volatile static uint32_t nxt1 = 0;
	if (HAL_GetTick() < nxt1) return;
	//HAL_UART_Transmit(&huart3,(uint8_t*)&yaw,1,1000);
	//printf("%f,%f,%f\n",cur_x,yaw,(yaw+0.042)*3.1415926f/180.0f);
	//printf("%f,%f,%f,%f,%f,%f,%f,%f\n",motor_l.speed,motor_r.speed,PID_L.SP,speed,gz,yaw,roll,pitch);
	nxt1 += 1000;
}
void Task_Sensor(void)
{
	angle_q angle;
	TickType_t LastTime1 = xTaskGetTickCount();
	yaw_q yaw_fb = { 0 };
	while (1)
	{
		MPU_update();
		xQueueReceive(Yaw_Queue, &yaw_fb, 0);
		angle.yaw = Eular_Compute(yaw_fb.fb_yaw);
		xQueueOverwrite(Angle_Queue, &angle);
		//vTaskDelayUntil(&LastTime1,pdMS_TO_TICKS(5));
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
void Task_Comm(void)
{
	cmd_q cmd;
	float target_x, target_y;
	float newsp_l, newsp_r;
	while (1)
	{
		if (xQueueReceive(Cmd_Queue, &cmd, portMAX_DELAY) == pdPASS)
		{
			if (cmd.type == Cmd_Char)
			{
				switch (cmd.data.ch)
				{
				case 'A':newsp_l = BLE_Speed;
					newsp_r = BLE_Speed;
					break;
				case 'B':newsp_l = BLE_Speed;
					if (BLE_Speed >= 200)
						newsp_r = BLE_Speed - 200.0;
					else newsp_r = 0;
					break;
				case 'C':newsp_l = 200;
					newsp_r = -200;
					break;
				case 'D':newsp_l = -BLE_Speed;
					if (BLE_Speed >= 200)
						newsp_r = -BLE_Speed + 200;
					else newsp_r = 0;
					break;
				case 'E':newsp_l = -BLE_Speed;
					newsp_r = -BLE_Speed;
					break;
				case 'F':if (BLE_Speed >= 200)
					newsp_l = -BLE_Speed + 200;
						else newsp_l = 0;
					newsp_r = -BLE_Speed;
					break;
				case 'G':newsp_l = -200;
					newsp_r = 200;
					break;
				case 'H':if (BLE_Speed >= 200)
					newsp_l = BLE_Speed - 200;
						else newsp_l = 0;
					newsp_r = BLE_Speed;
					break;
				case 'X':if (BLE_Speed <= 950)
					BLE_Speed += 50;
					break;
				case 'Y':if (BLE_Speed >= 50)
					BLE_Speed -= 50;
					break;
				case 'Z':newsp_l = 0;
					newsp_r = 0;
					BLE_Speed = 400;
					break;
				default:newsp_l = 0;
					newsp_r = 0;
					break;
				}
				pid_q sp = { .SP_L = newsp_l,.SP_R = newsp_r };
				xQueueOverwrite(PID_Queue, &sp);
			}
			else if (cmd.type == Cmd_Coord)
			{
				if (strncasecmp(cmd.data.coord, "( ", 2) == 0)
				{
					if (sscanf(cmd.data.coord, "( %f,%f )", &target_x, &target_y) == 2)
					{
						position_q position = { .target_x = target_x,.target_y = target_y };
						xQueueOverwrite(Position_Queue, &position);
						par_suc = 1;
					}
					else par_suc = 0;
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
void Task_PIDControl(void)
{
	float target_x;
	float target_y;
	float yaw;
	int n = 0;
	float yaw_b = 0;
	pid_q pid;
	yaw_q yaw_fb;
	position_q position;
	angle_q angle;
	TickType_t LastTime2 = xTaskGetTickCount();
	while (1)
	{
		n++;
		if (xQueueReceive(Angle_Queue, &angle, 0) == pdPASS)
		{
			yaw = angle.yaw;
		}
		if (xQueueReceive(PID_Queue, &pid, 0) == pdPASS)
		{
			PID_L.SP = pid.SP_L;
			PID_R.SP = pid.SP_R;
		}
		else if (xQueueReceive(Position_Queue, &position, 0) == pdPASS)
		{
			target_x = position.target_x;
			target_y = position.target_y;
		}
		Update_Speed_L();
		Update_Speed_R();
		yaw_b += (motor_r.deltas - motor_l.deltas) / 0.233f;
		yaw = 0.98f * yaw + 0.02f * yaw_b;
		yaw_fb.fb_yaw = yaw;
		xQueueOverwrite(Yaw_Queue, &yaw_fb);
		//if(n%5==0)
		//printf("%f\n",motor_l.speed);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		//vTaskDelay(pdMS_TO_TICKS(5));
		cur_x += (motor_l.deltas + motor_r.deltas) / 2 * cosf((yaw + 0.042) * 3.1415926f / 180.0f);
		cur_y += (motor_l.deltas + motor_r.deltas) / 2 * sinf((yaw + 0.042) * 3.1415926f / 180.0f);
		if (mode == 1 && par_suc == 1) {
			Update_SP(target_x, target_y, yaw);
		}
		PID_Pro(&PID_L, motor_l.speed, &PID_R, motor_r.speed);
		vTaskDelay(pdMS_TO_TICKS(10));
		//vTaskDelayUntil(&LastTime2,5);
	}
}
void Task_Init(void)
{
	PID_L.Kp = 0.15;
	PID_L.Ki = 3.3;
	PID_L.Kd = 0;
	PID_R.Kp = 0.2;
	PID_R.Ki = 3.35;
	PID_R.Kd = 0.005;
	PID_Move.Kp = 0.012;
	PID_Move.Ki = 0;
	PID_Move.Kd = 0;
	PID_Turn.Kp = 3;
	PID_Turn.Ki = 0;
	PID_Turn.Kd = 0;
	PID_Turn.lower = -5.9806;
	PID_Turn.upper = 5.9806;
	PID_Move.lower = -0.4014;
	PID_Move.upper = 0.4014;
	vbat = Get_Voltage();
	PID_L.upper = 1000;
	PID_L.lower = -1000;
	PID_R.upper = 1000;
	PID_R.lower = -1000;
	uint8_t rx_copy = rx_byte;
	char cmd_cpy[16] = { 0 };
	volatile float  BLE_Speed = 400.0;
	reg_write(0x6b, 0x80);   //复位
	vTaskDelay(pdMS_TO_TICKS(100));         //等待复位
	reg_write(0x6b, 0x00);   //关闭休眠模式
	reg_write(0x1b, 0x18);   //螺仪量程 :+-2000°/s
	reg_write(0x1c, 0x00);   //加度计量程：+-2g
	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
	PID_Queue = xQueueCreate(4, sizeof(pid_q));
	Angle_Queue = xQueueCreate(4, sizeof(angle_q));
	Cmd_Queue = xQueueCreate(4, sizeof(cmd_q));
	Position_Queue = xQueueCreate(4, sizeof(position_q));
	Yaw_Queue = xQueueCreate(4, sizeof(yaw_q));
	xTaskCreate((TaskFunction_t)Task_Sensor, "Sensor", 512, NULL, 3, &Sensor_Handle);
	xTaskCreate((TaskFunction_t)Task_Comm, "Comm", 512, NULL, 2, &Comm_Handle);
	xTaskCreate((TaskFunction_t)Task_PIDControl, "PIDControl", 2048, NULL, 4, &PIDControl_Handle);
	vTaskDelete(NULL);
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
  //  PID_L.Kp=0.5;
  //  PID_L.Ki=1.91;
  //	PID_L.Kd=0.01;
  //  PID_R.Kp=0.5;
  //  PID_R.Ki=1.91;
  //  PID_R.Kd=0.01;
  //	PID_Move.Kp=0.012;
  //	PID_Move.Ki=0;
  //	PID_Move.Kd=0;
  //	PID_Turn.Kp=2;
  //	PID_Turn.Ki=0;
  //	PID_Turn.Kd=0;
  //  PID_Turn.lower=-5.9806;
  //  PID_Turn.upper=5.9806;
  //	PID_Move.lower=-0.4014;
  //	PID_Move.upper=0.4014;
  //  vbat=Get_Voltage();
  //  PID_L.upper=1000;
  //  PID_L.lower=-1000;
  //  PID_R.upper=1000;
  //  PID_R.lower=-1000;
  //  uint8_t rx_copy=rx_byte;
  //  char cmd_cpy[16]={0};
  //	volatile float  BLE_Speed=400.0;
  //	reg_write(0x6b,0x80);   //复位
  //	//HAL_Delay(100);         //等待复位
  //  reg_write(0x6b,0x00);   //关闭休眠模式
  //  reg_write(0x1b,0x18);   //螺仪量程 :+-2000°/s
  //  reg_write(0x1c,0x00);   //加度计量程：+-2g
  //	HAL_UART_Receive_IT(&huart3,&rx_byte,1);
  //  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  //	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  //	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
  //	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2);
  //	HAL_TIM_Base_Start_IT(&htim2);
	xTaskCreate((TaskFunction_t)Task_Init, "Init", 512, NULL, 5, &Init_Handle);
	vTaskStartScheduler();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//		if(get_ready==1)
		//		{
		//		get_ready=0;
		//		if(mode==0)
		//		{
		//		rx_copy=rx_byte;		
		//		switch(rx_copy)
		//    {
		//			case 'A':PID_L.SP=BLE_Speed;
		//			         PID_R.SP=BLE_Speed;
		//				       break;
		//			case 'B':PID_L.SP=BLE_Speed;
		//			         if(BLE_Speed>=200)
		//			         PID_R.SP=BLE_Speed-200.0;
		//							 else PID_R.SP=0;
		//				       break;
		//			case 'C':PID_L.SP=200;
		//			         PID_R.SP=-200;
		//				       break;
		//			case 'D':PID_L.SP=-BLE_Speed;
		//							 if(BLE_Speed>=200)
		//			         PID_R.SP=-BLE_Speed+200;
		//							 else PID_R.SP=0;
		//							 break;
		//			case 'E':PID_L.SP=-BLE_Speed;
		//			         PID_R.SP=-BLE_Speed;
		//				       break;
		//			case 'F':if(BLE_Speed>=200)
		//				       PID_L.SP=-BLE_Speed+200;
		//			         else PID_L.SP=0;
		//			         PID_R.SP=-BLE_Speed;
		//				       break;
		//			case 'G':PID_L.SP=-200;
		//			         PID_R.SP=200;
		//				       break;
		//			case 'H':if(BLE_Speed>=200)
		//				       PID_L.SP=BLE_Speed-200;
		//			         else PID_L.SP=0;
		//			         PID_R.SP=BLE_Speed;
		//				       break;
		//			case 'X':if(BLE_Speed<=950)
		//				       BLE_Speed+=50;
		//				       break;
		//			case 'Y':if(BLE_Speed>=50)
		//				       BLE_Speed-=50;
		//				       break;
		//			case 'Z': PID_L.SP=0;
		//			          PID_R.SP=0;
		//                BLE_Speed=400;			
		//				       break;
		//			default :PID_L.SP=0;
		//			         PID_R.SP=0;
		//				       break;
		//    }
		//	  }
		//		else
		//			{
		//				strcpy(cmd_cpy,cmd);
		//				if(strncasecmp(cmd_cpy,"( ",2)==0)
		//				{
		//					if(sscanf(cmd_cpy,"( %f,%f )",&target_x,&target_y)==2)
		//					{
		//						diff=sqrt((target_x-cur_x)*(target_x-cur_x)+(target_y-cur_y)*(target_y-cur_y));
		//						if(diff<10.0f)diff=10.0f;
		//						par_suc=1;
		//					}
		//					else par_suc=0;
		//				}
		//		  }	
		//	  }
		//		if(mode==1 && par_suc ==1)
		//		{
		//			Update_SP(target_x,target_y);
		//		}
		//		Eular_Compute();
		//		Update_Speed_L();
		//		Update_Speed_R();
		//		cur_x+=(motor_l.deltas+motor_r.deltas)/2*cosf((yaw+0.042)*3.1415926f/180.0f);
		//		cur_y+=(motor_l.deltas+motor_r.deltas)/2*sinf((yaw+0.042)*3.1415926f/180.0f);
		//		yaw_b+=(motor_r.deltas-motor_l.deltas)/0.233f;
		//		yaw=0.98f*yaw+0.02f*yaw_b;
		//    PID_Pro(&PID_L,motor_l.speed,&PID_R,motor_r.speed);

		//		printf_test();
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	__disable_irq();
	while (1)
	{
	}
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
