/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include <stdlib.h>
#include "EEPROM.h"
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define					SET								1
	#define					NULL							0

	#define					HEARTBEAT					0x01
	#define					CMD_MASK					0x01f		
	#define					BT_READ						HAL_GPIO_ReadPin(UART_State_GPIO_Port,UART_State_Pin);
	#define					BUZZER_OFF				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	
	#define					BUZZER_ON					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	
	#define					BUZZER_TOGGLE			HAL_GPIO_TogglePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin );									HAL_GPIO_TogglePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin);
	#define					ENGAGE_BRAKE_VERTICAL			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);	
	#define					DISENGAGE_BRAKE_VERTICAL	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	#define					ENGAGE_BRAKE_CONTOUR			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);	
	#define					DISENGAGE_BRAKE_CONTOUR	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	
	
	#define					LSB												0
	#define					MSB	 											1
	#define					REMOTE										1
	#define					DATA											2
	#define					STEERING_BOUNDARY 				2
	#define					STEERING_HOMING_SPEED	 	  15
	#define					STEERING_KP			 					3
	#define					STEERING_MAX_VEL					50
	
	#define					VELOCITY					0
	#define					TORQUE						1
	#define					VEL_LIMIT					2
	#define					MOT_ERROR					3
	#define					ENC_ERROR					4
	#define					IQ								5
	#define					ENC_EST						6
	#define					T_RAMP						7
	#define					REQ_STATE					8
	#define					SNL_ERROR					9
	#define					SENSL_EST					10
	#define					POSITION					11
	
	#define					HEARTBEAT					0x01
	#define					POS_ID						0x0C
	#define					VEL_ID						0x0D
	#define					TRQ_ID						0x0E
	#define					VLMT_ID						0x0F
	#define					MERR_ID						0x03
	#define					ENERR_ID					0x04
	#define					SNERR_ID					0x05
	#define					IQM_ID						0x14
	#define					ENEST_ID					0x009
	#define					T_RAMP_ID					0x1C
	#define					REQ_STATE_ID			0x07
	#define					SENS_EST					0x015
	#define					VOLTAGE					  0x017

	#define					ALL_WHEEL					1
	#define					CRAB							3
	#define					ZERO_TURN					2
	#define					WIDTH_SHRINK			4
	#define					WIDTH_EXTEND			5
	
	#define     Half_Wheel_Base                      850
	#define     Half_Wheel_Track                     1218  // 1218 - Min Width		1368 - Max Width
	#define     Pi                                   3.141592654
	#define     Steering_Reductions                  76
	#define     Wheel_Reductions                     114
	
	

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
uint8_t BT_Rx[8];
bool BT_State=0 ,Prev_BT_State=0;
uint16_t BT_Count=0;


/* 							CAN_VARIABLES 						*/

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t TxData2[8];
uint8_t RxData2[8];
uint8_t RxData2_Temp[8];
uint8_t RxData_Temp[8];
uint32_t TxMailbox, CAN_Count=0;
uint8_t Node_Id[23],Prev_Node_Id[22], Received_Node_Id=0, Received_Command_Id=0;
uint8_t Sensor_Id[10], Axis_State[20];
float Motor_Velocity[20], Rover_Voltage=0, Rover_Voltage_Temp=0;;uint8_t Motor_Error[20], Encoder_Error[20] , Motor_Current[20], Volt_Tx=0, Volt_Tx_Temp=0;
uint8_t LFD=1,LRD=2,RFD=3,RRD=4,LVert=5, RVert=6, Contour=7, LFS=8, LRS=9, RFS=10, RRS=11, L_Arm=12, R_Arm=13, P_Arm=14 , Upper_Width =16 , Lower_Width = 15, Cutter=17, Side_Belt = 18, Selective = 19, Paddle =20;

/* 							CAN_VARIABLES 						*/


/* 							JOYSTICK_VARIABLES 						*/
uint8_t Mode=1,Mode_Temp, Speed=1, Joystick=0, Steering_Mode=1, Shearing=0, Skiffing=0, Side_Trimmer=0, Pot_Angle=90, Joystick_Temp=0, Shearing_Temp=0, Steering_Mode_Temp=1, BT_Steer_Temp=0;		 
/* 							JOYSTICK_VARIABLES 						*/


float  Macro_Speed = 0, Macro_Speed_Temp=0;

/* 							DRIVE_WHEELS_VARIABLES 						*/
bool DRIVES_ERROR_FLAG = NULL;
float L_R_Err=0, R_R_Err=0, C_Err=0, Contour_Avg=0, Drive_Torque=1, Wheel_Torque = 3,Left_Wheel_Torque=1;
float Vel_Limit=10, Vel_Limit_Temp=1, Torque=0, Torque_Temp=0 , Prev_Torque=0, Prev_Vel_Limit=30;
int Left_Wheels_Torque =0, Left_Wheels_Torque_Temp=0;

/* 							DRIVE_WHEELS_VARIABLES 						*/



/* 							BT_VARIABLES 						*/
uint8_t BT_Rx[8], RxBuff[8];
/* 							BT_VARIABLES 						*/

float Absolute_Position[20];
int16_t Absolute_Position_Int[20];

uint8_t node_id_BLE,command_id_BLE;
float Left_Encoder=0,Left_IMU=0,Right_Encoder=0,Right_IMU=0;
float Left_roll_value,Left_pitch_value,Right_roll_value,Right_pitch_value,roll_value=0,pich_value=0;
uint32_t Left_Roll_Int=0, Left_Pitch_Int=0,Right_Roll_Int=0,Right_Pitch_Int=0,Roll_Int=0,Pich_Int=0;

uint16_t Left_IMU_Node=0,Left_Encoder_Node=0,Right_Encoder_Node=0,Right_IMU_Node=0,Encoder=0;
uint8_t Buzzer_Acivated=0;


bool Left_IMU_State=1;
int Left_Steering_Speed=0, Right_Steering_Speed=0, Left_Frame_Speed =0;
uint8_t Left_Vel_Limit = 10, Right_Vel_Limit=10, Prev_Left_Vel_Limit = 30, Prev_Right_Vel_Limit = 30, Left_Transmit_Vel=0, Right_Transmit_Vel=0;

int Position=0, Position_Temp =0;


double Right_Steer_Angle = 0;
float Right_Turn_Radius = 0, Chord_Dist=0, Turning_Radius=0, Right_Motor_Position=0;
float Right_Front_Steer_Pos = 0, Right_Rear_Steer_Pos=0, Right_Front_Steer_Pos_Temp=0, Right_Rear_Steer_Pos_Temp=0; ;
double Rover_Centre_Dist=0, TimeTaken=0, Inner_Speed=0, Outer_Speed=0;
double Mean_kmph =0;

uint8_t Brake_Check=0,Brake_Check_Temp=1;

int8_t Read_Value[28], Write_Value[28], Prev_Write_Value[28];
bool Store_Data = 0;

/* 							FRAME_VARIABLES 						*/
bool IMU_Reception_State = 1;
float R_Vert_Error=0,L_Vert_Error, Right_Roll_Home_Pos =-1.1,Left_Roll_Home_Pos =180, Right_Roll=-1,Left_Roll=-1,R_Contour_Error=0,L_Contour_Error=0,Right_Pitch_Home_Pos =1.56,Left_Pitch_Home_Pos =-3,Right_Pitch=-1,Left_Pitch=-1;
float Vert_Bandwidth = 1,Contour_Bandwidth = 1, Right_Vert_Pos=0,Left_Vert_Pos=0,Right_Contour_Pos=0,Left_Contour_Pos=0,Right_Vert_Pos_Temp=0,Left_Vert_Pos_Temp=0,Left_Contour_Pos_Temp=0,Right_Contour_Pos_Temp=0,Current_Vert_Pos = 0,Current_Right_Contour_Pos = 0,Current_Left_Contour_Pos = 0, R_Kp=1, Current_Vert_Angle=0,Current_Right_Contour_Angle=0,Current_Left_Contour_Angle=0;
int Right_Vert_Vel_Limit = 2,Frame_Vel_Limit=2;
float Upper_Width_Motor_Speed = 0, Upper_Width_Motor_Speed_Temp=0;

/* 							FRAME_VARIABLES 						*/


uint32_t Previous_ToggleTime = 0,Last_Update_Time=0,current_time=0;
uint8_t Led_State = 0;
uint64_t a=0;
  int changed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times);
void Drives_Error_Check(void);
void Reboot (int Axis);
void Heal_Error(uint8_t Axis_Id);
void Stop_Motors(void);
void Joystick_Reception(void);
void Wheel_Controls (void);
void Macro_Controls (void);
float convertRawDataToFloat(uint8_t* data);
uint16_t readEncoderValue(uint8_t* data);
void Steering_Controls(void);
void New_Drive_Controls(void);
void Brake_Controls(void);
void Frame_Controls(void);
void Frame_Controls_Velocity_Based(void);
void Dynamic_Width_Adjustment (void);
void checkNodeIds(void);
void Frame_Control_Position_Adjust(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format)
{
//uint64_t BUFF;
	TxHeader.ExtId = NULL;
	
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxHeader.IDE = CAN_ID_STD;
	
	TxHeader.DLC	= Data_Size;
	
	TxHeader.RTR = (Frame_Format == REMOTE) ? (CAN_RTR_REMOTE) : (Frame_Format == DATA) ? (CAN_RTR_DATA):(CAN_RTR_REMOTE);
	
	switch (Command)
	{
		case VELOCITY:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| VEL_ID;
									break;
		case POSITION:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| POS_ID;
									break;
		
		case TORQUE:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| TRQ_ID;
							//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
									break;
		
		case VEL_LIMIT:	

									//TxData[6] = 0x20; for 40 amps//8C - 70//0x70-60 amps
									TxData[6] = 0x8C;
									TxData[7] = 0x42;
									memcpy (TxData, &Tx_Data, 4);	
									TxHeader.DLC	= 8;
									TxHeader.StdId = (NODE << 5)| 0x00F;
									break;
		
		case MOT_ERROR: 	
									TxHeader.StdId = (NODE << 5)| MERR_ID;
									break;
		
		case ENC_ERROR:					
									TxHeader.StdId = (NODE << 5)| ENERR_ID;
									break;
		
		case SNL_ERROR:					
									TxHeader.StdId = (NODE << 5)| SNERR_ID;
									break;
		
		case IQ:						
									TxHeader.StdId = (NODE << 5)| IQM_ID;
									break;
		
		case ENC_EST:					
									TxHeader.StdId = (NODE << 5)| ENEST_ID;
									break;
									
		case T_RAMP:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| T_RAMP_ID;
									break;
		
		case SENSL_EST:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| 0x015;
									break;
		
		case REQ_STATE:	 
									break;
		
		case 0x017:
									TxHeader.StdId = (NODE << 5)| 0x017;
									break;
		
		default: break;
		

	}
	
		for ( uint8_t i=0 ; i<3; i++ ) 
	{

				HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox); 
				HAL_Delay(1);//optional
				CAN_Count++;
	}
	
	for ( uint8_t i=0 ; i<8; i++ ) 
	{
		TxData[i] = 0;
	}
				

}

float CAN_Reception(uint8_t byte_choice)
{
		float Can_Temp;
	
	if ( byte_choice == LSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else if ( byte_choice == MSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k+4];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else { }

	return(Can_Temp);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
			HAL_UART_Receive_IT(&huart4,BT_Rx ,sizeof(BT_Rx));
			BT_Count++;
}

void Absolute_Position_Reception( uint8_t Node_Id )
{
  memcpy(&Absolute_Position[Node_Id],RxData2, sizeof(float)); 
	Absolute_Position_Int[Node_Id] = Absolute_Position[Node_Id]; 
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan2)
{
	HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO1, &RxHeader2, RxData2); // changed


	Received_Node_Id = RxHeader2.StdId >> 5;
	Received_Command_Id = RxHeader2.StdId & CMD_MASK;
	
    node_id_BLE = (RxHeader2.StdId >> 1) & 0x0F; 
    command_id_BLE = RxHeader2.StdId & 0x01;
/*
//	if(RxHeader2.StdId == 0x03){ 
//		Left_Roll_Int = (RxData2[0] << 24) | (RxData2[1] << 16) | (RxData2[2] << 8) | RxData2[3];
//		Left_Pitch_Int =  (RxData2[4] << 24) | (RxData2[5] << 16) | (RxData2[6] << 8) | RxData2[7];
//		
//		Left_roll_value = (*((float*)&Left_Roll_Int));
//		Left_pitch_value = *((float*)&Left_Pitch_Int);
//		Left_IMU_Node++;
//	}
//	if(RxHeader2.StdId == 0x05){
//	Right_Roll_Int = (RxData2[0] << 24) | (RxData2[1] << 16) | (RxData2[2] << 8) | RxData2[3];
//		Right_Pitch_Int =  (RxData2[4] << 24) | (RxData2[5] << 16) | (RxData2[6] << 8) | RxData2[7];
//		
//		Right_roll_value = (*((float*)&Right_Roll_Int));
//  	Right_pitch_value = *((float*)&Right_Pitch_Int);
//		Right_Encoder_Node++;
//	}
//	if ( RxHeader2.StdId == 0x02 ) {
//		Left_Encoder_Node++ ;
//		Left_Encoder= (RxData2[0] << 8) | RxData2[1];	
//	}  
//	if ( RxHeader2.StdId == 0x04 ){ 
//		Right_Encoder_Node++ ;
//	  Right_Encoder= (RxData2[0] << 8) | RxData2[1]; 
//	}*/
	switch (RxHeader2.StdId) 
	{
    case 0x03: // Left IMU
        Left_Roll = convertRawDataToFloat(RxData2);
        Left_Pitch = convertRawDataToFloat(&RxData2[4]);
        Left_IMU_Node++; Sensor_Id[1]++;
        break;

    case 0x05: // Right IMU
//        Right_roll_value = convertRawDataToFloat(RxData2);
//        Right_pitch_value = convertRawDataToFloat(&RxData2[4]);
		    Right_Roll = ((int16_t)(RxData2[1]<<8 | RxData2[0]))/16.0;	
		    Right_Pitch = ((int16_t)(RxData2[3]<<8 | RxData2[2]))/16.0;	
        Right_IMU_Node++; Sensor_Id[3]++;
        break;

    case 0x02: // Left Encoder
        Left_Encoder = readEncoderValue(RxData2);
        Left_Encoder_Node++; Sensor_Id[2]++;
        break;

    case 0x04: // Right Encoder
        Right_Encoder = readEncoderValue(RxData2);
        Right_Encoder_Node++; Sensor_Id[4]++;
        break;

    default:						 break;
}
			
	switch( Received_Command_Id )
	{
		case HEARTBEAT:  							Node_Id[Received_Node_Id]++;    Axis_State[Received_Node_Id] = RxData2[4]; break;
		
		case ENEST_ID:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB);  Absolute_Position_Reception (	Received_Node_Id ); 	   break;		

//		case SENS_EST:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB); 				  			 	 break;
//		
//		case MERR_ID:  								Motor_Error[Received_Node_Id]			= CAN_Reception(MSB); 								 	 break;
//		
//		case ENERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(MSB); 					 			 	 break;
//		
//		case SNERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(LSB); 					  		 	 break;
//		
//		case IQM_ID:  								Motor_Current[Received_Node_Id]		= CAN_Reception(MSB); 					  		 	 break;
//		
//		case VOLTAGE: 								memcpy(&Rover_Voltage, RxData2, 4);	 																		 	 break;

		default: 																																													  	 	 break;

	}
}

void Set_Motor_Torque ( uint8_t Axis , float Torque )
{
	Torque =  (Axis==0) || (Axis==3)? -Torque : Torque ;	

	CAN_Transmit(Axis,TORQUE,-Torque,4,DATA);// osDelay(1);//5
}
void Set_Motor_Velocity ( uint8_t Axis , float Velocity )
{
//	if ( Axis == LFS ) 				CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA);
//	else if ( Axis == LRS ) 	CAN_Transmit(Axis,VELOCITY,-Velocity,4,DATA);
//	else if ( Axis == RRS ) 	CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA);
//	//else if ( Axis == 13 ) 	CAN_Transmit(Axis,VELOCITY,-Velocity,4,DATA);
//	else if (Axis >= 19){CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA);}
		CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA);//osDelay(10);

}
void Set_Motor_Position ( uint8_t Axis , float Position )
{
		CAN_Transmit(Axis,POSITION,Position,4,DATA);
}
void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times)
{
				memcpy(TxData, &command_id, 4);		
				TxHeader.DLC = 4;	
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = ( axis_id <<5) | 0x007 ;	
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);HAL_Delay(20); 		
}

float convertRawDataToFloat(uint8_t* data) {
    int32_t raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    return *((float*)&raw);
}
uint16_t readEncoderValue(uint8_t* data) {
    return (data[0] << 8) | data[1];
}

void Node_Id_Check()
{
  for(int i=0;i<13;i++)
	{if(Node_Id[i] ==0) Buzzer_Acivated=1;}
	
	if(Buzzer_Acivated){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);}
  else{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);}
}

void Brake_Controls()
{
	if(Brake_Check==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);}
	else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);}
//	if(Axis_State[4]!=8){Brake_Check=0;}
//	if(Brake_Check!=Brake_Check_Temp){ memcpy(&Write_Value[0], &Brake_Check, sizeof(Brake_Check));
//		Brake_Check_Temp=Brake_Check;
//	}
	
	
	
	/* Play with Clocks per sec and engage brake while heartbeat stops for 1 sec */
	
//	if ( Axis_State[4] != 8 ) {ENGAGE_BRAKE_VERTICAL;}
//	else {DISENGAGE_BRAKE_VERTICAL;}
//		if ( Axis_State[5] != 8 || Axis_State[6] != 8  ) {ENGAGE_BRAKE_CONTOUR;}
//	else {DISENGAGE_BRAKE_CONTOUR;}
//	

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//	for ( uint8_t i = 0 ; i < 255 ; i++ ) { EEPROM_PageErase(i)	; }
	HAL_Delay(600);
//	HAL_CAN_Start(&hcan1);HAL_Delay(100);
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);HAL_Delay(100);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	
	for ( uint8_t i = 12 ; i < 14; i++ ) { Start_Calibration_For (i, 8, 5);}
	HAL_Delay(1000);
	/*         Time        */
	Previous_ToggleTime = HAL_GetTick();
	 Last_Update_Time = HAL_GetTick();
    for (int i = 0; i < 23; i++) {
        Prev_Node_Id[i] = Node_Id[i];
    }
		/*         Time        */
	/* UART INITS */
	MX_UART4_Init();
	MX_UART5_Init();
	HAL_UART_Receive_IT(&huart4,BT_Rx ,sizeof(BT_Rx));
	/* UART INITS */
	HAL_Delay(2000);
	BUZZER_OFF;
	Left_IMU_State = 1;
	
//	for ( uint8_t i = 0 ; i < 255 ; i++ ) { EEPROM_PageErase(i)	; }
	
	HAL_Delay(1000);
//  Write_Value[0] =1;
//  Write_Value[1]=2;

//   EEPROM_Write(15, 0, (uint8_t *)Write_Value, sizeof(Write_Value));
//	HAL_Delay(1000);
//		EEPROM_Read(15, 0, (uint8_t *)Read_Value, sizeof(Read_Value));
//	Left_IMU_State = ( Sensor_Id[1] == 0 || Sensor_Id[2] || Sensor_Id[3] == 0 || Sensor_Id[4]   == 0 ) ? NULL : SET ;
//	if ( !Left_IMU_State ) Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		Joystick_Reception();
//		Macro_Controls();
		Steering_Controls();
		New_Drive_Controls();
//		Frame_Control_Position_Adjust();
		 Brake_Controls();
//		checkNodeIds();
//		Frame_Controls_Velocity_Based();
//		Frame_Controls();
//		if ( Position_Temp != Position )
//		{
//			Set_Motor_Position ( 8 , Position );
//			Position_Temp = Position;
		
//		Drives_Error_Check();
//		Wheel_Controls();
//		Node_Id_Check();
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
//		HAL_Delay(1000);
//		Buzzer_Acivated++;
	
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
			CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;	// which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; //14 // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = ENABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	
	CAN_FilterTypeDef canfilterconfig2;

	canfilterconfig2.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig2.FilterBank = 14; //14 // which filter bank to use from the assigned ones
	canfilterconfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig2.FilterIdHigh = 0x000<<5;
	canfilterconfig2.FilterIdLow = 0;
	canfilterconfig2.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig2.FilterMaskIdLow = 0x0000;
	canfilterconfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig2.SlaveStartFilterBank = 14;//14	// how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig2);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Blue_LED_Pin|Buzzer1_Pin|Buzzer2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Custom_Pin_2_Pin|Custom_Pin_3_Pin|Error_LED_Pin|Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Blue_LED_Pin Buzzer1_Pin Buzzer2_Pin */
  GPIO_InitStruct.Pin = Blue_LED_Pin|Buzzer1_Pin|Buzzer2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_State_Pin */
  GPIO_InitStruct.Pin = UART_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Custom_Pin_2_Pin Custom_Pin_3_Pin Error_LED_Pin Green_LED_Pin */
  GPIO_InitStruct.Pin = Custom_Pin_2_Pin|Custom_Pin_3_Pin|Error_LED_Pin|Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UART5_State_Pin */
  GPIO_InitStruct.Pin = UART5_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART5_State_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Reboot (int Axis)
{	
	TxHeader.DLC = 4;	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = ( Axis <<5) | 0x016 ;	
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
	//osDelay(1); 		
}
void Stop_Motors(void)
{
			for(uint8_t i = 1; i <= 3; i++){Set_Motor_Torque(i, 0);}	for(uint8_t i = 7; i <= 8; i++){Set_Motor_Velocity(i, 0);}
}
void Heal_Error(uint8_t Axis_Id)
{
	BUZZER_ON;
	Stop_Motors();
	
	while ( Axis_State[Axis_Id] != 8 )
	{
		Reboot(Axis_Id);	HAL_Delay(3000);
		//Start_Calibration_For ( Axis_Id,  8 , 2 );HAL_Delay(1000);
	} 
	
	DRIVES_ERROR_FLAG = NULL;
	BUZZER_OFF;
}
void Drives_Error_Check(void)
{

	for(uint8_t i = 1; i <= 3; i++)
	{
	//	if ( i != 9 && i !=10 && i !=11 ){ if ( Axis_State[i] != 8 ){ DRIVES_ERROR_FLAG = SET; Heal_Error(i); } HAL_Delay(1); }
		if ( Axis_State[i] != 8 ){ DRIVES_ERROR_FLAG = SET; Heal_Error(i); } HAL_Delay(1); 
		
		//HAL_Delay(1);
	}
}
void Frame_Control_Position_Adjust(void)
{
	if(Joystick==0)
{
	Prev_Left_Vel_Limit = Left_Transmit_Vel; Left_Transmit_Vel=10;
//	CAN_Transmit(1,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1); 
  L_Vert_Error = Left_Roll_Home_Pos - Left_Roll;
	Torque=L_Vert_Error>2?Wheel_Torque:L_Vert_Error<-2?-Wheel_Torque:0;
	if ( Torque_Temp != Torque )
		{
//   	Set_Motor_Torque ( 1 , Torque );
			Torque_Temp = Torque;
		}
}
  Left_Transmit_Vel=Prev_Left_Vel_Limit;
}

void checkNodeIds(void) {
    current_time = HAL_GetTick();
    for (int i = 0; i < 23; i++) {
        if (Node_Id[i] != Prev_Node_Id[i]) {  Prev_Node_Id[i] = Node_Id[i]; Last_Update_Time = current_time; changed = 1; }
    }
    if (!changed && (current_time - Last_Update_Time >= 600)) { BUZZER_ON; Last_Update_Time = current_time; }
}
float KMPHtoRPS(float kmph)
{
    float rps=0; float Circumference = 1335.177;
    rps = (((kmph/(Circumference/1000)*1000)/3600)*Wheel_Reductions);
    return rps;
}

void Joystick_Reception(void)
{
	/*				JOYSTICK VALUES ASSIGNING								*/
	if (( BT_Rx[0] == 0xAA ) &&	( BT_Rx[7] == 0xFF )) 
	{	
		Mode 						 = BT_Rx[1];
		Speed 					 = BT_Rx[2]  != 0 ? BT_Rx[2] : Speed ;
		Steering_Mode 	 = BT_Rx[3];
		Pot_Angle        = BT_Rx[4]; 
		Joystick         = BT_Rx[5];
		Shearing				 = BT_Rx[6];
		
	}

	
		if( Steering_Mode == 0 ) Steering_Mode=1;
	
		
}

void Macro_Controls (void)
{
	if( Mode == 1 )
	{
			if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Macro_Speed =  0; 				break;								
				case 1 :   Macro_Speed =	30;				break; 
				case 2 :   Macro_Speed = -30;				break; 
				default :														break;
			}
			Joystick_Temp = Joystick;
		}
	
	}
	
	else {Macro_Speed = 0 ;}
	
	if ( Macro_Speed_Temp != Macro_Speed )
	{
		Set_Motor_Velocity ( 12, Macro_Speed ) ; 
		Set_Motor_Velocity ( 13, Macro_Speed ) ;
		Macro_Speed_Temp = Macro_Speed;
	}
}

void Wheel_Controls (void)
{
	if (Mode == 2)
	{
		if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Torque = NULL; 							break;								
				case 1 :   Torque =	Wheel_Torque;				break; 
				case 2 :   Torque =-Wheel_Torque;				break; 
				default :																break;
			}
			Joystick_Temp = Joystick;
		}
	}
	else {Torque = 0;}
	
		if ( Torque_Temp != Torque )
		{
			for ( uint8_t i = 1 ; i < 4 ; i++ )
			{ 
				Set_Motor_Torque ( i , Torque );
			}
			Torque_Temp = Torque;
		}
}

void Transmit_Motor_Torque (void)
{
	if ( Mode == 2 )
	{
		if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Torque = NULL; 							break;								
				case 1 :   Torque =	Wheel_Torque;				break; 
				case 2 :   Torque =-Wheel_Torque;				break; 
				default :																break;
			}
			Joystick_Temp = Joystick;
		}
	}
		
		if ( Torque_Temp != Torque )
		{
			if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
			{
				for ( uint8_t i = 1 ; i < 4 ; i++ )
				{
				 Torque = ( i==1 ) ? -Torque : Torque ;   
				 Set_Motor_Torque ( i , Torque );
         HAL_Delay(1);
				}	
			}
			else
			{
			 for ( uint8_t i = 1 ; i < 4 ; i++ )
			 { 
				Set_Motor_Torque ( i , Torque );
        HAL_Delay(1);
			 }
			}
			Torque_Temp = Torque;
		}
}

void New_Drive_Controls(void)
{
	if ( (Speed!= 0) && Left_IMU_State && (Mode != 1) ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
	{

		//if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || Left_Vertical_Error > 10 || Left_Vertical_Error < -10 ){ BUZZER_ON; Error_Handler();}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
	
		//Vel_Limit = Joystick == 0 ? 15 : Speed*12;
		Vel_Limit = Steering_Mode == ALL_WHEEL? Speed * 8 : 8;
		Vel_Limit = Vel_Limit > 36 ? 36 : Vel_Limit;

	
			Transmit_Motor_Torque();

			if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
			Left_Frame_Speed=0;
			Left_Vel_Limit = Vel_Limit  + Left_Steering_Speed + Left_Frame_Speed;//+4;
			Right_Vel_Limit = Vel_Limit + Right_Steering_Speed;//+4;
		
		
		
			if ( Left_Vel_Limit > Prev_Left_Vel_Limit)
			{
				Left_Transmit_Vel = Prev_Left_Vel_Limit + 1;
				CAN_Transmit(1,VEL_LIMIT,Left_Transmit_Vel,4,DATA);
				HAL_Delay(1); 
				Prev_Left_Vel_Limit = Left_Transmit_Vel; //HAL_Delay(5);
			}
			else if ( Left_Vel_Limit < Prev_Left_Vel_Limit)
			{
				Left_Transmit_Vel = Prev_Left_Vel_Limit - 1;
				CAN_Transmit(1,VEL_LIMIT,Left_Transmit_Vel,4,DATA); 
				HAL_Delay(1);
				Prev_Left_Vel_Limit = Left_Transmit_Vel; //HAL_Delay(5);
			}
			else {}
			
			
			
			
			if ( Right_Vel_Limit > Prev_Right_Vel_Limit)
			{
				Right_Transmit_Vel = Prev_Right_Vel_Limit + 1;
				for(uint8_t i=2 ; i < 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1); }
				Prev_Right_Vel_Limit = Right_Transmit_Vel;
			}
			else if ( Right_Vel_Limit < Prev_Right_Vel_Limit)
			{
				Right_Transmit_Vel = Prev_Right_Vel_Limit - 1;
				for(uint8_t i=2 ; i < 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1); }
				Prev_Right_Vel_Limit = Right_Transmit_Vel;
			}
			else {}
			

	}
	else 
	{
		for ( uint8_t i = 1 ; i < 4 ; i++ )
		{ 
				Set_Motor_Torque ( i , 0 );HAL_Delay(1);
		}
	}
	
//	if ( (!BT_State ) && Joystick != 0 )
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
//		Joystick = 0 ;
//	}
}

void Steering_Controls(void)
{
    if ( Steering_Mode == ALL_WHEEL )
    {
        /*Steering Angle Calculation*/
        Right_Steer_Angle = abs(Pot_Angle - 90);
        Right_Steer_Angle = Right_Steer_Angle * 0.38889;
        if (Pot_Angle < 89) Right_Steer_Angle = Right_Steer_Angle / 2.825;
        else if ( Pot_Angle >88 && Pot_Angle < 92 ) Right_Steer_Angle = 0;
        Right_Steer_Angle = round (Right_Steer_Angle * 10) / 10 ;
        /*Steering Angle Calculation*/

        /*Turning Radius Calculation*/
        Right_Turn_Radius = Half_Wheel_Base/(sin(Right_Steer_Angle*Pi/180));
        Chord_Dist = Right_Turn_Radius -  (Half_Wheel_Base/(tan(Right_Steer_Angle*Pi/180)));
        Turning_Radius = Pot_Angle < 89 ? Right_Turn_Radius - Chord_Dist - Half_Wheel_Track : Pot_Angle > 91 ? Right_Turn_Radius - Chord_Dist + Half_Wheel_Track : 0;
        Turning_Radius = Turning_Radius/1000;
        /*Turning Radius Calculation*/

        /*Motor Position Calculation*/
        Right_Motor_Position = (Right_Steer_Angle * Steering_Reductions ) / 360 ;
        Right_Motor_Position = Pot_Angle < 89 ? Right_Motor_Position : -Right_Motor_Position;
        Right_Rear_Steer_Pos = Right_Front_Steer_Pos = Right_Motor_Position;
        Right_Rear_Steer_Pos = -Right_Rear_Steer_Pos;
        /*Motor Position Calculation*/

        /*Steering Speed Calculation*/
        Mean_kmph   = Vel_Limit / 24;
        TimeTaken   = Turning_Radius/(Mean_kmph*0.277);
        Inner_Speed = ((Turning_Radius- 1.368f)/TimeTaken)*3.6;
        Inner_Speed = KMPHtoRPS(Inner_Speed) -  Vel_Limit;
        Outer_Speed = ((Turning_Radius+ 1.368f)/TimeTaken)*3.6;
        Outer_Speed = KMPHtoRPS(Outer_Speed) - Vel_Limit;

        if ( Pot_Angle  < 89 ) // Left Turn of the Rover
        {
            Left_Steering_Speed = Inner_Speed;
            Right_Steering_Speed = Outer_Speed;
        }
        else if ( Pot_Angle > 91 ) // Right Turn of the Rover
        {
            Left_Steering_Speed = Outer_Speed;
            Right_Steering_Speed = Inner_Speed;
        }
        else // Straight Run
        {
            Left_Steering_Speed = Right_Steering_Speed = 0;		
        }
    }
    else if ( Steering_Mode == ZERO_TURN )
    {
        Left_Steering_Speed = Right_Steering_Speed = 0;	
        Right_Motor_Position = 8.23;
        Right_Rear_Steer_Pos = Right_Front_Steer_Pos = Right_Motor_Position;
        Right_Rear_Steer_Pos = -Right_Rear_Steer_Pos;

    }
    else if ( Steering_Mode == WIDTH_SHRINK ) 
		{
			Left_Steering_Speed = Right_Steering_Speed = 0;	
      Right_Motor_Position = -1.05; // -5 deg
      Right_Rear_Steer_Pos = Right_Front_Steer_Pos = Right_Motor_Position;
		}
		else if ( Steering_Mode == WIDTH_EXTEND ) 
		{
			Left_Steering_Speed = Right_Steering_Speed = 0;	
      Right_Motor_Position = 1.05; // +5 deg
      Right_Rear_Steer_Pos = Right_Front_Steer_Pos = Right_Motor_Position;
		}
		else // Do Nothing
		{
			Left_Steering_Speed = Right_Steering_Speed = 0;	
      Right_Motor_Position = 0; // +5 deg
      Right_Rear_Steer_Pos = Right_Front_Steer_Pos = Right_Motor_Position;
		}
		
    if ( Right_Front_Steer_Pos_Temp != Right_Front_Steer_Pos)
    {
        Set_Motor_Position ( 7 , Right_Front_Steer_Pos );HAL_Delay(1);
        Right_Front_Steer_Pos_Temp = Right_Front_Steer_Pos;
    }
    
    if ( Right_Rear_Steer_Pos_Temp != Right_Rear_Steer_Pos)
    {
        Set_Motor_Position ( 8 , Right_Rear_Steer_Pos );HAL_Delay(1);
        Right_Rear_Steer_Pos_Temp = Right_Rear_Steer_Pos;
    }
}

void Frame_Controls_Position_Based(void)
{
	Current_Vert_Pos = Absolute_Position[4];
	
	if ( IMU_Reception_State)
  {

	 R_Vert_Error = Right_Roll_Home_Pos - Right_Roll;

   if (R_Vert_Error < -Vert_Bandwidth || R_Vert_Error > Vert_Bandwidth )
   {
    Right_Vert_Pos = (Current_Vert_Pos + (R_Vert_Error * R_Kp));
    Right_Vert_Pos = (R_Vert_Error/360)*456 ;
   }
		
	}
	
	
	if ( Right_Vert_Pos_Temp != Right_Vert_Pos)
	{
			Set_Motor_Position ( 4 , Right_Vert_Pos );HAL_Delay(1);
			Right_Vert_Pos_Temp = Right_Vert_Pos;
	} 
	
//	Current_Vert_Pos = Right_Vert_Pos;
}

void Frame_Controls_Velocity_Based(void)
{
	//Current_Vert_Pos = Absolute_Position[4];
	
	if ( IMU_Reception_State)
  {
//	 R_Vert_Error = Right_Roll_Home_Pos - Right_Roll;
//   if (R_Vert_Error < -Vert_Bandwidth || R_Vert_Error > Vert_Bandwidth )
//   {
//    Right_Vert_Pos = (R_Vert_Error * R_Kp);
//   // Right_Vert_Pos = (R_Vert_Error/360)*456 ;
//   }
//	 else
//	{
//		Right_Vert_Pos = 0 ;
//	}
	R_Vert_Error = Right_Roll_Home_Pos - Right_Roll;
	Right_Vert_Pos=(R_Vert_Error < -Vert_Bandwidth || R_Vert_Error > Vert_Bandwidth )?(R_Vert_Error * R_Kp):0;
		
	R_Contour_Error = Right_Pitch_Home_Pos - Right_Pitch;
	Right_Contour_Pos=(R_Contour_Error < -Contour_Bandwidth || R_Contour_Error > Contour_Bandwidth )?(R_Contour_Error *(- R_Kp)):0;
		
	L_Contour_Error = Left_Pitch_Home_Pos - Left_Pitch;
	Left_Contour_Pos=(L_Contour_Error < -Contour_Bandwidth || L_Contour_Error > Contour_Bandwidth )?(L_Contour_Error * (-R_Kp)):0;
	}
	
	Right_Vert_Pos = Right_Vert_Pos > 3 ? 3: Right_Vert_Pos < -3 ? -3 : Right_Vert_Pos;
	Right_Contour_Pos = Right_Contour_Pos>3?3:Right_Contour_Pos<-3?-3:Right_Contour_Pos;
	Left_Contour_Pos = Left_Contour_Pos>3?3:Left_Contour_Pos<-3?-3:Left_Contour_Pos;
	if ( Right_Vert_Pos_Temp != Right_Vert_Pos)
	{
		//Set_Motor_Velocity ( 4 , Right_Vert_Pos );HAL_Delay(1);
			Right_Vert_Pos_Temp = Right_Vert_Pos;
	} 
	if ( Right_Contour_Pos_Temp != Right_Contour_Pos)
	{
//		   Right_Contour_Pos=-Right_Contour_Pos;
			Set_Motor_Velocity ( 5 , Right_Contour_Pos );HAL_Delay(1);
			Right_Contour_Pos_Temp = Right_Contour_Pos;
	} 
	if ( Left_Contour_Pos_Temp != Left_Contour_Pos)
	{
//		    Left_Contour_Pos=-Left_Contour_Pos;
			Set_Motor_Velocity ( 6 , Left_Contour_Pos );HAL_Delay(1);
			Left_Contour_Pos_Temp = Left_Contour_Pos;
	} 
//	Current_Vert_Pos = Right_Vert_Pos;
}

void Dynamic_Width_Adjustment (void)
{
	float Width_Speed= 40/2;
	
	if ( Steering_Mode >= 4 )
	{
		if ( Steering_Mode == WIDTH_SHRINK   ) 
		{
			if ( Joystick == 1 )Upper_Width_Motor_Speed =  Width_Speed;
			else if ( Joystick == 2 ) Upper_Width_Motor_Speed  = -Width_Speed;
			else Upper_Width_Motor_Speed = 0;
		}
		
		if ( Steering_Mode == WIDTH_EXTEND  ) 
		{
			if ( Joystick ==1 ) Upper_Width_Motor_Speed = -Width_Speed;
			else if ( Joystick == 2 ) Upper_Width_Motor_Speed = Width_Speed;
			else Upper_Width_Motor_Speed = 0;
		}
		else if ( Steering_Mode < 4 )
		{
			Upper_Width_Motor_Speed = 0;
			//Dynamic_Width_Corrections();
		}

	}
	else 
	{
		Upper_Width_Motor_Speed = 0 ;
	}
	
	if( Steering_Mode < 4 ) Upper_Width_Motor_Speed = 0;
	
	
//	 if (( Upper_Width_Motor_Speed > 0) && ( Upper_Width_Motor_Count >= 335 )) Upper_Width_Motor_Speed = 0;
//	else if (( Upper_Width_Motor_Speed < 0) && ( Upper_Width_Motor_Count <= 20 )) Upper_Width_Motor_Speed = 0;
	
	
	if ( Upper_Width_Motor_Speed != Upper_Width_Motor_Speed_Temp )
	{
		for ( uint8_t i = 0 ; i < 4 ; i++){ Set_Motor_Velocity ( 14, Upper_Width_Motor_Speed);}
		Upper_Width_Motor_Speed_Temp=Upper_Width_Motor_Speed;
	}
}




	

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//	BUZZER_ON;
	for ( uint8_t i =0; i < 5 ; i ++ )
	{
		for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}	
		for(uint8_t i = 5; i <= 20; i++){Set_Motor_Velocity(i, 0);}	
	}
  __disable_irq();
  while (1)
  {
//	BUZZER_ON;	
	for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}	
	for(uint8_t i = 5; i <= 20; i++){Set_Motor_Velocity(i, 0);}	
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
