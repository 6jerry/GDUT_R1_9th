/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "driver_usart.h"
#include "hardware.h"
#include "calculation.h"
#include "hardware.h"
#include "FSM.h"
#include "MoveBase.h"
#include "communicate.h"
#include "serial_to_matlab.h"
#include "serial_to_esp32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int test = 0;
int c = 0;
int d = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t g_printf = 1;
extern uint16_t PPM_buf[10];
unsigned char Laser_Flag = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for motor_control */
osThreadId_t motor_controlHandle;
const osThreadAttr_t motor_control_attributes = {
    .name = "motor_control",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for robot_state */
osThreadId_t robot_stateHandle;
const osThreadAttr_t robot_state_attributes = {
    .name = "robot_state",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for robot_move */
osThreadId_t robot_moveHandle;
const osThreadAttr_t robot_move_attributes = {
    .name = "robot_move",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
osThreadId_t uartRXTaskHandle;
const osThreadAttr_t uartRXTask_attributes = {
    .name = "RXTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorControl(void *argument);
void RobotState(void *argument);
void RobotMove(void *argument);
void uartRXTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of motor_control */
    motor_controlHandle = osThreadNew(MotorControl, NULL, &motor_control_attributes);

    /* creation of robot_state */
    robot_stateHandle = osThreadNew(RobotState, NULL, &robot_state_attributes);

    /* creation of robot_move */
    robot_moveHandle = osThreadNew(RobotMove, NULL, &robot_move_attributes);
    // uartRXTaskHandle = osThreadNew(uartRXTask, NULL, &uartRXTask_attributes);
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    float move_time_counter = 0;
    /* Infinite loop */
    for (;;)
    {
        //	  if(SWA<1500)
        //	{
        //		MOTOR_REAL_INFO[0].unitMode = SPEED_CONTROL_MODE;
        //		MOTOR_REAL_INFO[1].unitMode = SPEED_CONTROL_MODE;
        //		MOTOR_REAL_INFO[2].unitMode = SPEED_CONTROL_MODE;
        //		remote_control();
        //	}

        //		a=1;
        //		MOTOR_REAL_INFO[0].unitMode = SPEED_CONTROL_MODE;
        //		MOTOR_REAL_INFO[1].unitMode = SPEED_CONTROL_MODE;
        //		MOTOR_REAL_INFO[2].unitMode = SPEED_CONTROL_MODE;
        //		a=2;
        //		move_time_counter += 0.01f;
        //		PathPlan(move_time_counter,3,7,X0,Y0,Yaw0);
        //		world_to_robot(ROBOT_REAL_POS_DATA);
        //		Kinematic_Analysis(Robot_Chassis.Robot_V[0],Robot_Chassis.Robot_V[1]);
        //
        //		a=3;

        //	 Usart4_SendData(1.0,2.0,3.0,4);
        //	printf("%f,%hd\n",MOTOR_REAL_INFO[2].TARGET_RPM,MOTOR_REAL_INFO[2].RPM);
        //	c=Left_Hand;
        //	d=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14);
        //	Laser_FSM();
        // Activate_FSM();
        vTaskDelay(10);
    }

    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorControl */
/**
 * @brief Function implementing the motor_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorControl */
float test_msgss[8] = {0};
void MotorControl(void *argument)
{
    /* USER CODE BEGIN MotorControl */
    /* Infinite loop */
    for (;;)
    {
        //	  moving_point_track(1.0f, 1.0f, 0.0f,1.0f);
        // MOTOR_REAL_INFO[0].TARGET_RPM = 200.0f;
        // MOTOR_REAL_INFO[1].TARGET_CURRENT = rcurrent_to_vcurrent(500);
        // MOTOR_REAL_INFO[2].TARGET_RPM = 0.0f;
        // remote_FSM();
        // MOVE_FSM();
        // Photogate_FSM();

        test_msgss[0] = ROBOT_REAL_POS_DATA.POS_YAW_RAD;
        test_msgss[1] = heading_lock.setpoint;
        test_msgss[2] = SHOOT_MOTOR_INFO[2].RPM;
        test_msgss[3] = SHOOT_MOTOR_INFO[3].RPM;
        test_msgss[4] = SHOOT_MOTOR_INFO[0].real_current;
        // send_serial_frame_mat(&huart2, 0x01, 8, test_msgss);
        vTaskDelay(10);
    }
    /* USER CODE END MotorControl */
}

/* USER CODE BEGIN Header_RobotState */
/**
 * @brief Function implementing the robot_state thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RobotState */
void RobotState(void *argument)
{
    /* USER CODE BEGIN RobotState */
    /* Infinite loop */
    for (;;)
    {
        test++;
        xbox_remote_control();

        //	  YawAdjust(45);
        //  	printf("x: %f y: %f \r\n",ROBOT_REAL_POS_INFO.Position[x],ROBOT_REAL_POS_INFO.Position[y]);
        // remote_control();
        // Robot_Chassis.Robot_V[1] = 0.50f;
        robot_tf();
        //	  move_test();
        //	  VelCrl(&MOTOR_REAL_INFO[1],200);
        //	  remote_FSM();
        //	  if(Laser_calibration(-2.63f, -3.06f,0,0.8f,1))
        //		{
        ////			Usart4_SendData(0,0,0,Inspection);
        //			Laser_Flag=1;
        //		}
        // Retry_FSM();
        // move_Laser();
        robot_speed_control();
        // MoveCtrl();
        //	ZONE2_FSM();
        vTaskDelay(5);
    }
    /* USER CODE END RobotState */
}

/* USER CODE BEGIN Header_RobotMove */
/**
 * @brief Function implementing the robot_move thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RobotMove */
void RobotMove(void *argument)
{
    /* USER CODE BEGIN RobotMove */
    //	static portTickType move_xLastWakeTime;
    //	const portTickType move_xFrequency = pdMS_TO_TICKS(10); // ��?10ms
    /* Infinite loop */
    for (;;)
    {
        if (ppm_update_flag == 1)
        {
            // remote_control();
        }
        // shoot_control();

        if (ROCK_L_X < 800) // ??500????????ppm?0????????????
        {
            ROCK_L_X = 1500;
            ROCK_L_Y = 1500;
            ROCK_R_X = 1500;
            ROCK_R_Y = 1500;
        }
        MotorCtrl();
        vTaskDelay(1);
        // vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // ������?
    }
    /* USER CODE END RobotMove */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
/*void uartRXTask(void *argument)
{
    UART_Message msg;
    for (;;)
    {
        // ????????
        if (xQueueReceive(uartQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            // ?? uart_id ????????
            switch (msg.uart_id)
            {
            case UART_ID_USART1:
                uint8_t esp32_id = handle_serial_data_esp32(msg.data);
                if (esp32_id == 0x01)
                {
                }
                // ?? UART4 ???

                break;
            case UART_ID_USART2:
                break;
            case UART_ID_USART3:
                // ?? USART3 ???
                break;
            case UART_ID_UART4:

                break;
            case UART_ID_UART5:
                // ?? UART5 ???
                break;
            case UART_ID_UART7:
                // ?? UART7 ???
                break;
            case UART_ID_UART8:
                // ?? UART8 ???
                break;
            default:
                // ?? UART
                break;
            }
        }
    }
}*/
