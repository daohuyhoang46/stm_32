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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MPU_ADDR        (0x68 << 1)
#define LPF_ALPHA       0.30f
#define DEADZONE_LOW    0.50f
#define DEADZONE_MID    0.70f

int16_t ax, ay, az, gx, gy, gz;
float   yaw           = 0;
float   gyroZ_offset  = 0;
float   gz_lpf        = 0;
uint32_t last_update  = 0;

#define MOTOR_SPEED     50
#define MOTOR_SPEED_L   50
#define MOTOR_SPEED_R   50
#define MOTOR_KICK      90

bool     isMovingForward = false;
uint32_t kickStartTime   = 0;

#define WALL_DETECT          14
#define WALL_TOO_CLOSE        2
#define FRONT_BLOCK          15
#define LEFT_LOST_THRESHOLD  7

static bool     wasFollowingLeft = false;
static int      leftLostCount    = 0;
static uint32_t stuckCount       = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
float MPU_GetYaw(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}

uint32_t micros() { return __HAL_TIM_GET_COUNTER(&htim1); }
void delayMicroseconds(uint16_t us) {
    uint32_t start = micros();
    while ((micros() - start) < us);
}

long pulseIn(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state, uint32_t timeout) {
    uint32_t start = micros();
    while (HAL_GPIO_ReadPin(port, pin) != state) {
        if (micros() - start > timeout) return 0;
    }
    uint32_t pulse_start = micros();
    while (HAL_GPIO_ReadPin(port, pin) == state) {
        if (micros() - pulse_start > timeout) return 0;
    }
    return micros() - pulse_start;
}

void MPU_Write(uint8_t reg, uint8_t data) { HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, 1, &data, 1, 100); }
void MPU_Read(uint8_t reg, uint8_t* buf, uint8_t len) { HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, 1, buf, len, 100); }

bool MPU_Init() {
    MPU_Write(0x6B, 0x00);
    HAL_Delay(100);
    MPU_Write(0x1B, 0x08);
    uint8_t who;
    MPU_Read(0x75, &who, 1);
    return (who == 0x68);
}

void MPU_ReadRaw() {
    uint8_t data[14];
    MPU_Read(0x3B, data, 14);
    ax = (data[0]  << 8) | data[1];  ay = (data[2]  << 8) | data[3];  az = (data[4]  << 8) | data[5];
    gx = (data[8]  << 8) | data[9];  gy = (data[10] << 8) | data[11]; gz = (data[12] << 8) | data[13];
}

void MPU_Calibrate() {
    long sum = 0;
    for (int i = 0; i < 2000; i++) {
        MPU_ReadRaw(); sum += gz; HAL_Delay(2);
    }
    gyroZ_offset = sum / 2000.0f;
}

void setMotorSpeed(uint8_t percent) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, percent);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, percent);
}

void setMotorSpeedLR(uint8_t left, uint8_t right) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, left);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
}
void setMotorSpeedWithKick(uint8_t percent) {
    setMotorSpeed(MOTOR_KICK);
    HAL_Delay(150);
    setMotorSpeed(percent);
}
void forward() {
    if (!isMovingForward) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9  | GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8  | GPIO_PIN_11, GPIO_PIN_RESET);
        setMotorSpeed(MOTOR_KICK);
        isMovingForward = true;
        kickStartTime = HAL_GetTick();
        printf(" DI THANG");
        return;
    }

    if (HAL_GetTick() - kickStartTime > 150) {
        setMotorSpeedLR(MOTOR_SPEED_L, MOTOR_SPEED_R);
    }
}

void stopMotor() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
    setMotorSpeed(0);
    isMovingForward = false;
}
void backward() {
    if (isMovingForward) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);
        setMotorSpeed(MOTOR_KICK);
        isMovingForward = false;
        kickStartTime = HAL_GetTick();
        printf(" LUI");
        return;
    }

    if (HAL_GetTick() - kickStartTime > 150) {
        setMotorSpeedLR(MOTOR_SPEED_L, MOTOR_SPEED_R);
    }
}

void turnLeft() {
    setMotorSpeed(MOTOR_KICK);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(150);
    setMotorSpeed(MOTOR_SPEED);
    printf(" RE TRAI");
}

void turnRight() {
    setMotorSpeed(MOTOR_KICK);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(150);
    setMotorSpeed(MOTOR_SPEED);
    printf(" RE PHAI");
}
void correctAngle() {
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < 1000) {
        int dL = getDistLeft();
        int dR = getDistRight();

        bool hasLeft  = (dL != -1 && dL < WALL_DETECT);
        bool hasRight = (dR != -1 && dR < WALL_DETECT);

        if (hasLeft && dL < WALL_TOO_CLOSE) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);
            setMotorSpeedLR(MOTOR_SPEED_L + 5, MOTOR_SPEED_R - 5);
            isMovingForward = true;
        } else if (hasRight && dR < WALL_TOO_CLOSE) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);
            setMotorSpeedLR(MOTOR_SPEED_L - 5, MOTOR_SPEED_R + 5);
            isMovingForward = true;
        } else {
            break;
        }
        HAL_Delay(10);
    }

    stopMotor();
}

int getDist(GPIO_TypeDef* tPort, uint16_t tPin, GPIO_TypeDef* ePort, uint16_t ePin) {
    HAL_GPIO_WritePin(tPort, tPin, GPIO_PIN_RESET); delayMicroseconds(3);
    HAL_GPIO_WritePin(tPort, tPin, GPIO_PIN_SET);   delayMicroseconds(10);
    HAL_GPIO_WritePin(tPort, tPin, GPIO_PIN_RESET);
    long t = pulseIn(ePort, ePin, GPIO_PIN_SET, 30000);
    if (t == 0) return -1;
    int d = t * 0.034 / 2;
    return (d >= 2 && d <= 400) ? d : -1;
}
int getDistFront() { return getDist(GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_6);  }
int getDistLeft()  { return getDist(GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_7);  }
int getDistRight() { return getDist(GPIOB, GPIO_PIN_6,  GPIOB, GPIO_PIN_10); }

void getDistAll(int* dF, int* dL, int* dR) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);
    delayMicroseconds(3);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_SET);
    delayMicroseconds(10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);

    uint32_t startF = 0, startL = 0, startR = 0;
    uint32_t endF   = 0, endL   = 0, endR   = 0;
    bool doneF = false, doneL = false, doneR = false;

    uint32_t wait_start = micros();
    while (micros() - wait_start < 5800) {
        if (!startF && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))  startF = micros();
        if (!startL && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  startL = micros();
        if (!startR && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)) startR = micros();
        if (startF && startL && startR) break;
    }

    uint32_t fall_start = micros();
    while (micros() - fall_start < 5800) {
        if (startF && !doneF && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))  { endF = micros(); doneF = true; }
        if (startL && !doneL && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  { endL = micros(); doneL = true; }
        if (startR && !doneR && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)) { endR = micros(); doneR = true; }
        if (doneF && doneL && doneR) break;
    }

    long tF = (startF && doneF) ? (long)(endF - startF) : 0;
    long tL = (startL && doneL) ? (long)(endL - startL) : 0;
    long tR = (startR && doneR) ? (long)(endR - startR) : 0;

    *dF = (tF > 0) ? (int)(tF * 0.034f / 2.0f) : -1;
    *dL = (tL > 0) ? (int)(tL * 0.034f / 2.0f) : -1;
    *dR = (tR > 0) ? (int)(tR * 0.034f / 2.0f) : -1;

    if (*dF < 2 || *dF > 100) *dF = -1;
    if (*dL < 2 || *dL > 100) *dL = -1;
    if (*dR < 2 || *dR > 100) *dR = -1;
}

float MPU_GetYaw() {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_update) / 1000.0f;
    last_update = now;

    if (dt <= 0.001f || dt > 0.3f) return yaw;

    MPU_ReadRaw();
    float gz_raw = (gz - gyroZ_offset) / 65.5f;

    gz_lpf = LPF_ALPHA * gz_raw + (1.0f - LPF_ALPHA) * gz_lpf;

    float gz_out;
    if      (fabs(gz_lpf) < DEADZONE_LOW) gz_out = 0.0f;
    else if (fabs(gz_lpf) < DEADZONE_MID) gz_out = gz_lpf * 0.5f;
    else                                  gz_out = gz_lpf;

    if (gz_out != 0.0f) {
        yaw += gz_out * dt;
        if (yaw >  180) yaw -= 360;
        if (yaw < -180) yaw += 360;
    }

    return yaw;
}

void SnapYawTo90() {
    float snapped;
    float y = yaw;

    if      (y >= -45  && y <  45)  snapped =   0;
    else if (y >=  45  && y < 135)  snapped =  90;
    else if (y >= 135  || y < -135) snapped = 180;
    else                            snapped = -90;

    printf("[SNAP] %.1f → %.1f\r\n", y, snapped);
    yaw = snapped;
}
void turnLeft90() {
    stopMotor();
    HAL_Delay(300);
    gz_lpf = 0.0f;
    last_update = HAL_GetTick();
    HAL_Delay(10);

    for (int i = 0; i < 10; i++) { MPU_GetYaw(); HAL_Delay(5); }

    SnapYawTo90();

    float start_yaw = yaw;
    float target = start_yaw + 85.0f;
    if (target >  180) target -= 360;
    if (target < -180) target += 360;

    last_update = HAL_GetTick();
    turnLeft();
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < 6000) {
        float y = MPU_GetYaw();
        float err = target - y;
        if (err >  180) err -= 360;
        if (err < -180) err += 360;

        printf("[L] Yaw:%.1f Target:%.1f Err:%.1f\r\n", y, target, err);

        if (fabs(err) <= 25.0f) {
            stopMotor();
            uint32_t coast_start = HAL_GetTick();
            while (HAL_GetTick() - coast_start < 400) {
                float cy = MPU_GetYaw();
                float cerr = target - cy;
                if (cerr >  180) cerr -= 360;
                if (cerr < -180) cerr += 360;
                if (cerr < -3.0f) { turnRight(); HAL_Delay(60); stopMotor(); }
                if (fabs(cerr) <= 2.0f) break;
                HAL_Delay(5);
            }
            break;
        }
        HAL_Delay(5);
    }

    stopMotor();
    gz_lpf = 0.0f;
    last_update = HAL_GetTick();
    HAL_Delay(200);

    SnapYawTo90();
    correctAngle();

    for (int i = 0; i < 10; i++) { MPU_GetYaw(); HAL_Delay(5); }
    printf("[L] Final Yaw: %.2f\r\n", yaw);
}

void turnRight90() {
    stopMotor();
    HAL_Delay(300);
    gz_lpf = 0.0f;
    last_update = HAL_GetTick();
    HAL_Delay(10);

    for (int i = 0; i < 10; i++) { MPU_GetYaw(); HAL_Delay(5); }

    SnapYawTo90();

    float start_yaw = yaw;
    float target = start_yaw - 85.0f;
    if (target < -180) target += 360;
    if (target >  180) target -= 360;

    last_update = HAL_GetTick();
    turnRight();
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < 6000) {
        float y = MPU_GetYaw();
        float err = target - y;
        if (err >  180) err -= 360;
        if (err < -180) err += 360;

        printf("[R] Yaw:%.1f Target:%.1f Err:%.1f\r\n", y, target, err);

        if (fabs(err) <= 25.0f) {
            stopMotor();
            uint32_t coast_start = HAL_GetTick();
            while (HAL_GetTick() - coast_start < 400) {
                float cy = MPU_GetYaw();
                float cerr = target - cy;
                if (cerr >  180) cerr -= 360;
                if (cerr < -180) cerr += 360;
                if (cerr > 3.0f) { turnLeft(); HAL_Delay(60); stopMotor(); }
                if (fabs(cerr) <= 2.0f) break;
                HAL_Delay(5);
            }
            break;
        }
        HAL_Delay(5);
    }

    stopMotor();
    gz_lpf = 0.0f;
    last_update = HAL_GetTick();
    HAL_Delay(200);

    SnapYawTo90();
    correctAngle();

    for (int i = 0; i < 10; i++) { MPU_GetYaw(); HAL_Delay(5); }
    printf("[R] Final Yaw: %.2f\r\n", yaw);
}

void robot_setup() {
    stopMotor();
    if (!MPU_Init()) { printf("MPU ERROR!\r\n"); while (1); }
    MPU_Calibrate();
    last_update = HAL_GetTick();
    printf("READY\r\n");
}
void robot_loop() {
    int dF, dL, dR;
    getDistAll(&dF, &dL, &dR);

    bool frontBlocked = (dF != -1 && dF < FRONT_BLOCK);
    bool hasLeft      = (dL != -1 && dL < WALL_DETECT);

    printf("F:%d L:%d R:%d Yaw:%.1f\r\n", dF, dL, dR, MPU_GetYaw());

    if (frontBlocked) {
        stopMotor();
        HAL_Delay(50);

        wasFollowingLeft = false;
        leftLostCount    = 0;

        if (dL == -1 || dL >= WALL_DETECT) {
            printf("BLOCKED -> TURN LEFT\r\n");
            turnLeft90();
        } else if (dR == -1 || dR >= WALL_DETECT) {
            printf("BLOCKED -> TURN RIGHT\r\n");
            turnRight90();
        } else {
            printf("BLOCKED -> U-TURN\r\n");
            turnRight90();
            HAL_Delay(100);
            turnRight90();
        }
        return;
    }

    if (hasLeft) {
        wasFollowingLeft = true;
        leftLostCount    = 0;

        if (dL < WALL_TOO_CLOSE) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);
            setMotorSpeedLR(MOTOR_SPEED_L + 10, MOTOR_SPEED_R - 10);
            isMovingForward = true;
        } else {
            forward();
        }
        return;
    }

    if (wasFollowingLeft) {
        leftLostCount++;

        if (leftLostCount < LEFT_LOST_THRESHOLD) {
            forward();
            return;
        }

        wasFollowingLeft = false;
        leftLostCount    = 0;
        stopMotor();
        HAL_Delay(50);
        printf("LEFT LOST -> TURN LEFT\r\n");
        turnLeft90();
        return;
    }

    forward();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  robot_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    robot_loop();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
