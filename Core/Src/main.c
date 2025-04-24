/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <TMC2208.h>
#include <stdbool.h>
#include <Util.h>
#include <I2C_LCD_cfg.h>
#include <I2C_LCD.h>

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Physical characteristics of the motor and the screw
#define MOTOR_STEPS_PER_REV		200	// Cantidad de pasos por vuelta del motor
#define SPINDLE_THREAD_PITCH	8	// Paso del husillo (mm)

#define PI	3.14159

#define MyI2C_LCD I2C_LCD_1

#define NUM_PANTALLAS 		4
#define NUM_OPCIONES_MAX 	4
#define DEBOUNCE_DELAY 		150 // Milisegundos
#define MAX_LINEAS 3 // Número máximo de líneas de opciones visibles

#define COL cursor_pos[0]
#define FIL cursor_pos[1]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Physical characteristics of the syringe
float syringe_vol = 0;	// Volumen total de la jeringa (mL)
float syringe_internal_diameter = 0;	// Diametro interno de la jeringa (mm)
float syringe_length;

// Stepper Configuration
int microsteps = 256;	// Cantidad de micropasos por paso
int motor_current = 600;	// Corriente del motor paso a paso (mA)
float Rsense = 0.11;	// Resistencia de sensado del TMC2208 (omhs)

// Stepper control
volatile uint32_t steps_remaining = 0;
volatile bool motor_running = false;
uint32_t steps;

// User variables
float vol_to_dispense = 0;	// Volumen de liquido que se quiere dispensar (mL)
float flow_to_dispense = 0;	// Flujo al que se quiere dispensar el liquido (mL/h)

// Variables display
int col = 0;
int fil = 0;
int cursor_pos[2];

// Estados de la máquina de estados
typedef enum {
	MENU_PRINCIPAL,
	AJUSTANDO_VALOR
} EstadoMenu_t;

typedef enum {
	DISPENSADO,
	CONF_JERINGA,
	CONF_STEPPER,
	PRINCIPAL
} Pantalla_t;

typedef enum {
	VOLUMEN_DISP,
	CAUDAL,
	VOLVER_DISP
} OpcDispensado_t;

typedef enum {
	VOLUMEN_TOTAL,
	LARGO,
	DIAM_INTERNO,
	VOLVER_JER
} OpcConfJeringa_t;

typedef enum {
	MICROPASOS,
	CORRIENTE,
	RSENSE,
	VOLVER_STEP
} OpcConfStepper_t;

// Estructura para cada opción del menú
typedef struct {
	char nombre[16];
	int valor;
} OpcionMenu_t;

// Estructura para cada pantalla
typedef struct {
	char titulo[20];
	OpcionMenu_t opciones[NUM_OPCIONES_MAX];
	int num_opciones;
} PantallaMenu_t;

// Variables globales
EstadoMenu_t estado_actual = MENU_PRINCIPAL;
PantallaMenu_t menu[NUM_PANTALLAS] = {
		{"Dispensado", 			{ {"Vol=", 			0},	{"Caudal=", 		0},							{"Volver",  -1} }, 3 },
		{"Conf Jeringa", 		{ {"Vol total=", 	0}, {"Largo=", 			0}, {"Diametro int=", 	0}, {"Volver",  -1} }, 4 },
		{"Conf stepepr", 		{ {"Micropasos=", 	0}, {"Corriente max=", 	0}, {"Rsense=0.", 		0}, {"Volver",  -1} }, 4 },
		{"Bomba de jeringa", 	{ {"Dispensado",   -1}, {"Conf jeringa",   -1}, {"Conf stepper",   -1}, {"Iniciar", -1} }, 4 },
};

int pantalla_actual = PRINCIPAL;
int opcion_actual = 0;
int desplazamiento = 0;
volatile uint32_t last_interrupt_time = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int vol_To_Steps(float vol);
int flow_To_Time(float vol, float flow);
void TMC2208_Move(int steps, int time);
void TMC2208_Stop(void);

void actualizarLCD();
void manejarBotonArriba();
void manejarBotonAbajo();
void manejarBotonSeleccionar();
void configurarInterrupciones();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	/// MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */



	// LCD I2C configuration
	I2C_LCD_Init(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD, 2, 0);
	I2C_LCD_WriteString(MyI2C_LCD, "Syringe Pump V1.1");
	I2C_LCD_SetCursor(MyI2C_LCD, 6, 1);
	I2C_LCD_WriteString(MyI2C_LCD, "INQUIMAE");

	HAL_Delay(2000);

	actualizarLCD();

	HAL_GPIO_WritePin(STEPPER_UART_GPIO_Port, STEPPER_UART_Pin, GPIO_PIN_RESET);
	HAL_delay(10);
	MX_USART1_UART_Init();
	// TMC2208 configuration
	TMC2208_Init(&huart1, Rsense);
	TMC2208_SetMotorCurrent(motor_current);
	TMC2208_SetMicrostepping(microsteps);
	TMC2208_SetOperationMode(false); 	// Desactivar stealthChop


	menu[CONF_STEPPER].opciones[0].valor = microsteps;
	menu[CONF_STEPPER].opciones[1].valor = motor_current;
	menu[CONF_STEPPER].opciones[2].valor = Rsense * 100;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 7200-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, STEPPER_EN_Pin|STEPPER_DIR_Pin|STEPPER_STEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : STEPPER_EN_Pin STEPPER_DIR_Pin STEPPER_STEP_Pin */
	GPIO_InitStruct.Pin = STEPPER_EN_Pin|STEPPER_DIR_Pin|STEPPER_STEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_DOWN_Pin BTN_SELECT_Pin BTN_UP_Pin */
	GPIO_InitStruct.Pin = BTN_DOWN_Pin|BTN_SELECT_Pin|BTN_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Función para actualizar el LCD
void actualizarLCD() {
	char buffer[20];
	I2C_LCD_Clear(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD, 2, 0);
	I2C_LCD_WriteString(MyI2C_LCD, menu[pantalla_actual].titulo);

	int inicio = desplazamiento;
	int fin = inicio + MAX_LINEAS;
	if(fin > menu[pantalla_actual].num_opciones) {
		fin = menu[pantalla_actual].num_opciones;
	}

	for(int i = inicio; i < fin; i++)
	{
		if(estado_actual == MENU_PRINCIPAL)
		{
			if(menu[pantalla_actual].opciones[i].valor == -1)
			{
				if(i == opcion_actual)
				{
					sprintf(buffer, ">%s", menu[pantalla_actual].opciones[i].nombre);
				} else
				{
					sprintf(buffer, " %s", menu[pantalla_actual].opciones[i].nombre);
				}
			} else
			{
				if(i == opcion_actual) {
					sprintf(buffer, ">%s %d", menu[pantalla_actual].opciones[i].nombre, menu[pantalla_actual].opciones[i].valor);
				} else
				{
					sprintf(buffer, " %s %d", menu[pantalla_actual].opciones[i].nombre, menu[pantalla_actual].opciones[i].valor);
				}
			}
		} else if(estado_actual == AJUSTANDO_VALOR)
		{
			if(menu[pantalla_actual].opciones[i].valor == -1)
			{
				if(i == opcion_actual)
				{
					sprintf(buffer, " %s>", menu[pantalla_actual].opciones[i].nombre);
				} else
				{
					sprintf(buffer, " %s", menu[pantalla_actual].opciones[i].nombre);
				}
			} else
			{
				if(i == opcion_actual)
				{
					sprintf(buffer, " %s>%d", menu[pantalla_actual].opciones[i].nombre, menu[pantalla_actual].opciones[i].valor);
				} else
				{
					sprintf(buffer, " %s %d", menu[pantalla_actual].opciones[i].nombre, menu[pantalla_actual].opciones[i].valor);
				}
			}
		}
		I2C_LCD_SetCursor(MyI2C_LCD, 0, i + 1 - desplazamiento);
		I2C_LCD_WriteString(MyI2C_LCD, buffer);

	}
}

void manejarBotonArriba()
{
	if (estado_actual == MENU_PRINCIPAL)
	{
		if (opcion_actual > 0)
		{
			opcion_actual--;
			if (opcion_actual < desplazamiento)
			{
				desplazamiento--;
			}
		}
	} else if (estado_actual == AJUSTANDO_VALOR)
	{

		if(pantalla_actual == CONF_STEPPER)
		{	// Cambiar micropasos
			if(opcion_actual == MICROPASOS)
			{
				microsteps = microsteps * 2;
				if(microsteps > 256)
					microsteps = 256;
				menu[pantalla_actual].opciones[opcion_actual].valor = microsteps;
			}
		}
		else
			menu[pantalla_actual].opciones[opcion_actual].valor++;
	}
	actualizarLCD();
}

void manejarBotonAbajo()
{
	if (estado_actual == MENU_PRINCIPAL)
	{
		if (opcion_actual < menu[pantalla_actual].num_opciones - 1)
		{
			opcion_actual++;
			if (opcion_actual >= desplazamiento + MAX_LINEAS)
			{
				desplazamiento++;
			}
		}
	} else if (estado_actual == AJUSTANDO_VALOR)
	{

		if(pantalla_actual == CONF_STEPPER && opcion_actual == MICROPASOS)
		{	// Cambiar micropasos
			microsteps = microsteps / 2;
			if(microsteps < 1)
				microsteps = 1;
			menu[pantalla_actual].opciones[opcion_actual].valor = microsteps;
		}
		else
			menu[pantalla_actual].opciones[opcion_actual].valor--;

	}
	actualizarLCD();
}

void manejarBotonSeleccionar()
{
	if (estado_actual == MENU_PRINCIPAL)
	{
		if (pantalla_actual == PRINCIPAL && opcion_actual == DISPENSADO)
		{
			pantalla_actual = DISPENSADO; // Ir a "Conf. Jeringa"
			opcion_actual = 0;
			desplazamiento = 0;
		} else if(pantalla_actual == PRINCIPAL && opcion_actual == CONF_JERINGA)
		{
			pantalla_actual = CONF_JERINGA; // Ir a "dispensado"
			opcion_actual = 0;
			desplazamiento = 0;
		} else if (pantalla_actual == PRINCIPAL && opcion_actual == CONF_STEPPER)
		{
			pantalla_actual = CONF_STEPPER; // Ir a "Conf Stepper."
			opcion_actual = 0;
			desplazamiento = 0;
		} else if ( (pantalla_actual == CONF_JERINGA && opcion_actual == VOLVER_JER) || (pantalla_actual == CONF_STEPPER && opcion_actual == VOLVER_STEP) || (pantalla_actual == DISPENSADO && opcion_actual == VOLVER_DISP))
		{
			pantalla_actual = PRINCIPAL; // Volver al menú principal
			opcion_actual = 0;
			desplazamiento = 0;
		} else
		{
			estado_actual = AJUSTANDO_VALOR;
		}
	} else if (estado_actual == AJUSTANDO_VALOR)
	{
		estado_actual = MENU_PRINCIPAL;
	}
	actualizarLCD();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GetTick() - last_interrupt_time > DEBOUNCE_DELAY)
	{
		switch(GPIO_Pin)
		{
			case BTN_DOWN_Pin:
				manejarBotonAbajo();
				break;
			case BTN_SELECT_Pin:
				manejarBotonSeleccionar();
				break;
			case BTN_UP_Pin:
				manejarBotonArriba();
				break;
		}
		last_interrupt_time = HAL_GetTick();
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3 && motor_running)
	{
		HAL_GPIO_WritePin(STEPPER_STEP_GPIO_Port, STEPPER_STEP_Pin, GPIO_PIN_SET);
		steps_remaining--;
		HAL_GPIO_WritePin(STEPPER_STEP_GPIO_Port, STEPPER_STEP_Pin, GPIO_PIN_RESET);

		if (steps_remaining == 0)
		{
			TMC2208_Stop();
		}
	}
}

/*
 * vol_To_Steps()
 *
 * Descripcion: Calcula los pasos necesarios para dispensar el volumen
 * 				Primero calcula la distancia que debe moverse el carro con la formula de volumen de un cilindo, segun
 * 				el diametro interno de la jeringa configurado previamente. Multiplica por 1000 para convertir de mL a mm3
 * Parametros: vol -> volumen de liquido que se quiere dispensar (mL)
 * return: steps -> pasos que debe hacer el motor para completar el volumen requerido (mm)
 */
int vol_To_Steps(float vol)
{
	float distance_to_move = (4 * vol * 1000) / (PI * syringe_internal_diameter * syringe_internal_diameter);	//Calculo de la distancia que debe recorrer el carro (mm)
	float steps = (distance_to_move * MOTOR_STEPS_PER_REV * microsteps) / SPINDLE_THREAD_PITCH;	// Calculo de pasos necesarios para mover dicha distancia
	return steps;
}

/*
 * flow_To_Time()
 *
 * Descripcion: Calcula el tiempo que va a llevar dispensar el volumen de liquido requerido al flujo seteado.
 * Parametros: 	vol -> volumen de liquido que se quiere dispensar (mL)
 * 				flow -> flujo al que se quiere dispensar el liquido (mL/h)
 * return: time_sec -> tiempo en segundo que va a tardar hasta completar el dispensado
 */
int flow_To_Time(float vol, float flow)
{
	int time = (int)(vol / flow); // Tiempo que tarda en dispensar el liquido segun el volumen y flujo seteados (en horas)
	int time_sec = time * 3600;	// Mismo tiempo en segundos
	return time_sec;
}

/*
 * TMC2208_Move()
 *
 * Descripcion:configura el registro ARR y activa la interrupcion por desbordamiento del timer.
 * Parametros: 	steps -> Pasos de motor calculador
 * 				time -> tiempo hasta completar el dispensado
 *
 * return: void
 */
void TMC2208_Move(int steps, int time)
{
	float t_interrupt = steps / time;
	float ARR = (t_interrupt * 72000000) / TIM3->PSC;
	__HAL_TIM_SET_AUTORELOAD(&htim3, ARR);

	motor_running = true;

	// Iniciar el temporizador en modo interrupción
	HAL_TIM_Base_Start_IT(&htim3);

}

void TMC2208_Stop()
{
	motor_running = false;
	HAL_TIM_Base_Stop_IT(&htim3);
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
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
