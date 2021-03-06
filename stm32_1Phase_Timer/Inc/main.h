/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SW_bypass_Pin GPIO_PIN_0
#define SW_bypass_GPIO_Port GPIOF
#define SW_auto_man_Pin GPIO_PIN_1
#define SW_auto_man_GPIO_Port GPIOF
#define Low_level_Pin GPIO_PIN_6
#define Low_level_GPIO_Port GPIOA
#define High_level_Pin GPIO_PIN_7
#define High_level_GPIO_Port GPIOA
#define RELAY_1_Pin GPIO_PIN_0
#define RELAY_1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_1
#define RELAY_2_GPIO_Port GPIOB
#define RELAY_3_Pin GPIO_PIN_2
#define RELAY_3_GPIO_Port GPIOB
#define Manual_led_Pin GPIO_PIN_10
#define Manual_led_GPIO_Port GPIOB
#define Auto_led_Pin GPIO_PIN_11
#define Auto_led_GPIO_Port GPIOB
#define DB_Pin GPIO_PIN_12
#define DB_GPIO_Port GPIOB
#define G_Pin GPIO_PIN_13
#define G_GPIO_Port GPIOB
#define F_Pin GPIO_PIN_14
#define F_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_15
#define E_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define D_Pin GPIO_PIN_12
#define D_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_6
#define C_GPIO_Port GPIOF
#define B_Pin GPIO_PIN_7
#define B_GPIO_Port GPIOF
#define A_Pin GPIO_PIN_15
#define A_GPIO_Port GPIOA
#define SW_down_Pin GPIO_PIN_3
#define SW_down_GPIO_Port GPIOB
#define SW_up_Pin GPIO_PIN_4
#define SW_up_GPIO_Port GPIOB
#define SW_menu_Pin GPIO_PIN_5
#define SW_menu_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOB
#define Power_loss_Pin GPIO_PIN_9
#define Power_loss_GPIO_Port GPIOB
#define Power_loss_EXTI_IRQn EXTI4_15_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
