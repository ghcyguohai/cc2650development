/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-January-2014
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 

#include "FreeRTOS.h"
#include "hw_config.h"
#include "task.h"
#include "stdint.h"
#include "board.h"
#include "AppTask_Start.h"
#include "stm32f0xx_syscfg.h"
#include "string.h"

/***********************************************************************************
 This firmware  is aimed at distinguish the difference between bluetooth and 2.4G reciver/transmiter.
 of course, The functions  are  different.please upload the firmware to corresponding  borad/module.

************************************************************************************/
/**
  * @brief  Program entry point
  * @param  None
  * @retval None
  */
 
int main(void)
{
  memcpy((void*)0x20000000,(void*)0x8004000,0x100); 
  SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
  AppTask_Start_Creat();
  vTaskStartScheduler();  
} 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
