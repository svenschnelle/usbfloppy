/**
  ******************************************************************************
  * @file    memory.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "memory.h"
#include "usb_scsi.h"
#include "usb_bot.h"
#include "usb_regs.h"
#include "usb_mem.h"
#include "usb_conf.h"
#include "hw_config.h"
#include "mass_mal.h"
#include "usb_lib.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t Block_Read_count = 0;
__IO uint32_t Block_offset;
__IO uint32_t Counter = 0;
uint32_t  Idx;
uint32_t Data_Buffer[BULK_MAX_PACKET_SIZE *8]; /* 2048 bytes*/
uint8_t TransferState = TXFR_IDLE;
/* Extern variables ----------------------------------------------------------*/
extern uint8_t Bulk_Data_Buff[BULK_MAX_PACKET_SIZE];  /* data buffer*/
extern uint16_t Data_Len;
extern uint8_t Bot_State;
extern Bulk_Only_CBW CBW;
extern Bulk_Only_CSW CSW;
extern uint32_t Mass_Memory_Size[2];
extern uint32_t Mass_Block_Size[2];

/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int Read_Memory(uint8_t lun, uint32_t Memory_Offset, uint32_t Transfer_Length)
{
	static uint32_t Offset, Length;
	int ret;

	if (TransferState == TXFR_IDLE ) {
		Offset = Memory_Offset * Mass_Block_Size[lun];
		Length = Transfer_Length * Mass_Block_Size[lun];
		TransferState = TXFR_ONGOING;
	}

	if (TransferState == TXFR_ONGOING) {
		if (!Block_Read_count) {
			ret = MAL_Read(lun ,
				       Offset ,
				       Data_Buffer,
				       Mass_Block_Size[lun]);
			if (ret) {
				Block_Read_count = 0;
				Block_offset = 0;
				Offset = 0;
				Bot_State = BOT_DATA_IN_LAST;
				TransferState = TXFR_IDLE;
				return ret;
			}

			USB_SIL_Write(MSC_IN_EP, (uint8_t *)Data_Buffer, BULK_MAX_PACKET_SIZE);

			Block_Read_count = Mass_Block_Size[lun] - BULK_MAX_PACKET_SIZE;
			Block_offset = BULK_MAX_PACKET_SIZE;
		} else {
			USB_SIL_Write(MSC_IN_EP, (uint8_t *)Data_Buffer + Block_offset, BULK_MAX_PACKET_SIZE);

			Block_Read_count -= BULK_MAX_PACKET_SIZE;
			Block_offset += BULK_MAX_PACKET_SIZE;
		}

		SetEPTxCount(MSC_EP_IDX, BULK_MAX_PACKET_SIZE);
		SetEPTxStatus(MSC_EP_IDX, EP_TX_VALID);
		Offset += BULK_MAX_PACKET_SIZE;
		Length -= BULK_MAX_PACKET_SIZE;

		CSW.dDataResidue -= BULK_MAX_PACKET_SIZE;
		//Led_RW_ON();
	}

//	printf("%s: Length remaining %x\n", __func__, (int)Length);
	if (Length == 0) {
		Block_Read_count = 0;
		Block_offset = 0;
		Offset = 0;
		Bot_State = BOT_DATA_IN_LAST;
		TransferState = TXFR_IDLE;
		//Led_RW_OFF();
	}
	return 0;
}

/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int Write_Memory (uint8_t lun, uint32_t Memory_Offset, uint32_t Transfer_Length)
{
	static uint32_t W_Offset, W_Length;
	uint32_t temp =  Counter + 64;
	int ret;

	if (TransferState == TXFR_IDLE ) {
		W_Offset = Memory_Offset * Mass_Block_Size[lun];
		W_Length = Transfer_Length * Mass_Block_Size[lun];
		MAL_StartWrite(lun, Memory_Offset, Transfer_Length);
		TransferState = TXFR_ONGOING;
	}

	if (TransferState == TXFR_ONGOING ) {
		for (Idx = 0 ; Counter < temp; Counter++)
			*((uint8_t *)Data_Buffer + Counter) = Bulk_Data_Buff[Idx++];

		W_Offset += Data_Len;
		W_Length -= Data_Len;

		if (!(W_Length % Mass_Block_Size[lun])) {
			Counter = 0;
			ret = MAL_Write(lun ,
					W_Offset - Mass_Block_Size[lun],
					Data_Buffer,
					Mass_Block_Size[lun], W_Length == 0 ? 1 : 0);
			if (ret) {
				Counter = 0;
				TransferState = TXFR_IDLE;
				Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
				return ret;
			}
		}

		CSW.dDataResidue -= Data_Len;
		SetEPRxStatus(MSC_EP_IDX, EP_RX_VALID); /* enable the next transaction*/
		//Led_RW_ON();
	}

	if ((W_Length == 0) || (Bot_State == BOT_CSW_Send)) {
		Counter = 0;
		Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
		TransferState = TXFR_IDLE;
		//Led_RW_OFF();
	}
	return 0;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
