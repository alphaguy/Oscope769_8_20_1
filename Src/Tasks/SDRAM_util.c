/*
 * SDRAM_util.c
 *
 *  Created on: May 10, 2016
 *      Author: Aaron
 */

#include "oscope.h"


extern SDRAM_HandleTypeDef hsdram1;


void SDRAM_Init(void){
	  SDRAM_Initialization_sequence(&hsdram1, REFRESH_COUNT);
	  __HAL_RCC_CRC_CLK_ENABLE(); /* Enable the CRC Module */
}



/**
  * @brief  Reads an amount of data from the SDRAM memory in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval SDRAM status
  */
  
  /*
uint8_t BSP_SDRAM_ReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Read_32b(&hsdram1, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


*/

/**
  * @brief  Writes an amount of data to the SDRAM memory in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval SDRAM status
  */
  
  /*
uint8_t BSP_SDRAM_WriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Write_32b(&hsdram1, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

*/

/**
  * @brief  Programs the SDRAM device.
  * @param  RefreshCount: SDRAM refresh counter value
  * @retval None
  */
void SDRAM_Initialization_sequence(SDRAM_HandleTypeDef *sdramHandle, uint32_t RefreshCount)
{
  __IO uint32_t tmpmrd = 0;
  static FMC_SDRAM_CommandTypeDef Command;

  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_2           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(sdramHandle, RefreshCount);
}



void SDRAM_Test(void){
#define SDRAM_Base ((uint32_t *)(0xC0000000U))
#define EndOfMemory 1024*1024*2
	uint32_t * RAMptr =SDRAM_Base;
	uint32_t tempword[4];
	int i,j,k;

	printf("Starting SDRAM Test...\r\n");

	for(i=0;i<EndOfMemory;i++){
		RAMptr[i] = 0; // clear all memory to start
	}

	for(i=0; i<2048;i++){
		for(j=0; j<1024;j++){
			tempword[0] = RAMptr[i*j];
			RAMptr[(i*1024+j)] = 0xAAAA;
			if(RAMptr[(i*1024+j)] != 0xAAAA){
				printf("Memory readback error for 0xAAAA.  i=%ud, j=%ud\r\n",i,j);
				RAMptr[(i*1024+j)] = tempword[0]; // restore value in case still alive
				return;
			}
/*
			for(k=0;k<EndOfMemory;k++){
				if(RAMptr[k] != 0){
					if(k != ((i*1024+j))){
						printf("Mirror write error. i=%d, j=%d, k=%d, data = %ud",
								i,j,k,RAMptr[k]);
						return;
					}
				}
			}
*/
			RAMptr[(i*1024+j)] = 0x5555;
			if(RAMptr[(i*1024+j)] != 0x5555){
				printf("Memory readback error for 0x5555.  i=%ud, j=%ud\r\n",i,j);
				RAMptr[(i*1024+j)] = tempword[0]; // restore value in case still alive
				return;
			}
			for(k=(i*1024+j);k<EndOfMemory;k+=1024){
				if(RAMptr[k] != 0){
					if(k != (i*1024+j)){
						printf("Mirror write error. i=%d, j=%d, k=%d, data = %ud",
								i,j,k,RAMptr[k]);
						return;
					}
				}
			}

			RAMptr[(i*1024+j)] = tempword[0]; //test done for that word, restore original value
		}
		if(i%128 == 0){
			printf("%dkWords = %d kBytes checked\r\n",i,(i*4));
		}
	}
	printf("Test complete, %d kBytes checked with 0xAAAA and 0x5555 patterns\r\n",(i*4));



}

