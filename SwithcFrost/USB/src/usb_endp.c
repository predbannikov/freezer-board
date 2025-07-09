/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "HidDev_Config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Receive_Buffer[ENDP_OUT_SIZE+1];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#if ((ENDP_OUT_SIZE)>0)
__attribute__((weak))
void HID_Receive(uint8_t *Buff, uint8_t Size)
{

}
#endif

void HID_SendBuff(uint8_t *Buff, uint8_t Size)
{
    if ((Buff != 0) && (Size > 0) && (Size<=(ENDP_IN_SIZE)))
    {
        USB_SIL_Write(EP1_IN, Buff, Size);
        SetEPTxValid(ENDP1);
    }
}


/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{

  /* Read received data */
  USB_SIL_Read(EP1_OUT, Receive_Buffer);

#if ((ENDP_OUT_SIZE)>0)
  HID_Receive(Receive_Buffer, ENDP_OUT_SIZE);
#endif

  SetEPRxStatus(ENDP1, EP_RX_VALID);

}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

