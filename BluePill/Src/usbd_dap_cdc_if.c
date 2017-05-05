/**
  ******************************************************************************
  * @file           : usbd_dap_cdc_if.c
  * @brief          :
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
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
#include "usbd_dap_cdc_if.h"
#include "DAP.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t DAP_CDC_Init     (void);
static int8_t DAP_CDC_DeInit   (void);
static int8_t DAP_OutEvent     (uint8_t* event_idx);
static int8_t CDC_Control      (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive      (uint8_t* pbuf, uint32_t *Len);

uint8_t SendBuffer[USBD_DAP_OUTREPORT_BUF_SIZE];

static uint8_t DAP_ReportDesc[USBD_DAP_REPORT_DESC_SIZE] =
{
  0x06, 0x00, 0xFF,      /* USAGE_PAGE (Vendor Page: 0xFF00) */
  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0xa1, 0x01,            /* COLLECTION (Application)       */
  /* 7 */

  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */
  0x26, 0xFF, 0x00,      /*     LOGICAL_MAXIMUM (255)      */           
  0x75, 0x08,            /*     REPORT_SIZE (8 bit)        */    
  0x95, 0x40,            /*     REPORT_COUNT (0x40)        */
  /* 16 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0x81, 0x02,            /* INPUT (Array)                  */
  0x95, 0x40,            /*     REPORT_COUNT (0x40)        */
  /* 22 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0x91, 0x02,            /* OUTPUT (Array)                 */
  0x95, 0x01,            /*     REPORT_COUNT (1)           */
  /* 28 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0xB1, 0x02,            /*     FEATURE (Array)            */
  /* 32 */

  0xc0                   /*     END_COLLECTION             */
};

USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint32_t TxReadPtr;

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;

#define DMA_WRITE_PTR ( (APP_TX_DATA_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx)) & (APP_TX_DATA_SIZE - 1) )

static void ComPort_Config(void);

extern void Error_Handler(void);

USBD_DAP_CDC_ItfTypeDef USBD_Interface_fops = 
{
  DAP_ReportDesc,
  DAP_CDC_Init,
  DAP_CDC_DeInit,
  DAP_OutEvent,

  CDC_Control,  
  CDC_Receive
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  DAP_CDC_Init
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DAP_CDC_Init(void)
{ 
  DAP_Setup();

  HAL_TIM_Base_Start_IT(&htim4);

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  return (USBD_OK);
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DAP_CDC_DeInit(void)
{
  HAL_TIM_Base_Stop_IT(&htim4);

  return (USBD_OK);
}

/**
  * @brief  DAP_OutEvent
  *         Manage the DAP(HID) class Out Event    
  * @param  request: received HID report
  */
static int8_t DAP_OutEvent  (uint8_t* request)
{
  uint32_t length = 0;

  memset(SendBuffer, 0, 64);

  length = DAP_ProcessCommand(request, SendBuffer);
  
  USBD_DAP_SendReport(&hUsbDeviceFS, SendBuffer, 64);

  return (0);
}

/**
  * @brief  CDC_Control
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    
    /* Set the new configuration */
    ComPort_Config();

    break;

  case CDC_GET_LINE_CODING:     
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  TIM4 period elapsed callback
  * @retval None
  */
void HAL_TIM4_PeriodElapsedCallback(void)
{
  uint32_t buffsize;
  uint32_t writeptr = DMA_WRITE_PTR;

  if (TxReadPtr != writeptr)
  {
    if (TxReadPtr > writeptr)
    {
      buffsize = APP_TX_DATA_SIZE - TxReadPtr;
    }
    else
    {
      buffsize = writeptr - TxReadPtr;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBufferFS[TxReadPtr], buffsize);

    if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
    {
      TxReadPtr += buffsize;
      if (TxReadPtr == APP_RX_DATA_SIZE)
      {
        TxReadPtr = 0;
      }
    }
  }
}

/**
  * @brief  CDC_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive (uint8_t* Buf, uint32_t *Len)
{
  HAL_UART_Transmit_DMA(&huart2, Buf, *Len);

  return (USBD_OK);
}

/**
  * @brief  USART2 Tx Transfer completed callback
  * @retval None
  */
void HAL_UART2_TxCpltCallback(void)
{
  /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(void)
{
  if(HAL_UART_DeInit(&huart2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* set the Stop bit */
  switch (LineCoding.format)
  {
  case 0:
    huart2.Init.StopBits = UART_STOPBITS_1;
    break;
  case 2:
    huart2.Init.StopBits = UART_STOPBITS_2;
    break;
  default :
    huart2.Init.StopBits = UART_STOPBITS_1;
    break;
  }
  
  /* set the parity bit*/
  switch (LineCoding.paritytype)
  {
  case 0:
    huart2.Init.Parity = UART_PARITY_NONE;
    break;
  case 1:
    huart2.Init.Parity = UART_PARITY_ODD;
    break;
  case 2:
    huart2.Init.Parity = UART_PARITY_EVEN;
    break;
  default :
    huart2.Init.Parity = UART_PARITY_NONE;
    break;
  }
  
  /*set the data type : only 8bits and 9bits is supported */
  switch (LineCoding.datatype)
  {
  case 0x07:
    /* With this configuration a parity (Even or Odd) must be set */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  case 0x08:
    if(huart2.Init.Parity == UART_PARITY_NONE)
    {
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else 
    {
      huart2.Init.WordLength = UART_WORDLENGTH_9B;
    }
    
    break;
  default :
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  }
  
  huart2.Init.BaudRate   = LineCoding.bitrate;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  HAL_UART_Receive_DMA(&huart2, UserTxBufferFS, APP_RX_DATA_SIZE);
  TxReadPtr = 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
