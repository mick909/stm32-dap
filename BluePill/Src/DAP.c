/**
 * @file    DAP.c
 * @brief   Implementation of DAP.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main.h"
#include "string.h"
#include "DAP_config.h"
#include "DAP.h"

#define DAP_FW_VER      "1.0"   // Firmware Version

DAP_Data_t DAP_Data;            // DAP Data
__IO uint8_t DAP_TransferAbort; // Transfer Abort Flag

const char DAP_FW_Ver [] = DAP_FW_VER;

static void PORT_SWD_SETUP(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIOA->BSRR = SWCLK_Pin | SWDIO_Pin;
  GPIOA->BRR  = nRESET_Pin;

  GPIO_InitStruct.Pin = SWDIO_Pin|SWCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  PIN_nRESET_HIGH();
}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
static void PORT_OFF(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : SWDIO_Pin SWCLK_Pin nRESET_Pin */
  GPIO_InitStruct.Pin = SWDIO_Pin|SWCLK_Pin|nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Get DAP Information
//   id:      info identifier
//   info:    pointer to info data
//   return:  number of bytes in info data
static uint8_t DAP_Info(uint8_t id, uint8_t* info)
{
  uint8_t length = 0;

  switch (id) {
    case DAP_ID_VENDOR:
      break;
    case DAP_ID_PRODUCT:
      break;
    case DAP_ID_SER_NUM:
      break;
    case DAP_ID_FW_VER:
      memcpy(info, DAP_FW_Ver, sizeof(DAP_FW_Ver)+1);
      length = sizeof(DAP_FW_Ver)+1;
      break;
    case DAP_ID_DEVICE_VENDOR:
      break;
    case DAP_ID_DEVICE_NAME:
      break;
    case DAP_ID_CAPABILITIES:
      info[0] = 1;  /* SWD */
      length = 1;
      break;
    case DAP_ID_PACKET_SIZE:
      info[0] = (uint8_t)(DAP_PACKET_SIZE >> 0);
      info[1] = (uint8_t)(DAP_PACKET_SIZE >> 8);
      length = 2;
      break;
    case DAP_ID_PACKET_COUNT:
      info[0] = DAP_PACKET_COUNT;
      length = 1;
      break;
    default:
      break;
  }

  return (length);
}

extern TIM_HandleTypeDef htim2;

// Start Timer
static inline __attribute__((always_inline)) void TIMER_START (uint32_t usec) {
  SysTick->VAL  = 0;
  SysTick->LOAD = usec * CPU_CLOCK / 1000000;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

// Stop Timer
static inline __attribute__((always_inline)) void TIMER_STOP (void) {
  SysTick->CTRL = 0;
}

// Check if Timer expired
static inline __attribute__((always_inline)) uint32_t TIMER_EXPIRED (void) {
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0);
}

// Process Host Status command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_HostStatus(uint8_t *request, uint8_t *response)
{
  switch (*request) {
    case DAP_DEBUGGER_CONNECTED:
      if (*(request+1) & 1) {
        GPIOA->BSRR = DAP_LED_1_Pin;
      } else {
        GPIOA->BRR = DAP_LED_1_Pin;
      }
      break;
    case DAP_TARGET_RUNNING:
      if (*(request+1) & 1) {
        GPIOA->BSRR = DAP_LED_2_Pin;
      } else {
        GPIOA->BRR = DAP_LED_2_Pin;
      }
      break;
    default:
      *response = DAP_ERROR;
      return (1);
  }

  *response = DAP_OK;
  return (1);
}

// Process Connect command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_Connect(uint8_t *request, uint8_t *response)
{
  uint32_t port;

  if (*request == DAP_PORT_AUTODETECT) {
    port = 1;   /* SWD */
  } else {
    port = *request;
  }

  switch (port) {
    case DAP_PORT_SWD:
      DAP_Data.debug_port = DAP_PORT_SWD;
      PORT_SWD_SETUP();
      GPIOC->BRR = LED1_Pin;
      break;

    default:
      *response = DAP_PORT_DISABLED;
      return (1);
  }

  *response = port;
  return (1);
}

// Process Disconnect command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_Disconnect(uint8_t *response)
{
  DAP_Data.debug_port = DAP_PORT_DISABLED;

  PORT_OFF();

  GPIOC->BSRR = LED1_Pin;

  *response = DAP_OK;
  return (1);
}

// Process SWJ Pins command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWJ_Pins(uint8_t *request, uint8_t *response)
{
  uint32_t value;
  uint32_t select;
  uint32_t wait;

  value  =  *(request+0);
  select =  *(request+1);
  wait   = (*(request+2) <<  0) |
           (*(request+3) <<  8) |
           (*(request+4) << 16) |
           (*(request+5) << 24);

  if (select & (1 << DAP_SWJ_SWCLK_TCK)) {
    if (value & (1 << DAP_SWJ_SWCLK_TCK)) {
      PIN_SWCLK_TCK_SET();
    } else {
      PIN_SWCLK_TCK_CLR();
    }
  }
  if (select & (1 << DAP_SWJ_SWDIO_TMS)) {
    if (value & (1 << DAP_SWJ_SWDIO_TMS)) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
  }
  if (select & (1 << DAP_SWJ_nRESET)) {
    PIN_nRESET_OUT(value >> DAP_SWJ_nRESET);
  }

  if (wait) {
    if (wait > 3000000) wait = 3000000;
    TIMER_START(wait);
    do {
      if (select & (1 << DAP_SWJ_SWCLK_TCK)) {
        if ((value >> DAP_SWJ_SWCLK_TCK) ^ PIN_SWCLK_TCK_IN()) continue;
      }
      if (select & (1 << DAP_SWJ_SWDIO_TMS)) {
        if ((value >> DAP_SWJ_SWDIO_TMS) ^ PIN_SWDIO_TMS_IN()) continue;
      }
      if (select & (1 << DAP_SWJ_nRESET)) {
        if ((value >> DAP_SWJ_nRESET) ^ PIN_nRESET_IN()) continue;
      }
      break;
    } while (!TIMER_EXPIRED());
    TIMER_STOP();
  }

  value = (PIN_SWCLK_TCK_IN() << DAP_SWJ_SWCLK_TCK) |
          (PIN_SWDIO_TMS_IN() << DAP_SWJ_SWDIO_TMS) |
          (PIN_nRESET_IN()    << DAP_SWJ_nRESET);

  *response = (uint8_t)value;
  return (1);
}

// Process SWJ Clock command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWJ_Clock(uint8_t *request, uint8_t *response)
{
  uint32_t clock;
  uint32_t delay;

  clock = (*(request+0) <<  0) |
          (*(request+1) <<  8) |
          (*(request+2) << 16) |
          (*(request+3) << 24);

  if (clock == 0) {
    *response = DAP_ERROR;
    return (1);
  }

  delay = (CPU_CLOCK/2 + (clock - 1)) / clock;
  if (delay > IO_PORT_WRITE_CYCLES) {
    delay -= IO_PORT_WRITE_CYCLES;
    delay  = (delay + (DELAY_SLOW_CYCLES - 1)) / DELAY_SLOW_CYCLES;
  }

  DAP_Data.clock_delay = delay;

  *response = DAP_OK;
  return (1);
}

// Process SWJ Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWJ_Sequence(uint8_t *request, uint8_t *response)
{
  uint32_t count;

  count = *request++;
  if (count == 0) count = 256;

  SWJ_Sequence(count, request);

  *response = DAP_OK;
  return (1);
}

// Process SWD Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_Configure(uint8_t *request, uint8_t *response)
{
  uint8_t value;

  value = *request;
  DAP_Data.swd_conf.turnaround  = (value & 0x03) + 1;
  DAP_Data.swd_conf.data_phase  = (value & 0x04) ? 1 : 0;

  *response = DAP_OK;

  return (1);
}

// Process SWD Abort command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_Abort(uint8_t *request, uint8_t *response)
{
  uint32_t data;

  if (DAP_Data.debug_port != DAP_PORT_SWD) {
    *response = DAP_ERROR;
    return (1);
  }

  // Load data (Ignore DAP index)
  data = (*(request+1) <<  0) |
         (*(request+2) <<  8) |
         (*(request+3) << 16) |
         (*(request+4) << 24);

  // Write Abort register
  SWD_Transfer(DP_ABORT, &data);
  *response = DAP_OK;

  return (1);
}

// Process Transfer Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_TransferConfigure(uint8_t *request, uint8_t *response)
{
  DAP_Data.transfer.idle_cycles = *(request+0);
  DAP_Data.transfer.retry_count = *(request+1) | (*(request+2) << 8);
  DAP_Data.transfer.match_retry = *(request+3) | (*(request+4) << 8);

  *response = DAP_OK;

  return (1);
}

// Process SWD Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_Transfer(uint8_t *request, uint8_t *response)
{
  uint32_t  request_count;
  uint32_t  request_value;
  uint32_t  response_count;
  uint32_t  response_value;
  uint8_t  *response_head;
  uint32_t  post_read;
  uint32_t  check_write;
  uint32_t  match_value;
  uint32_t  match_retry;
  uint32_t  retry;
  uint32_t  data;

  response_count = 0;
  response_value = 0;
  response_head  = response;
  response      += 2;

  DAP_TransferAbort = 0;

  post_read   = 0;
  check_write = 0;

  request++;            // Ignore DAP index

  request_count = *request++;
  while (request_count--) {
    request_value = *request++;
    if (request_value & DAP_TRANSFER_RnW) {
      // Read register
      if (post_read) {
        // Read was posted before
        retry = DAP_Data.transfer.retry_count;
        if ((request_value & (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) == DAP_TRANSFER_APnDP) {
          // Read previous AP data and post next AP read
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        } else {
          // Read previous AP data
          do {
            response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          post_read = 0;
        }
        if (response_value != DAP_TRANSFER_OK) break;
        // Store previous AP data
        *response++ = (uint8_t) data;
        *response++ = (uint8_t)(data >>  8);
        *response++ = (uint8_t)(data >> 16);
        *response++ = (uint8_t)(data >> 24);
      }
      if (request_value & DAP_TRANSFER_MATCH_VALUE) {
        // Read with value match
        match_value = (*(request+0) <<  0) |
                      (*(request+1) <<  8) |
                      (*(request+2) << 16) |
                      (*(request+3) << 24);
        request += 4;
        match_retry = DAP_Data.transfer.match_retry;
        if (request_value & DAP_TRANSFER_APnDP) {
          // Post AP read
          retry = DAP_Data.transfer.retry_count;
          do {
            response_value = SWD_Transfer(request_value, NULL);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) break;
        }
        do {
          // Read register until its value matches or retry counter expires
          retry = DAP_Data.transfer.retry_count;
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) break;
        } while (((data & DAP_Data.transfer.match_mask) != match_value) && match_retry-- && !DAP_TransferAbort);
        if ((data & DAP_Data.transfer.match_mask) != match_value) {
          response_value |= DAP_TRANSFER_MISMATCH;
        }
        if (response_value != DAP_TRANSFER_OK) break;
      } else {
        // Normal read
        retry = DAP_Data.transfer.retry_count;
        if (request_value & DAP_TRANSFER_APnDP) {
          // Read AP register
          if (post_read == 0) {
            // Post AP read
            do {
              response_value = SWD_Transfer(request_value, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
            if (response_value != DAP_TRANSFER_OK) break;
            post_read = 1;
          }
        } else {
          // Read DP register
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) break;
          // Store data
          *response++ = (uint8_t) data;
          *response++ = (uint8_t)(data >>  8);
          *response++ = (uint8_t)(data >> 16);
          *response++ = (uint8_t)(data >> 24);
        }
      }
      check_write = 0;
    } else {
      // Write register
      if (post_read) {
        // Read previous data
        retry = DAP_Data.transfer.retry_count;
        do {
          response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        if (response_value != DAP_TRANSFER_OK) break;
        // Store previous data
        *response++ = (uint8_t) data;
        *response++ = (uint8_t)(data >>  8);
        *response++ = (uint8_t)(data >> 16);
        *response++ = (uint8_t)(data >> 24);
        post_read = 0;
      }
      // Load data
      data = (*(request+0) <<  0) |
             (*(request+1) <<  8) |
             (*(request+2) << 16) |
             (*(request+3) << 24);
      request += 4;
      if (request_value & DAP_TRANSFER_MATCH_MASK) {
        // Write match mask
        DAP_Data.transfer.match_mask = data;
        response_value = DAP_TRANSFER_OK;
      } else {
        // Write DP/AP register
        retry = DAP_Data.transfer.retry_count;
        do {
          response_value = SWD_Transfer(request_value, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        if (response_value != DAP_TRANSFER_OK) break;
        check_write = 1;
      }
    }
    response_count++;
    if (DAP_TransferAbort) break;
  }

  if (response_value == DAP_TRANSFER_OK) {
    if (post_read) {
      // Read previous data
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      // Store previous data
      *response++ = (uint8_t) data;
      *response++ = (uint8_t)(data >>  8);
      *response++ = (uint8_t)(data >> 16);
      *response++ = (uint8_t)(data >> 24);
    } else if (check_write) {
      // Check last write
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
    }
  }

end:
  *(response_head+0) = (uint8_t)response_count;
  *(response_head+1) = (uint8_t)response_value;

  return (response - response_head);
}

// Process SWD Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_TransferBlock(uint8_t *request, uint8_t *response)
{
  uint32_t  request_count;
  uint32_t  request_value;
  uint32_t  response_count;
  uint32_t  response_value;
  uint8_t  *response_head;
  uint32_t  retry;
  uint32_t  data;

  response_count = 0;
  response_value = 0;
  response_head  = response;
  response      += 3;

  DAP_TransferAbort = 0;

  request++;            // Ignore DAP index

  request_count = *request | (*(request+1) << 8);
  request += 2;
  if (request_count == 0) goto end;

  request_value = *request++;
  if (request_value & DAP_TRANSFER_RnW) {
    // Read register block
    if (request_value & DAP_TRANSFER_APnDP) {
      // Post AP read
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
    }
    while (request_count--) {
      // Read DP/AP register
      if ((request_count == 0) && (request_value & DAP_TRANSFER_APnDP)) {
        // Last AP read
        request_value = DP_RDBUFF | DAP_TRANSFER_RnW;
      }
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      // Store data
      *response++ = (uint8_t) data;
      *response++ = (uint8_t)(data >>  8);
      *response++ = (uint8_t)(data >> 16);
      *response++ = (uint8_t)(data >> 24);
      response_count++;
    }
  } else {
    // Write register block
    while (request_count--) {
      // Load data
      data = (*(request+0) <<  0) |
             (*(request+1) <<  8) |
             (*(request+2) << 16) |
             (*(request+3) << 24);
      request += 4;
      // Write DP/AP register
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      response_count++;
    }
    // Check last write
    retry = DAP_Data.transfer.retry_count;
    do {
      response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
  }

end:
  *(response_head+0) = (uint8_t)(response_count >> 0);
  *(response_head+1) = (uint8_t)(response_count >> 8);
  *(response_head+2) = (uint8_t) response_value;

  return (response - response_head);
}

// Process DAP Vendor command and prepare response
// Default function (can be overridden)
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
// this function is declared as __weak in DAP.c
uint32_t DAP_ProcessVendorCommand(uint8_t *request, uint8_t *response)
{
  *response = ID_DAP_Invalid;
  return (1);
}

// Process DAP command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
uint32_t DAP_ProcessCommand(uint8_t *request, uint8_t *response)
{
  uint32_t num;

  if ((*request >= ID_DAP_Vendor0) && (*request <= ID_DAP_Vendor31)) {
    return DAP_ProcessVendorCommand(request, response);
  }

  *response++ = *request;

  switch (*request++) {
    case ID_DAP_Info:
      num = DAP_Info(*request, response+1);
      *response = num;
      return (2 + num);
    case ID_DAP_HostStatus:
      num = DAP_HostStatus(request, response);
      break;
    case ID_DAP_Connect:
      num = DAP_Connect(request, response);
      break;
    case ID_DAP_Disconnect:
      num = DAP_Disconnect(response);
      break;

    case ID_DAP_TransferConfigure:
      num = DAP_TransferConfigure(request, response);
      break;

    case ID_DAP_Transfer:
      num = DAP_SWD_Transfer (request, response);
      break;

    case ID_DAP_TransferBlock:
      num = DAP_SWD_TransferBlock (request, response);
      break;

    case ID_DAP_WriteABORT:
      num = DAP_SWD_Abort (request, response);
      break;

    case ID_DAP_SWJ_Pins:
      num = DAP_SWJ_Pins(request, response);
      break;
    case ID_DAP_SWJ_Clock:
      num = DAP_SWJ_Clock(request, response);
      break;
    case ID_DAP_SWJ_Sequence:
      num = DAP_SWJ_Sequence(request, response);
      break;
    case ID_DAP_SWD_Configure:
      num = DAP_SWD_Configure(request, response);
      break;

    default:
      *(response-1) = ID_DAP_Invalid;
      return (1);
  }

  return (1 + num);
}

// Setup DAP
void DAP_Setup(void) {

  // Default settings (only non-zero values)
//DAP_Data.debug_port  = 0;
//DAP_Data.fast_clock  = 0;
  DAP_Data.clock_delay = 2;
//DAP_Data.transfer.idle_cycles = 0;
  DAP_Data.transfer.retry_count = 100;
//DAP_Data.transfer.match_retry = 0;
//DAP_Data.transfer.match_mask  = 0x000000;
  DAP_Data.swd_conf.turnaround  = 1;
//DAP_Data.swd_conf.data_phase  = 0;

  PORT_OFF();
}