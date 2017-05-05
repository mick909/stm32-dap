STM32-DAP
====

CMSIS/DAP implementation for STM32.

SWD interface only. (JTAG interface not supported)  
Support CDC intarface.

## Supported MCU/board

#### BluePill board (STM32F103C8T6)

GPIO assignment:
```
PA4 -> SWDIO  to target device  
PA5 -> SWCLK  to target device
PA6 -> nRESET to target device

PA2 : USART TX (to target device's RX)
PA3 : USART RX (to target device's TX)

PA11 : USB DM (to host)
PA12 : USB DP (to host)
```
**NOTICE**  
remove R10 on board.  
and connect PA15 to PA12 with 1.5k ohm resistor.

## Requirement

[launchpad GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded)

## Licence

* CubeMX firmware
 Copyright (c) 2017 STMicroelectronics International N.V.  
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted, provided that the following conditions are met:
 
  1. Redistribution of source code must retain the above copyright notice,  this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of other contributors to this software may be used to endorse or promote products derived from this software without specific written permission.
  4. This software, including modifications and/or derivative works of this software, must execute solely and exclusively on microcontroller or microprocessor devices manufactured by or for STMicroelectronics.
  5. Redistribution and use of this software other than as permitted under this license is void and will automatically terminate your rights under this license. 

* DAP implementation
 DAPLink Interface Firmware  
 Copyright (c) 2009-2016, ARM Limited, All Rights Reserved  
 SPDX-License-Identifier: Apache-2.0  
[APACHE2.0](http://www.apache.org/licenses/LICENSE-2.0)