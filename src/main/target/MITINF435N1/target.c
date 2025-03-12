/*
* This file is part of Cleanflight.
*
* Cleanflight is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Cleanflight is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

timerHardware_t timerHardware[] = {

    DEF_TIM(TMR2, CH1, PA5,  TIM_USE_OUTPUT_AUTO, 0,0), // motor1 DMA2 CH7
    DEF_TIM(TMR2, CH2, PA1,  TIM_USE_OUTPUT_AUTO, 0,2), // motor2 DMA2 CH6
    DEF_TIM(TMR2, CH3, PB10,  TIM_USE_OUTPUT_AUTO, 0,1), // motor3 DMA2 CH5
    DEF_TIM(TMR2, CH4, PB11,  TIM_USE_OUTPUT_AUTO, 0,3), // motor4 DMA2 CH4

    DEF_TIM(TMR3, CH1, PA6,  TIM_USE_OUTPUT_AUTO, 0,11), // PWM1 - OUT5  DMA1 CH7
    DEF_TIM(TMR3, CH2, PA7,  TIM_USE_OUTPUT_AUTO, 0,10), // PWM2 - OUT6  DMA2 CH1
    DEF_TIM(TMR3, CH3, PB0,  TIM_USE_OUTPUT_AUTO, 0,9),  // PWM3 - OUT7  DMA2 CH2
    DEF_TIM(TMR3, CH4, PB1,  TIM_USE_OUTPUT_AUTO, 0,8),  // PWM4 - OUT8  DMA2 CH3

    // DEF_TIM(TMR5, CH1, PA0,  TIM_USE_ANY,  0, 13), // TIM_USE_CAMERA_CONTROL
    DEF_TIM(TMR8, CH4, PC9,  TIM_USE_LED, 0,6), // PWM1 - LED MCO1 DMA1 CH2
    // DEF_TIM(TMR2, CH4, PA3,  TIM_USE_ANY |TIM_USE_PPM, 0,6), // PWM2 - PPM DMA1 CH6
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
  
// Define TIM device/DMA/MUX 
/*
#define DEF_TIM(tim, ch, pin, usage, flags, dmavar)     {               \
    tim,                                                               \
    IO_TAG(pin),                                                       \
    DEF_TIM_CHNL_ ## ch,                                               \
    DEF_TIM_OUTPUT(ch) | flags,                                        \
    IOCFG_AF_PP,                                                       \
    DEF_TIM_AF(TCH_## tim ## _ ## ch, pin),                            \
    usage,                                                             \
    DEF_TIM_DMAMAP(dmavar, tim ## _ ## ch),                            \
    DEF_TIM_DMA_REQUEST(tim ## _ ## ch)                                \
 }
*/