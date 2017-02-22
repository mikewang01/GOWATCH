/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CLING_PINMAP_H
#define CLING_PINMAP_H

#include "pin_names.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PinName pin;
    int direction;
    int function;
#if PIN_VOLTAGE_CHANGE_SUPPORTED
    int voltage;
#endif
    bool high;
		int int_level;
} PinMap;

/*function declaration*/
extern void pin_map_init(void);
extern void pin_set_high(PinName pin);
extern void pin_set_low(PinName pin);
extern int pin_get_status(PinName pin);
extern void pin_toggle(PinName pin);

#ifdef __cplusplus
}
#endif

#endif
