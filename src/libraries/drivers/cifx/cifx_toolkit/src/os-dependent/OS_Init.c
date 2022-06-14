/* ============================================================================================================================ *//**
 * @file       OS_Init.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es OS-initialization functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <stdint.h>
#include "cifXErrors.h"

/* ======================================================= Public functions ======================================================= */

/**
 * @brief OS specific initialization (if needed), called during cifXTKitInit()
 */
int32_t OS_Init() { return CIFX_NO_ERROR; }

/**
 * @brief OS specific de-initialization (if needed), called during cifXTKitInit()
 */
void OS_Deinit() { }

/* ================================================================================================================================ */
