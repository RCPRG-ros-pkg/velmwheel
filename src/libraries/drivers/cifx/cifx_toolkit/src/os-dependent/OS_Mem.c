/* ============================================================================================================================ *//**
 * @file       OS_Mem.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es memory-allocation-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ======================================================= Public functions ======================================================= */

void* OS_Memalloc(uint32_t ulSize) {
	return malloc(ulSize);
}


void OS_Memfree(void* pvMem) {
	free(pvMem);
}


void* OS_Memrealloc(void* pvMem, uint32_t ulNewSize) {
	return realloc(pvMem, ulNewSize);
}


void OS_Memset(void* pvMem, uint8_t bFill, uint32_t ulSize) {
	memset(pvMem, bFill, ulSize);
}


void OS_Memcpy(void* pvDest, const void* pvSrc, uint32_t ulSize) {
	memcpy(pvDest, pvSrc, ulSize);
}


int OS_Memcmp(const void* pvBuf1, const void* pvBuf2, uint32_t ulSize) {
	return memcmp(pvBuf1, pvBuf2, ulSize);
}


void OS_Memmove(void* pvDest, const void* pvSrc, uint32_t ulSize) {
	memmove(pvDest, pvSrc, ulSize);
}


void* OS_MapUserPointer(void* pvDriverMem, uint32_t ulMemSize, void** ppvMappedMem, void* pvOSDependent) {
	(void)(ulMemSize);
	(void)(pvOSDependent);
	assert(ppvMappedMem != NULL);
	*ppvMappedMem = pvDriverMem;
	return pvDriverMem;
}


int OS_UnmapUserPointer(void* phMapping, void* pvOSDependent) {
	(void)(phMapping);
	(void)(pvOSDependent);
	return 1;
}

/* ================================================================================================================================ */
