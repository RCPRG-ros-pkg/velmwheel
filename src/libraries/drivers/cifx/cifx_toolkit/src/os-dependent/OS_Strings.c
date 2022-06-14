/* ============================================================================================================================ *//**
 * @file       OS_Strings.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es string-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <stdint.h>
#include <strings.h>
#include <string.h>

/* ======================================================= Public functions ======================================================= */

int OS_Strcmp(const char* pszBuf1, const char* pszBuf2) {
	return strcmp(pszBuf1, pszBuf2);
}


int OS_Strnicmp(const char* pszBuf1, const char* pszBuf2, uint32_t ulLen) {
	return strncasecmp(pszBuf1, pszBuf2, ulLen);
}


int OS_Strlen(const char* szText) {
	return strlen(szText);
}


char* OS_Strncpy(char* szDest, const char* szSource, uint32_t ulLength) {
	return strncpy(szDest, szSource, ulLength);
}

/* ================================================================================================================================ */