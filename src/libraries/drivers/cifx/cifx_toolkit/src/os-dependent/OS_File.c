/* ============================================================================================================================ *//**
 * @file       OS_File.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es files-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include "cifXToolkit.h"
#include "cifxDriver.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_file";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Returns the length of the open file
 * 
 * @param file
 *    open file
 * @param file_length [out]
 *    file's length
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int get_file_length(FILE* file, uint32_t* file_length)
{
	assert(file != NULL);
	assert(file_length != NULL);

	xTraceDebug(context, "Getting length of the file...");

	// Seek write pointer to the file's end
	const long seek_offset = 0;
	if(fseek(file, seek_offset, SEEK_END) == -1) {
		xTraceGlobalSystemError(context, "Could not get file length");
		return -1;
	}

	// Get position of the write pointer
	const long length = ftell(file);
	if(length == -1) {
		xTraceGlobalSystemError(context, "Could not get file length");
		return -1;
	}

	// Set write pointer to the file's beginning
	if(fseek(file, seek_offset, SEEK_SET) == -1) {
		xTraceGlobalSystemError(context, "Could not get file length");
		return -1;
	}

	*file_length = length;
	xTraceDebug(context, "File length is %lu", length);
	
	return 0;
}

/* ======================================================= Public functions ======================================================= */

void* OS_FileOpen(char* szFile, uint32_t* pulFileLen) {

	assert(pulFileLen != NULL);
	xTraceDebug(context, "Opening file '%s'...", szFile);

	// Open the file
	FILE* file = fopen(szFile, "rb");
	if(file == NULL) {
		xTraceGlobalSystemError(context, "Could not open '%s' file", szFile);
		return NULL;
	}

	// Get size of the file
	if(get_file_length(file, pulFileLen) == -1)	{
		fclose(file);
		return NULL;
	}

	xTraceDebug(context, "File opened successfully");
	return file;
}


void OS_FileClose(void* pvFile) {
	assert(pvFile != NULL);
	FILE* file = (FILE*) pvFile;
	const int ec = fclose(file);
	assert(ec != EOF);
	xTraceDebug(context, "File closed");
}


uint32_t OS_FileRead(void* pvFile, uint32_t ulOffset, uint32_t ulSize, void* pvBuffer) {

	assert(pvFile != NULL);
	assert(pvBuffer != NULL);

	// Cast to the OS-specific file structure
	FILE* file = (FILE*) pvFile;

	xTraceDebug(context, "Seeking file into offset=%x...", ulOffset);	

	// Set read pointer to the requested location
	if(fseek(file, ulOffset, SEEK_SET) == -1) {
		xTraceGlobalSystemError(context, "Could not seek file to given offset");
		return 0;
	}

	xTraceDebug(context, "Reading %u bytes from file...", ulSize);

	// Read data from the file
	const size_t nmemb = 1;
	const size_t ulRet = fread(pvBuffer, nmemb, ulSize, file);
	if(ferror(file) != 0) {
		xTraceGlobalSystemError(context, "Could not read from file");
	}

	return ulRet;
}

/* ================================================================================================================================ */
