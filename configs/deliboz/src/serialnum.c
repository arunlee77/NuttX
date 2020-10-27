/****************************************************************************
 *
 *   Copyright (c) 2019 Intel Corp. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file serialnum.c
 *
 * Read serial number using ffsreader
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "board_config.h"
#include "ffsreader/ffsreader.h"

#define ATOS_FFS_FILEID_BINDING_CONFIGURATION 0x40

typedef struct __attribute((packed))
{
        uint16_t year;
        uint16_t workWeek;
        uint16_t networkId;
        uint8_t key[32];
} ATOS_MSG_BINDING_CONFIGURATION;


__EXPORT int board_get_hw_networkid()
{
	uint8_t *buffer = 0;
	ATOS_MSG_BINDING_CONFIGURATION *config;
	uint32_t size = 0;
	int ret;

	ret = ffsreader_read(ATOS_FFS_FILEID_BINDING_CONFIGURATION, &buffer, &size);

	if (!ret) {
		config = (ATOS_MSG_BINDING_CONFIGURATION *) buffer;
		ret = config->networkId;
	}

	if (buffer)
	{
		/* ffreader_read allocates memory for buffer, so it must bee freed here */
		free(buffer);
	}

	return ret;
}


__EXPORT int board_get_hw_serial_string(char *serial, uint32_t size)
{
	uint8_t *buffer = 0;
	ATOS_MSG_BINDING_CONFIGURATION *config;
	uint32_t len = 0;
	int ret;

	if (!serial || !size) {
		// No buffer given for serial number string
		return -EINVAL;
	}

	ret = ffsreader_read(ATOS_FFS_FILEID_BINDING_CONFIGURATION, &buffer, &len);

	if (!ret) {
		config = (ATOS_MSG_BINDING_CONFIGURATION *) buffer;
		snprintf(serial, size, "GHEVT%cFG%01d%02d%04d",
			'0' + board_get_hw_version(),
			config->year%10,
			config->workWeek,
			config->networkId);
	} else {
		serial[0] = 0;
	}

	if (buffer)
	{
		/* ffreader_read allocates memory for buffer, so it must bee freed here */
		free(buffer);
	}

	return ret;
}
