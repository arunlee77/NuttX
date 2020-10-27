/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: Jari Nippula <jari.nippula@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_identity.c
 * Implementation of Non Arch specific Board identity API
 */

#include <px4_config.h>
#include <stdio.h>
#include <string.h>
#include <lib/parameters/flashparams/flashfs.h>

#define BOARD_UUID_MAX_SIZE 17

static char board_uuid[BOARD_UUID_MAX_SIZE] = {0};

int board_identity_store(char* idstring)
{
	uint8_t *fbuffer;
	size_t buf_size = BOARD_UUID_MAX_SIZE;

	/* Store guid into flashfs */
	int rv = parameter_flashfs_alloc(board_info_token, &fbuffer, &buf_size);
	if (rv < 0)
		return rv;

	fbuffer[0] = 0;
	strncat((char*)fbuffer, idstring, BOARD_UUID_MAX_SIZE-1);
	rv = parameter_flashfs_write(board_info_token, fbuffer, buf_size);

	parameter_flashfs_free();

	return rv;
}

void board_identity_init(void)
{
	if (board_uuid[0] == 0) {
		uint8_t *fbuffer;
		size_t buf_size;

		int read = parameter_flashfs_read(board_info_token, &fbuffer, &buf_size);

		if (read >= 0) {
			strncat(board_uuid, (char*)fbuffer, BOARD_UUID_MAX_SIZE-1);
		} else {

			char buffer[BOARD_UUID_MAX_SIZE];
			buffer[0] = 0;

			/* serial not found from flashfs, try legacy LFC FS */
			board_get_hw_serial_string(buffer, BOARD_UUID_MAX_SIZE);

			/* If found, store serial to flashfs and board_uuid */
			if (strlen(buffer)) {
				board_identity_store(buffer);
			}

		}
	}
}

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	board_identity_init();
	unsigned int len = strlen(board_uuid);
	if (len > PX4_CPU_UUID_BYTE_LENGTH) {
		len = PX4_CPU_UUID_BYTE_LENGTH;
	}

	uint8_t *bp = (uint8_t *) uuid_words;

	for (unsigned int i = 0; i < len; i++) {
		*bp++ = board_uuid[i];
	}

	for (unsigned int i = len; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		*bp++ = '0';
	}
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	board_identity_init();
	format_buffer[0] = 0;
	strncat(format_buffer, board_uuid, size-1);
	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	int i;
	board_identity_init();
	((char *) mfgid)[0] = 0;
	strncpy((char *) mfgid, board_uuid, PX4_CPU_MFGUID_BYTE_LENGTH-1);
	for (i = strlen(board_uuid); i < PX4_CPU_MFGUID_BYTE_LENGTH; i++) {
		mfgid[i] = 0;
	}
	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	board_identity_init();
	format_buffer[0] = 0;
	strncat(format_buffer, board_uuid, size-1);
	return strlen(format_buffer);
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	board_identity_init();
	px4_guid[0] = 0;
	strncat((char *) px4_guid, board_uuid, PX4_GUID_BYTE_LENGTH-1);
	for (int i = strlen(board_uuid); i < PX4_GUID_BYTE_LENGTH; i++) {
		px4_guid[i] = 0;
	}
	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	board_identity_init();
	format_buffer[0] = 0;
	strncat(format_buffer, board_uuid, size-1);
	return strlen(format_buffer);
}

int board_set_px4_guid_formated(char *format_buffer)
{
	/* Copy guid into board_uuid string */
	board_uuid[0] = 0;
	strncat(board_uuid, format_buffer, BOARD_UUID_MAX_SIZE-1);

	/* Store guid into flashfs */
	int rv = board_identity_store(board_uuid);

	if (rv < 0)
		return rv;

	return strlen(board_uuid);
}
