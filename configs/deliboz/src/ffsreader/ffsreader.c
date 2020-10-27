/****************************************************************************
 *
 *   Copyright (c) 2019 Intel Corp. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file ffsreader.c
 *
 * Read blocks from lfc_ffs
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#define LFC_FFS_SECTOR0_START_ADDR  0x81C0000
#define LFC_FFS_SECTOR1_START_ADDR  0x81E0000

#define LFC_FFS_SECTOR0_SIZE 131072U
#define LFC_FFS_SECTOR1_SIZE 131072U

#define ATFFS_DATA_BLOCK_SIZE 256

/* Typedefs */

typedef struct
{
	uint32_t addr;
	uint32_t size;
} sector_t;

typedef struct
{
	unsigned int invalidated[8]; //0xff = valid 0x00 =invalid
	unsigned char used; //0xff = empty  0x55 used
	unsigned char fileId; //0..255
	unsigned short segmentNr; // file segment. if 0x8000 is set, it's the last segment
	unsigned short sizeOfData;
	unsigned short fileCrc; //on first segment, this hold the fileCRC
	unsigned short dummy; //for alignment
} ATFFS_DATA_BLOCK_HEADER;

typedef struct
{
	ATFFS_DATA_BLOCK_HEADER header;
	uint8_t *data;
} FFSREADER_FILE_ENTRY;


/* Private data structures */

static sector_t sectors[] = {
	{LFC_FFS_SECTOR0_START_ADDR, LFC_FFS_SECTOR0_SIZE},
	{LFC_FFS_SECTOR1_START_ADDR, LFC_FFS_SECTOR1_SIZE},
	{0,0}
};


/* Private functions */

static unsigned short _crc_update (unsigned short crc, unsigned char data)
{
	data ^= (crc & 0xff);
	data ^= data << 4;

	return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
			^ ((unsigned short )data << 3));
}


static unsigned short _crc16(const void* data, unsigned short cnt)
{
	unsigned short crc=0xff;
	unsigned char * ptr=(unsigned char *) data;
	int i;

	for (i=0;i<cnt;i++)
	{
		crc=_crc_update(crc,*ptr);
		ptr++;
	}
	return crc;
}


static int _ffsreader_find_file(uint8_t fileid, FFSREADER_FILE_ENTRY *block)
{
	uint8_t *addr;
	uint8_t s;

	if (!block) {
		return -EINVAL;
	}

	for (s=0; sectors[s].addr; s++) {

		addr = (uint8_t*) sectors[s].addr;
		while (addr < (uint8_t*) (sectors[s].addr + sectors[s].size)) {

			memcpy((uint8_t*)&(block->header), addr, sizeof(ATFFS_DATA_BLOCK_HEADER));

			if (block->header.used == 0x55) {
				// written block

				// update block data pointer and calculate next block address
				block->data = (uint8_t*) (addr + sizeof(ATFFS_DATA_BLOCK_HEADER));
				addr += ATFFS_DATA_BLOCK_SIZE;

				if (block->header.invalidated[0] == 0xFFFFFFFF &&
					block->header.fileId == fileid)
				{
					if (_crc16(block->data, block->header.sizeOfData) == block->header.fileCrc) {

						if (block->header.segmentNr & 0x8000) {

							// Valid file found!
							return 0;

						} else {

							// multi block files not supported
							return -EFBIG;

						}

					} else {

						// CRC error, try next
						continue;

					}

				} else {

					// Invalidated block or wrong fileid, try next
					continue;

				}

			} else {

				// empty block, stop seaching this sector
				break;

			}
		}

	}

	return -ENOENT;
}

/* Public functions */

int ffsreader_read(uint8_t fileid, uint8_t **buffer, uint32_t *size)
{
	FFSREADER_FILE_ENTRY file;
	int err = _ffsreader_find_file(fileid, &file);

	if (!err)
	{
		*buffer = malloc(file.header.sizeOfData);
		if (!(*buffer)) {
			return -ENOMEM;
		}
		*size = file.header.sizeOfData;
		memcpy(*buffer, file.data, *size);
		return 0;
	}
	else
	{
		return -ENOENT;
	}
}
