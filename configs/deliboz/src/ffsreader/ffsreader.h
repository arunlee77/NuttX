/****************************************************************************
 *
 *   Copyright (c) 2019 Intel Corp. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file ffsreader.h
 *
 * Read blocks from lfc_ffs
 *
 */

int ffsreader_read(uint8_t fileid, uint8_t **buffer, uint32_t *size);

