/* SPDX-License-Identifier: MIT */
/*
 * sdio_test.h - sdio stm32f407vgt6 test utility
 *
 * Copyright (c) 2025 Michael Kaa
 *
 */

#ifndef _SDIO_TEST_H
#define _SDIO_TEST_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#include "dev_interface.h"

int ucmd_sdio(int argc, char **argv);

#endif /* _SDIO_TEST_H */


