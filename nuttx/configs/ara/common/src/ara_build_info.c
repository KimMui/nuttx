/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nuttx/config.h>
#include <debug.h>
#include <ara_build_info.h>
#include <ara_build_info_priv.h>

#ifdef CONFIG_ARCH_LOWPUTC
#define early_dbg(fmt, ...) lowsyslog(fmt, ##__VA_ARGS__)
#endif

#define ROMSTRING __attribute__ ((section(".rodata"),aligned(4)))

const char __ara_build_name[] ROMSTRING = {ARA_BUILD_NAME};
const char __ara_nuttx_version[]  ROMSTRING = {ARA_NUTTX_VERSION};
const char __ara_fdk_version[]  ROMSTRING = {ARA_FDK_VERSION};

void ara_show_build_info(void) {
    early_dbg("__ara_build_name     : '%s'\n", __ara_build_name);
    early_dbg("__ara_nuttx_version  : '%s'\n", __ara_nuttx_version);
    early_dbg("__ara_fdk_version    : '%s'\n", __ara_fdk_version);
}
