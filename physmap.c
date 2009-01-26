/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2009 Peter Stuge <peter@stuge.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include "flash.h"

#ifdef __DARWIN__
#include <DirectIO/darwinio.h>

#define MEM_DEV "DirectIO"

void *sys_physmap(unsigned long phys_addr, size_t len)
{
	return map_physical(phys_addr, len);
}

void physunmap(void *virt_addr, size_t len)
{
	unmap_physical(virt_addr, len);
}

#else
#include <sys/mman.h>

#if defined (__sun) && (defined(__i386) || defined(__amd64))
#  define MEM_DEV "/dev/xsvc"
#else
#  define MEM_DEV "/dev/mem"
#endif

static int fd_mem = -1;

void *sys_physmap(unsigned long phys_addr, size_t len)
{
	void *virt_addr;

	if (-1 == fd_mem) {
		/* Open the memory device UNCACHED. Important for MMIO. */
		if (-1 == (fd_mem = open(MEM_DEV, O_RDWR|O_SYNC))) {
			perror("Critical error: open(" MEM_DEV ")");
			exit(2);
		}
	}

	virt_addr = mmap(0, len, PROT_WRITE|PROT_READ, MAP_SHARED, fd_mem, (off_t)phys_addr);
	return MAP_FAILED == virt_addr ? NULL : virt_addr;
}

void physunmap(void *virt_addr, size_t len)
{
	munmap(virt_addr, len);
}
#endif

void *physmap(const char *descr, unsigned long phys_addr, size_t len)
{
	void *virt_addr = sys_physmap(phys_addr, len);

	if (NULL == virt_addr) {
		if (NULL == descr)
			descr = "memory";
		fprintf(stderr, "Error accessing %s, 0x%lx bytes at 0x%08lx\n", descr, (unsigned long)len, phys_addr);
		perror(MEM_DEV " mmap failed");
		if (EINVAL == errno) {
			fprintf(stderr, "In Linux this error can be caused by the CONFIG_NONPROMISC_DEVMEM (<2.6.27),\n");
			fprintf(stderr, "CONFIG_STRICT_DEVMEM (>=2.6.27) and CONFIG_X86_PAT kernel options.\n");
			fprintf(stderr, "Please check if either is enabled in your kernel before reporting a failure.\n");
			fprintf(stderr, "You can override CONFIG_X86_PAT at boot with the nopat kernel parameter but\n");
			fprintf(stderr, "disabling the other option unfortunately requires a kernel recompile. Sorry!\n");
		}
		exit(3);
	}

	return virt_addr;
}
