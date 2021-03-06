/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2000 Silicon Integrated System Corporation
 * Copyright (C) 2004 Tyan Corp <yhlu@tyan.com>
 * Copyright (C) 2005-2008 coresystems GmbH
 * Copyright (C) 2008,2009,2010 Carl-Daniel Hailfinger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include "flash.h"
#include "flashchips.h"
#include "programmer.h"

#if CONFIG_INTERNAL == 1
static enum programmer default_programmer = PROGRAMMER_INTERNAL;
#elif CONFIG_DUMMY == 1
static enum programmer default_programmer = PROGRAMMER_DUMMY;
#else
/* If neither internal nor dummy are selected, we must pick a sensible default.
 * Since there is no reason to prefer a particular external programmer, we fail
 * if more than one of them is selected. If only one is selected, it is clear
 * that the user wants that one to become the default.
 */
#if CONFIG_NIC3COM+CONFIG_NICREALTEK+CONFIG_NICNATSEMI+CONFIG_GFXNVIDIA+CONFIG_DRKAISER+CONFIG_SATASII+CONFIG_ATAHPT+CONFIG_FT2232_SPI+CONFIG_SERPROG+CONFIG_BUSPIRATE_SPI+CONFIG_DEDIPROG+CONFIG_RAYER_SPI+CONFIG_NICINTEL+CONFIG_NICINTEL_SPI+CONFIG_OGP_SPI+CONFIG_SATAMV > 1
#error Please enable either CONFIG_DUMMY or CONFIG_INTERNAL or disable support for all programmers except one.
#endif
static enum programmer default_programmer =
#if CONFIG_NIC3COM == 1
	PROGRAMMER_NIC3COM
#endif
#if CONFIG_NICREALTEK == 1
	PROGRAMMER_NICREALTEK
#endif
#if CONFIG_NICNATSEMI == 1
	PROGRAMMER_NICNATSEMI
#endif
#if CONFIG_GFXNVIDIA == 1
	PROGRAMMER_GFXNVIDIA
#endif
#if CONFIG_DRKAISER == 1
	PROGRAMMER_DRKAISER
#endif
#if CONFIG_SATASII == 1
	PROGRAMMER_SATASII
#endif
#if CONFIG_ATAHPT == 1
	PROGRAMMER_ATAHPT
#endif
#if CONFIG_FT2232_SPI == 1
	PROGRAMMER_FT2232_SPI
#endif
#if CONFIG_SERPROG == 1
	PROGRAMMER_SERPROG
#endif
#if CONFIG_BUSPIRATE_SPI == 1
	PROGRAMMER_BUSPIRATE_SPI
#endif
#if CONFIG_DEDIPROG == 1
	PROGRAMMER_DEDIPROG
#endif
#if CONFIG_RAYER_SPI == 1
	PROGRAMMER_RAYER_SPI
#endif
#if CONFIG_NICINTEL == 1
	PROGRAMMER_NICINTEL
#endif
#if CONFIG_NICINTEL_SPI == 1
	PROGRAMMER_NICINTEL_SPI
#endif
#if CONFIG_OGP_SPI == 1
	PROGRAMMER_OGP_SPI
#endif
#if CONFIG_SATAMV == 1
	PROGRAMMER_SATAMV
#endif
#if CONFIG_LINUX_MTD == 1
	PROGRAMMER_LINUX_MTD
#endif
#if CONFIG_LINUX_SPI == 1
	PROGRAMMER_LINUX_SPI
#endif
;
#endif

static void cli_classic_usage(const char *name)
{
	printf("Usage: flashrom [-n] [-V] [-f] [-h|-R|-L|"
#if CONFIG_PRINT_WIKI == 1
	         "-z|"
#endif
	         "-E|-r <file>|-w <file>|-v <file>]\n"
	       "       [-c <chipname>] [-m [<vendor>:]<part>] [-l <file>]\n"
	       "       [-i <image>] [-p <programmername>[:<parameters>]]\n\n");

	printf("Please note that the command line interface for flashrom has "
	         "changed between\n"
	       "0.9.1 and 0.9.2 and will change again before flashrom 1.0.\n"
	       "Do not use flashrom in scripts or other automated tools "
	         "without checking\n"
	       "that your flashrom version won't interpret options in a "
	         "different way.\n\n");

	printf("   -h | --help                       print this help text\n"
	       "   -R | --version                    print version (release)\n"
	       "   -r | --read <file|->              read flash and save to "
	         "<file> or write on the standard output\n"
	       "   -w | --write <file|->             write <file> or "
	         "the content provided on the standard input to flash\n"
	       "   -v | --verify <file|->            verify flash against "
	         "<file> or the content provided on the standard input\n"
	       "   -E | --erase                      erase flash device\n"
	       "   -V | --verbose                    more verbose output\n"
	       "   -c | --chip <chipname>            probe only for specified "
	         "flash chip\n"
#if CONFIG_INTERNAL == 1
	       /* FIXME: --mainboard should be a programmer parameter */
	       "   -m | --mainboard <[vendor:]part>  override mainboard "
	         "detection\n"
#endif
	       "   -f | --force                      force specific operations "
	         "(see man page)\n"
	       "   -n | --noverify                   don't auto-verify\n"
	       "   -l | --layout <file>              read ROM layout from "
	         "<file>\n"
	       "   -i | --image <name>               only flash image <name> "
	         "from flash layout\n"
	       "   -L | --list-supported             print supported devices\n"
#if CONFIG_PRINT_WIKI == 1
	       "   -z | --list-supported-wiki        print supported devices "
	         "in wiki syntax\n"
#endif
	       "   -p | --programmer <name>[:<param>] specify the programmer "
	         "device\n");

	list_programmers_linebreak(37, 80, 1);
	printf("\nYou can specify one of -h, -R, -L, "
#if CONFIG_PRINT_WIKI == 1
	         "-z, "
#endif
	         "-E, -r, -w, -v or no operation.\n"
	       "If no operation is specified, flashrom will only probe for "
	         "flash chips.\n\n");
}

static void cli_classic_abort_usage(void)
{
	printf("Please run \"flashrom --help\" for usage info.\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	unsigned long size;
	/* Probe for up to three flash chips. */
	const struct flashchip *flash;
	struct flashctx flashes[3];
	struct flashctx *fill_flash;
	const char *name;
	int namelen, opt, i;
	int startchip = 0, chipcount = 0, option_index = 0, force = 0;
#if CONFIG_PRINT_WIKI == 1
	int list_supported_wiki = 0;
#endif
	int read_it = 0, write_it = 0, erase_it = 0, verify_it = 0;
	int dont_verify_it = 0, list_supported = 0, operation_specified = 0;
	enum programmer prog = PROGRAMMER_INVALID;
	int ret = 0;

	static const char optstring[] = "r:Rw:v:nVEfc:m:l:i:p:Lzh";
	static const struct option long_options[] = {
		{"read",		1, NULL, 'r'},
		{"write",		1, NULL, 'w'},
		{"erase",		0, NULL, 'E'},
		{"verify",		1, NULL, 'v'},
		{"noverify",		0, NULL, 'n'},
		{"chip",		1, NULL, 'c'},
		{"mainboard",		1, NULL, 'm'},
		{"verbose",		0, NULL, 'V'},
		{"force",		0, NULL, 'f'},
		{"layout",		1, NULL, 'l'},
		{"image",		1, NULL, 'i'},
		{"list-supported",	0, NULL, 'L'},
		{"list-supported-wiki",	0, NULL, 'z'},
		{"programmer",		1, NULL, 'p'},
		{"help",		0, NULL, 'h'},
		{"version",		0, NULL, 'R'},
		{NULL,			0, NULL, 0},
	};

	char *filename = NULL;
	char *diff_file = NULL;

	char *tempstr = NULL;
	char *pparam = NULL;

	print_version();
	print_banner();

	if (selfcheck())
		exit(1);

	setbuf(stdout, NULL);
	/* FIXME: Delay all operation_specified checks until after command
	 * line parsing to allow --help overriding everything else.
	 */
	while ((opt = getopt_long(argc, argv, optstring,
				  long_options, &option_index)) != EOF) {
		switch (opt) {
		case 'r':
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			filename = strdup(optarg);
			read_it = 1;
			break;
		case 'w':
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			filename = strdup(optarg);
			write_it = 1;
			break;
		case 'v':
			//FIXME: gracefully handle superfluous -v
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			if (dont_verify_it) {
				fprintf(stderr, "--verify and --noverify are"
					"mutually exclusive. Aborting.\n");
				cli_classic_abort_usage();
			}
			filename = strdup(optarg);
			verify_it = 1;
			break;
		case 'n':
			if (verify_it) {
				fprintf(stderr, "--verify and --noverify are"
					"mutually exclusive. Aborting.\n");
				cli_classic_abort_usage();
			}
			dont_verify_it = 1;
			break;
		case 'c':
			chip_to_probe = strdup(optarg);
			break;
		case 'V':
			verbose++;
			break;
		case 'E':
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			erase_it = 1;
			break;
		case 'm':
#if CONFIG_INTERNAL == 1
			tempstr = strdup(optarg);
			lb_vendor_dev_from_string(tempstr);
#else
			fprintf(stderr, "Error: Internal programmer support "
				"was not compiled in and --mainboard only\n"
				"applies to the internal programmer. Aborting.\n");
			cli_classic_abort_usage();
#endif
			break;
		case 'f':
			force = 1;
			break;
		case 'l':
			tempstr = strdup(optarg);
			if (read_romlayout(tempstr))
				cli_classic_abort_usage();
			break;
		case 'i':
			/* FIXME: -l has to be specified before -i. */
			tempstr = strdup(optarg);
			if (find_romentry(tempstr) < 0) {
				fprintf(stderr, "Error: image %s not found in "
					"layout file or -i specified before "
					"-l\n", tempstr);
				cli_classic_abort_usage();
			}
			break;
		case 'L':
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			list_supported = 1;
			break;
		case 'z':
#if CONFIG_PRINT_WIKI == 1
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			list_supported_wiki = 1;
#else
			fprintf(stderr, "Error: Wiki output was not compiled "
				"in. Aborting.\n");
			cli_classic_abort_usage();
#endif
			break;
		case 'p':
			if (prog != PROGRAMMER_INVALID) {
				fprintf(stderr, "Error: --programmer specified "
					"more than once. You can separate "
					"multiple\nparameters for a programmer "
					"with \",\". Please see the man page "
					"for details.\n");
				cli_classic_abort_usage();
			}
			for (prog = 0; prog < PROGRAMMER_INVALID; prog++) {
				name = programmer_table[prog].name;
				namelen = strlen(name);
				if (strncmp(optarg, name, namelen) == 0) {
					switch (optarg[namelen]) {
					case ':':
						pparam = strdup(optarg + namelen + 1);
						if (!strlen(pparam)) {
							free(pparam);
							pparam = NULL;
						}
						break;
					case '\0':
						break;
					default:
						/* The continue refers to the
						 * for loop. It is here to be
						 * able to differentiate between
						 * foo and foobar.
						 */
						continue;
					}
					break;
				}
			}
			if (prog == PROGRAMMER_INVALID) {
				fprintf(stderr, "Error: Unknown programmer "
					"%s.\n", optarg);
				cli_classic_abort_usage();
			}
			break;
		case 'R':
			/* print_version() is always called during startup. */
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			exit(0);
			break;
		case 'h':
			if (++operation_specified > 1) {
				fprintf(stderr, "More than one operation "
					"specified. Aborting.\n");
				cli_classic_abort_usage();
			}
			cli_classic_usage(argv[0]);
			exit(0);
			break;
		default:
			cli_classic_abort_usage();
			break;
		}
	}

	if (optind < argc) {
		fprintf(stderr, "Error: Extra parameter found.\n");
		cli_classic_abort_usage();
	}

	/* FIXME: Print the actions flashrom will take. */

	if (list_supported) {
		print_supported();
		exit(0);
	}

#if CONFIG_PRINT_WIKI == 1
	if (list_supported_wiki) {
		print_supported_wiki();
		exit(0);
	}
#endif

	/* Does a chip with the requested name exist in the flashchips array? */
	if (chip_to_probe) {
		for (flash = flashchips; flash && flash->name; flash++)
			if (!strcmp(flash->name, chip_to_probe))
				break;
		if (!flash || !flash->name) {
			fprintf(stderr, "Error: Unknown chip '%s' specified.\n",
				chip_to_probe);
			printf("Run flashrom -L to view the hardware supported "
			       "in this flashrom version.\n");
			exit(1);
		}
		/* Clean up after the check. */
		flash = NULL;
	}

	if (prog == PROGRAMMER_INVALID)
		prog = default_programmer;

#if CONFIG_INTERNAL == 1
	if ((prog != PROGRAMMER_INTERNAL) && (lb_part || lb_vendor)) {
		fprintf(stderr, "Error: --mainboard requires the internal "
				"programmer. Aborting.\n");
		cli_classic_abort_usage();
	}
#endif

	/* FIXME: Delay calibration should happen in programmer code. */
	myusec_calibrate_delay();

	if (programmer_init(prog, pparam)) {
		fprintf(stderr, "Error: Programmer initialization failed.\n");
		ret = 1;
		goto out_shutdown;
	}
	tempstr = flashbuses_to_text(buses_supported);
	msg_pdbg("This programmer supports the following protocols: %s.\n",
		 tempstr);
	free(tempstr);

	for (i = 0; i < ARRAY_SIZE(flashes); i++) {
		startchip = probe_flash(startchip, &flashes[i], 0);
		if (startchip == -1)
			break;
		chipcount++;
		startchip++;
	}

	if (chipcount > 1) {
		printf("Multiple flash chips were detected: \"%s\"",
			flashes[0].name);
		for (i = 1; i < chipcount; i++)
			printf(", \"%s\"", flashes[i].name);
		printf("\nPlease specify which chip to use with the "
		       "-c <chipname> option.\n");
		ret = 1;
		goto out_shutdown;
	} else if (!chipcount) {
		printf("No EEPROM/flash device found.\n");
		if (!force || !chip_to_probe) {
			printf("Note: flashrom can never write if the flash "
			       "chip isn't found automatically.\n");
		}
		if (force && read_it && chip_to_probe) {
			printf("Force read (-f -r -c) requested, pretending "
			       "the chip is there:\n");
			startchip = probe_flash(0, &flashes[0], 1);
			if (startchip == -1) {
				printf("Probing for flash chip '%s' failed.\n",
				       chip_to_probe);
				ret = 1;
				goto out_shutdown;
			}
			printf("Please note that forced reads most likely "
			       "contain garbage.\n");
			return read_flash_to_file(&flashes[0], filename);
		}
		ret = 1;
		goto out_shutdown;
	} else if (!chip_to_probe) {
		/* repeat for convenience when looking at foreign logs */
		tempstr = flashbuses_to_text(flashes[0].bustype);
		msg_gdbg("Found %s flash chip \"%s\" (%d kB, %s).\n",
			 flashes[0].vendor, flashes[0].name,
			 flashes[0].total_size, tempstr);
		free(tempstr);
	}

	fill_flash = &flashes[0];

	check_chip_supported(fill_flash);

	size = fill_flash->total_size * 1024;
	if (check_max_decode((buses_supported & fill_flash->bustype), size) &&
	    (!force)) {
		fprintf(stderr, "Chip is too big for this programmer "
			"(-V gives details). Use --force to override.\n");
		ret = 1;
		goto out_shutdown;
	}

	if (!(read_it | write_it | verify_it | erase_it)) {
		printf("No operations were specified.\n");
		goto out_shutdown;
	}

	if (!filename && !erase_it) {
		printf("Error: No filename specified.\n");
		ret = 1;
		goto out_shutdown;
	}

	/* Always verify write operations unless -n is used. */
	if (write_it && !dont_verify_it)
		verify_it = 1;

	/* FIXME: We should issue an unconditional chip reset here. This can be
	 * done once we have a .reset function in struct flashchip.
	 * Give the chip time to settle.
	 */
	programmer_delay(100000);
	return doit(fill_flash, force, filename, read_it, write_it, erase_it, verify_it, diff_file);

out_shutdown:
	programmer_shutdown();
	return ret;
}
