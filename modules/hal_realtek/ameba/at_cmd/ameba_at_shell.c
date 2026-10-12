/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* AT commands at the shell prompt, e.g. "AT+SEC=RSIP_EN,temp" */

#include <string.h>

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/shell/shell.h>

/* hal_realtek atcmd_parser.c, returns NULL if the command is not found */
extern void *atcmd_handler(char *cmd);

bool shell_raw_line_hook(const struct shell *sh, char *line, int *ret)
{
	char buf[CMD_BUFLEN];
	size_t len;

	/* Leading spaces are only trimmed with CONFIG_SHELL_HISTORY */
	while (*line == ' ') {
		line++;
	}

	if (strncmp(line, "AT+", 3) != 0) {
		return false;
	}

	/* The AT parser silently truncates longer lines */
	len = strlen(line);
	if (len >= sizeof(buf)) {
		shell_error(sh, "AT command too long (max %u)", (unsigned int)sizeof(buf) - 1);
		*ret = -EINVAL;
		return true;
	}

	/* The AT parser edits its input, keep the line for the shell fallback */
	memcpy(buf, line, len + 1);
	if (atcmd_handler(buf) == NULL) {
		return false;
	}

	*ret = 0;
	return true;
}
