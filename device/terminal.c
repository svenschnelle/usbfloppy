/*
 * terminal.c
 *
 *  Created on: 11 oct 2021
 *      Author: v.simonenko
 */

#include "terminal.h"
#include <string.h>
#include "virtualComPort.h"
#include <stdint.h>
#include <stdio.h>

void term_send_data(const char *data, int16_t size)
{
	while (size--)
	{
		VCP_AddToTxBuff(*data++);
	}
}

void test_cmd1(const char *data)
{
	term_send_data(data, strlen(data));
}

void test_cmd2(const char *data)
{
	term_send_data("command executed", strlen("command executed"));
}

void toggle_cmd(const char *data)
{
	term_send_data("LED switched", strlen("LED switched"));
}

void help_cmd(const char *data)
{
	static const char str[] =
			"\x1B[92mAvailable commands:\x1B[0m\r\n"
			"test-arg\r\n"
			"command\r\n"
			"toggle\r\n"
			"help";
	term_send_data(str, strlen(str));
}

term_srv_cmd_t cmd_list[] =
{
	{ .cmd = "help", .len = 4, .handler = help_cmd },
};

size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);
