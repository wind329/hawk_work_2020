/*******************************************************************************
 * File Name:     comd_proc.c
 * Created By:    Hawk Ji
 * Created Date:  2010-09-10
 * Version:       1.0
 *
 * Description:   Command handler
 ******************************************************************************/

/*============================ Include Segment ===============================*/

#include <stdio.h>
#include <string.h>

#include "target.h"
#include "comd_proc.h"

/*============================ Declare Segment ===============================*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Type definitions
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

typedef int (*cmdproc)(int argc, char *argv[]);

typedef struct _sys_cmd_process {
	char *cmd;
	char *hlp;
	cmdproc proc;
} stSysComdProcess;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Local function Prototypes
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int help_process(int argc, char *argv[]);
int reboot_process(int argc, char *argv[]);
int cmd2_process(int argc, char *argv[]);
int cmd3_process(int argc, char *argv[]);
int cmd4_process(int argc, char *argv[]);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Variables Define.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int  gwCmdLineArgc;
char *gCmdLineArgv[MAX_ARGS_SIZE];

struct _sys_cmd_process tblSysCommand[] = {
	{"help",	"show command help",			help_process},
	{"reboot",	"System reboot",				reboot_process},
	{"hc595",	"74HC595 io-port test",			cmd2_process},
	{"ads886",	"ads8866 adc test",				cmd3_process},
	{"ad5920",	"ad5920 current-loop test",		cmd4_process},
	{NULL, NULL, NULL}
};

/*============================ Program Segment ===============================*/

/*----------------------------------------------------------------------------**
 * Name:        help_process
 * Description: Command1 process
 * Parameter:   argc - Argument c
 *              argv - Argument v
 * Return:      Operation result
 *----------------------------------------------------------------------------*/

int help_process(int argc, char *argv[])
{
	int i;

	for(i = 0; tblSysCommand[i].cmd != NULL; i++)
	{
		printf(tblSysCommand[i].cmd);
		printf(" --------> ");
		printf(tblSysCommand[i].hlp);
		putchar('\r');
		putchar('\n');
	}

	return 0;
}

/*----------------------------------------------------------------------------**
 * Name:        reboot_process
 * Description: Command1 process
 * Parameter:   argc - Argument c
 *              argv - Argument v
 * Return:      Operation result
 *----------------------------------------------------------------------------*/

int reboot_process(int argc, char *argv[])
{
	printf("CPU reboot!\r\n");
	mcu_restart();

	return 0;
}

/*----------------------------------------------------------------------------**
 * Name:        cmd2_process
 * Description: Command2 process
 * Parameter:   argc - Argument c
 *              argv - Argument v
 * Return:      Operation result
 *----------------------------------------------------------------------------*/

int cmd2_process(int argc, char *argv[])
{
	printf("Command2 is OK!\r\n");

	return 0;
}

/*----------------------------------------------------------------------------**
 * Name:        cmd3_process
 * Description: Command3 process
 * Parameter:   argc - Argument c
 *              argv - Argument v
 * Return:      Operation result
 *----------------------------------------------------------------------------*/

int cmd3_process(int argc, char *argv[])
{
	printf("Command3 is OK!\r\n");

	return 0;
}

/*----------------------------------------------------------------------------**
 * Name:        cmd4_process
 * Description: Command4 process
 * Parameter:   argc - Argument c
 *              argv - Argument v
 * Return:      Operation result
 *----------------------------------------------------------------------------*/

int cmd4_process(int argc, char *argv[])
{
	printf("Command4 is OK!\r\n");
	return 0;
}

/*----------------------------------------------------------------------------**
 * Name:        parse_arguments
 * Description: System parse argument.
 * Parameter:   cmdline - Command line.
 * Return:      None
 *----------------------------------------------------------------------------*/

void parse_arguments(char *cmdline, int *argc, char **argv)
{
	char *ptr;
	int state = 0;
	int i;

	*argc = 0;

	if (strlen(cmdline) == 0)
	{
		return;
	}

	/* convert all tabs into single spaces */
	ptr = cmdline;
	while (*ptr != '\0')
	{
		if (*ptr == '\t')
		{
			*ptr = ' ';
		}
		ptr++;
	}

	ptr = cmdline;
	i = 0;

	/* now find all words on the command line */
	while (*ptr != '\0')
	{
		if (state == 0)
		{
			if (*ptr != ' ')
			{
				argv[i] = ptr; /* Save ptr into argv[i] */
				i++;
				state = 1;
			}
		}
		else
		{
			if (*ptr == ' ')
			{
				*ptr = '\0';
				state = 0;
			}
		}
		ptr++;
	}
	*argc = i;
}

/*----------------------------------------------------------------------------**
 * Name:        parse_command
 * Description: Get command from command line.
 * Parameter:   cmdstr - String of command
 * Return:      None
 *----------------------------------------------------------------------------*/

int parse_command(char *cmdstr)
{
	int i;

	for (i = 0; tblSysCommand[i].cmd != NULL; i++)
	{
		if (strncmp((tblSysCommand[i].cmd), cmdstr, strlen((tblSysCommand[i].cmd))) == 0)
		{
			return i;
		}
	}
	return -1;
}

/*----------------------------------------------------------------------------**
 * Name:        command_handler
 * Description: System command handler.
 * Parameter:   cmdline - Command line string.
 *              max_len - Length of string.
 * Return:      Operation result.
 *----------------------------------------------------------------------------*/

int command_handler(char *cmdline, int max_len)
{
	int comd_index;

	parse_arguments(cmdline, &gwCmdLineArgc, gCmdLineArgv);

	/* only whitespace */
	if (gwCmdLineArgc == 0)
	{
		return 0;
	}

	comd_index = parse_command(gCmdLineArgv[0]);
	if (comd_index < 0)
	{
		printf("Command %s is not supproted!\r\n", gCmdLineArgv[0]);
		return -1;
	}

	if (tblSysCommand[comd_index].proc != NULL)
	{
		tblSysCommand[comd_index].proc(gwCmdLineArgc, gCmdLineArgv);
	}

	return 0;
}

/*******************************************************************************
 *                                End of File
 ******************************************************************************/
