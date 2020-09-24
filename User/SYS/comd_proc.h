/*******************************************************************************
 * File Name:    comd_proc.h
 * Created by:   Hawk Ji
 * Created Date: 2012-09-02
 * Version:      1.0
 *
 * Description:  Include file of utilities program.
 ******************************************************************************/

#ifndef  __COMD_PROC_H__
#define  __COMD_PROC_H__

#ifdef __COMMAND_FUNC_EX__
#define _ex_process0
#define _ex_process1
#define _ex_process2
#define _ex_process3
#define _ex_process4
#define _ex_process5
#endif

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Command handler related Macro definition.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define	MAX_COMMAND_LENGTH      256
#define	MAX_ARGS_SIZE           (MAX_COMMAND_LENGTH / 8)

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
 * Function prototype
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int command_handler(char *cmdline, int cmd_len);

#endif /* End of __COMD_PROC_H__ */
