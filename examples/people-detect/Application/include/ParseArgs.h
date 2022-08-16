/*
 ******************************************************************************
 Copyright (c) 2014 - Analog Devices Inc. All Rights Reserved.
 This software is proprietary & confidential to Analog Devices, Inc.
 and its licensors.
 ******************************************************************************
 $Revision: 20509 $
 $Date: 2017-01-27 20:23:51 +0530 (Fri, 27 Jan 2017) $

 Title: Command Line Parser

 Description:

 Routines for parsing command line arguments

 *****************************************************************************/
#ifndef __PARSE_ARGS_H__
#define __PARSE_ARGS_H__

#include <stdio.h>

/* Maximum number of keys*/
#define MAX_NUM_KEYS          100
/* Maximum length of the key*/
#define MAX_KEY_LENGTH        128
/* Maximum length of help message*/
#define MAX_MESSAGE_LENGTH    512
/* Maximum numbe rof occurance of one key in command line*/
#define MAX_NUM_OCCURANCE      100
/* Maximum length of one argument */
#define MAX_ARG_LENGTH         256
/* Maximum length of entire command line arguments.. relevant only for param file parsing*/
#define MAX_CMD_LENGTH         1024

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

typedef enum
{
    CMD_PARSER_ARG_TYPE_CHAR_PTR,
    CMD_PARSER_ARG_TYPE_INT,
	CMD_PARSER_ARG_TYPE_SHORT,
	CMD_PARSER_ARG_TYPE_INT8,
    CMD_PARSER_ARG_TYPE_FLOAT,
    CMD_PARSER_ARG_TYPE_STRING,
	CMD_PARSER_ARG_TYPE_BOOL
} CMD_PARSER_ARG_TYPE;

/*
 **  Public Function Prototype section
 */

void CmdParserPrintUsage(void);
int  CmdParserAddKeys(char *szKeyList);
int  CmdParserParseArgv(int argc, char *argv[]);
int  CmdParserParseCmdFile(FILE *fp, char *pStartID, char *pComment);
int  CmdParserParseCmdString(char *pCmdStr);
int  CmdParserGetArg(char *key, void *dst, CMD_PARSER_ARG_TYPE type,int occurence);
void CmdParserClearArgs(void);
void CmdParserClearAll(void);
int  CmdParserGetNumOccurance(char *szKey);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PARSE_ARGS_H__ */


