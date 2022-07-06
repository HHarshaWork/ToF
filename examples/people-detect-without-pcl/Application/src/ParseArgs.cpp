/*
 ******************************************************************************
 Copyright (c) 2014 - 2022  Analog Devices Inc. All Rights Reserved.
 This software is proprietary & confidential to Analog Devices, Inc.
 and its licensors.
 ******************************************************************************
 Title: Command Line Parser

 Description:

 Routines for parsing command line arguments

 *****************************************************************************/

/*============= I N C L U D E S =============*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "ParseArgs.h"

/*============= D E F I N E S =============*/

typedef struct
{
    char szKey[MAX_KEY_LENGTH];
    char szHelp[MAX_MESSAGE_LENGTH];
    char szDefault[MAX_ARG_LENGTH];
    char szArg[MAX_NUM_OCCURANCE][MAX_ARG_LENGTH];
    int nOccurance;
} CMD_ARG_INFO;

/*============= D A T A =============*/

static CMD_ARG_INFO gCmdInfo[MAX_NUM_KEYS];
static int gnNumKeys = 0;
static char *szProgramName = NULL;
static char szCmdBuf[MAX_CMD_LENGTH];

/*============= C O D E  =============*/

/*
 **  Function Prototype section
 */
static char CopyTokens(char *pDst, char **pSrc, char cTok, bool bRemoveSpace);
static int CheckMandatoryArgs(void);
static void RemoveTrailingSpaces(char *pSrc);
static void RemoveLeadingSpaces(char *pSrc);
/*
 **  Function Definition section
 */

/**
 * Parses the arguments in argv  and populate gCmdInfo structure.
 *
 * @param argc - Number of command line arguments
 * @param argv - Pointer to arguments
 * @return 0 -  Success , 1 Failure (Some mandatory arguments
 * are missing)
 */
int CmdParserParseArgv(int argc, char *argv[])
{
    int i, nKey, nResult;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
    int nOccurance;

    pCmdInfo[gnNumKeys].szArg[0][0] = '\0';
    pCmdInfo[gnNumKeys].nOccurance = 0;

	szProgramName = argv[0];

    for (i = 1; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {

            for (nKey = 0; nKey < gnNumKeys; nKey++)
            {
                if (strcmp(&argv[i][1], pCmdInfo[nKey].szKey) == 0)
                {
                    nOccurance = pCmdInfo[nKey].nOccurance++;
					if (argc <= (i + 1))
					{
						printf("Warning! Ignoring option %s  as no value is given\n", argv[i] );
					}
					else
					{
						strncpy(&pCmdInfo[nKey].szArg[nOccurance][0], argv[i + 1], MAX_ARG_LENGTH);
						i = i + 1;
					}
                    break;
                }
            }
            if (nKey == gnNumKeys)
            {
                printf("Ignoring the unknown argument %s ...\n", argv[i]);
            }
        }
    }

    nResult = CheckMandatoryArgs();

    return nResult;
}

/**
 * Parses param file and populate gCmdInfo structure
 *
 * @param fp  -  Pointer to param file - Assumes it is already opened
 * @param pProgramName - Program name (can be NULL)
 * @param pComment - Comment charector (can be NULL)
 * @return
 */
int CmdParserParseCmdFile(FILE *fp, char *pStartID, char *pComment)
{
    int nResult;
    char *pArgStr;
    char szArg[MAX_KEY_LENGTH];
    char *pCmdBuf = &szCmdBuf[0];
    char cTok;
    bool bCmdFound = false;
    char *pCmdStr;

    /* Loop till we get a command string */
    do
    {
        pArgStr = fgets(pCmdBuf, MAX_CMD_LENGTH, fp);
        pCmdStr = pArgStr;
        if (pArgStr == NULL)
        {
            break;
        }

        cTok = CopyTokens(szArg, &pArgStr, ' ', true);
		if(cTok == '\0')
		{
			/* Blank line */
			continue;
		}
        if (pComment != NULL)
        {
            if (szArg[0] == pComment[0])
            {
                /* This line is a comment..go to next line */
                continue;
            }
        }

        if (pStartID != NULL)
        {
            if (strcmp(szArg, pStartID) != 0)
            {
                /* Consider the line only if it starts with predefined string*/
                continue;
            }
        }

        bCmdFound = true;
    } while (bCmdFound == false);

    if (bCmdFound == true)
    {
        nResult = CmdParserParseCmdString(pCmdStr);
    }
    else
    {
        /* If no command are found , check whether there is any mandatory argument*/
        /* nResult = CheckMandatoryArgs() */
        /* Current VAT codes expect an error for this. so time being return an error */
        nResult  =  1;
    }

    return nResult;

}
/**
 * Parses the command line string
 * @param pCmdStr -  Command line string
 * @return 0 -  Success 1 - Failed.
 */
int CmdParserParseCmdString(char *pCmdStr)
{
    int nResult;
    char cTok;
    char *pCmdStrCurr = pCmdStr;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
    int nOccurance;
    char szArg[MAX_KEY_LENGTH];
    int nKey;

    cTok = CopyTokens(szArg, &pCmdStrCurr, ' ', true);
	
    /* Loop all arguments on this line */
    while (cTok != '\0')
    {
        if (szArg[0] == '-')
        {
            for (nKey = 0; nKey < gnNumKeys; nKey++)
            {
                if (strcmp(&szArg[1], pCmdInfo[nKey].szKey) == 0)
                {
                    nOccurance = pCmdInfo[nKey].nOccurance++;
                    //cTok = CopyTokens(szArg, &pCmdStrCurr, ' ', true);
                    strcpy(&pCmdInfo[nKey].szArg[nOccurance][0], szArg);
                    break;
                }
            }
            if (nKey == gnNumKeys)
            {
                printf("Ignoring the unknown argument %s ...\n", szArg);
                //cTok = CopyTokens(szArg, &pCmdStrCurr, ' ', true);
            }
        }
        cTok = CopyTokens(szArg, &pCmdStrCurr, ' ', true);
    }

    nResult = CheckMandatoryArgs();

    return nResult;
}

/**
 *  Checks whether all mandatory arguments are present
 * @return  0 -  Success , all are present , 1 - Failed
 */
int CheckMandatoryArgs()
{
    int nKey;
    int nResult = 0;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
	int i;

    for (nKey = 0; nKey < gnNumKeys; nKey++)
    {
        if (pCmdInfo[nKey].szArg[0][0] == '\0')
        {
            if (pCmdInfo[nKey].szDefault[0] == '@')
            {
				printf("\nMissing mandatory option  -%s \n\n", pCmdInfo[nKey].szKey);
                CmdParserPrintUsage();
                nResult = 1;
                break;
            }
			if (pCmdInfo[nKey].szDefault[0] == '$')
			{
				nResult = 1;
				for (i = 0; i < gnNumKeys; i++)
				{
					if (strcmp(pCmdInfo[nKey].szDefault, pCmdInfo[i].szDefault) == 0)
					{
						if (pCmdInfo[i].szArg[0][0] != '\0')
						{
							nResult = 0;
							break;
						}
					}
				}

				if (nResult != 0)
				{
					printf("\nMissing mandatory option. One of the following options is mandatory \n ");
					printf("-%s ", pCmdInfo[nKey].szKey);
					for (i = nKey +  1; i < gnNumKeys; i++)
					{
						if(strcmp(pCmdInfo[nKey].szDefault, pCmdInfo[i].szDefault) == 0)
						{
							printf("-%s ",pCmdInfo[i].szKey);
						}	
					}
					printf("\n\n");
					CmdParserPrintUsage();
					break;
				}
			}
        }
    }

    return nResult;

}

/**
 *  Adds list of keys to master keylist
 *
 * @param szKeyList -  String pointing to the list of keys
 *   Example:
 *  "a | 0     |Calculate fractional format automatically ,"
 *  "b | 0.0   |This value is subtracted from rho  before converting, "
 *  "c | adi_pd_SVMhyperplane.c      | Name of the c file."
 *  "f |  @     |Number of fractional bits for hyperplane";

 * @return 0 -  Keys are successfully added , 1 -Failed to add keys
 */
int CmdParserAddKeys(char *szKeyList)
{
    int nResult = 0;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
    char *szKeyListCurr = szKeyList;
    char cTok;

    CmdParserClearAll();

    do
    {
        cTok = CopyTokens(pCmdInfo[gnNumKeys].szKey, &szKeyListCurr, '|', true);
        if (strlen(pCmdInfo[gnNumKeys].szKey) > MAX_KEY_LENGTH)
        {
            printf("Length of key is more than supported %d\n", MAX_KEY_LENGTH);
            nResult = 1;
            break;
        }
        cTok = CopyTokens(pCmdInfo[gnNumKeys].szDefault,
                          &szKeyListCurr,
                          '|',
                          true);
        if (strlen(pCmdInfo[gnNumKeys].szDefault) > MAX_ARG_LENGTH)
        {
            printf("Length of default value is more than supported %d\n",
                   MAX_ARG_LENGTH);
            nResult = 1;
            break;
        }

        cTok = CopyTokens(pCmdInfo[gnNumKeys].szHelp,
                          &szKeyListCurr,
                          ',',
                          false);
        if (strlen(pCmdInfo[gnNumKeys].szHelp) > MAX_MESSAGE_LENGTH)
        {
            printf("Length of help message is more than supported %d\n",
                   MAX_MESSAGE_LENGTH);
            nResult = 1;
            break;
        }

		RemoveTrailingSpaces(pCmdInfo[gnNumKeys].szHelp);
		RemoveLeadingSpaces(pCmdInfo[gnNumKeys].szHelp);

        gnNumKeys++;

        if (gnNumKeys > MAX_NUM_KEYS)
        {
            printf("Number of argument keys is more than supported %d\n",
                   MAX_NUM_KEYS);
            nResult = 1;
            break;
        }

    } while (cTok != '\0');

    return nResult;
}

/**
 *  Copies the argument to pDst
 *
 * @param szKey - The argument key
 * @param pDst  - Pointer to the location where argument should be copied.
 * @param eType - Type of the argument
 * @param nOccurance - Number of occurrence of argument. '0' to indicate
 * first occurrence.
 * @return 0 -  Success , 1 Failure
 */
int CmdParserGetArg(char *szKey, void *pDst, CMD_PARSER_ARG_TYPE eType,
                    int nOccurance)
{
    int i;
    int nResult = 1;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
    char *szArg;

    for (i = 0; i < gnNumKeys; i++)
    {
        if (strcmp(szKey, pCmdInfo->szKey) == 0)
        {
            nResult = 0;
            szArg = NULL;
            if (pCmdInfo->szArg[nOccurance][0] != '\0')
            {
                szArg = &pCmdInfo->szArg[nOccurance][0];
            }
            else if ((pCmdInfo->szDefault[0] != '\0')
				&& (pCmdInfo->szDefault[0] != '@') && (pCmdInfo->szDefault[0] != '$'))
            {
                szArg = pCmdInfo->szDefault;
            }

            if (szArg != NULL)
            {
                switch (eType)
                {
                case CMD_PARSER_ARG_TYPE_CHAR_PTR:
                    *(char **) pDst = szArg;
                    break;
                case CMD_PARSER_ARG_TYPE_INT:
                    *(int *) pDst = atoi(szArg);
                    break;
                case CMD_PARSER_ARG_TYPE_SHORT:
                    *(short *) pDst = atoi(szArg);
                    break;
				case CMD_PARSER_ARG_TYPE_INT8:
                    *(char *) pDst = atoi(szArg);
                    break;
                case CMD_PARSER_ARG_TYPE_FLOAT:
                    *(float *) pDst = (float) atof(szArg);
                    break;
				case CMD_PARSER_ARG_TYPE_BOOL:
					*(bool *)pDst = (bool)(atoi(szArg) == 0) ? false : true;
                    break;
                case CMD_PARSER_ARG_TYPE_STRING:
                    strcpy((char*)pDst, szArg);
                    break;
                default:
                    nResult = 1;
                    break;
                }
            }
            else
            {
                nResult = 1;
            }
			break;
        }
        pCmdInfo++;
    }

    return nResult;
}

/**
* Get numbe rof occurance of a particular argument
*
* @param szKey - The argument key
* @return Number of occurance for the arument, -1 if szKey doesnt match
*
*/
int CmdParserGetNumOccurance(char *szKey)
{
	int i;
	int nOut = -1;
	CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
	

	for (i = 0; i < gnNumKeys; i++)
	{
		if (strcmp(szKey, pCmdInfo->szKey) == 0)
		{
			nOut = pCmdInfo->nOccurance;
			break;
		}
		pCmdInfo++;
	}

	return nOut;
}

/**
 * Prints the help message for usage.
 *
 */
void CmdParserPrintUsage()
{
    int i,j;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];
	bool bPrintInfo = true;

	if (szProgramName != NULL)
	{
		printf("Usage:\n");
		printf("%s  [options]\n\n", szProgramName);
	}
    printf("Supported arguments/options are\n");

	/* First print all mandatory argument */
	for (i = 0; i < gnNumKeys; i++)
	{
		if (pCmdInfo[i].szDefault[0] == '@')
		{
			if (bPrintInfo == true)
			{
				//If we keep this printf outiside loop it will print even if there are no arguments 
				//satisfying this criteria
				printf("\nFollowing arguments are mandatory \n\n");
				bPrintInfo = false;
			}
			printf("-%-4s: %s\n", pCmdInfo[i].szKey, pCmdInfo[i].szHelp);
		}
	}

	/* Print mandatory argument which are in groups */
	bPrintInfo = true;
	for (i = 0; i < gnNumKeys; i++)
	{
		if (pCmdInfo[i].szDefault[0] == '$')
		{
			if (bPrintInfo == true)
			{
				//If we keep this printf outiside loop it will print even if there are no arguments 
				//satisfying this criteria
				printf("\nOne of the following arguments are mandatory in each group seprated by blank line\n");
				bPrintInfo = false;
			}

			for (j = 0; j < gnNumKeys; j++)
			{
				if (strcmp(pCmdInfo[i].szDefault, pCmdInfo[j].szDefault) == 0)
				{
					if (j < i)
					{
						//already printed
						break;
					}
					if (j == i)
					{
						printf("\n");
					}
					printf("-%-4s: %s\n", pCmdInfo[j].szKey, pCmdInfo[j].szHelp); 
				}
			}
			
		}
	}

	bPrintInfo = true;
	for (i = 0; i < gnNumKeys; i++)
	{
		if ((pCmdInfo[i].szDefault[0] != '@') && (pCmdInfo[i].szDefault[0] != '\0') && (pCmdInfo[i].szDefault[0] != '$'))
		{
			if (bPrintInfo == true)
			{
				printf("\nFollowing arguments are optional. Default values are given in brackets in the end \n\n");
				bPrintInfo = false;
			}

			printf("-%-4s: %s ", pCmdInfo[i].szKey, pCmdInfo[i].szHelp);
			printf("(%s)\n", pCmdInfo[i].szDefault);
		}
	}
	bPrintInfo = true;

	for (i = 0; i < gnNumKeys; i++)
	{
		if (pCmdInfo[i].szDefault[0] == '\0')
		{
			if (bPrintInfo == true)
			{
				printf("\nFollowing arguments are optional. Corresponding feature is disabled if not given \n\n");
				bPrintInfo = false;
			}

			printf("-%-4s: %s\n", pCmdInfo[i].szKey, pCmdInfo[i].szHelp);
		}
	}

}
/**
 * Copies the token from string *ppSrc to pDst till the 'cTok'
 * character is encountered. Optionally the spaces are ignored while copying
 * The pDst is suffixed with '\0' after copying the characters. *ppSrc
 * is updated to new position
 *
 * Empty words are not supported by separator space
 *
 * @param pDst  - Pointer to destination character array
 * @param ppSrc - Pointer to source pointer
 * @param cTok -  Separator token
 * @param bRemoveSpace - If true ignores space while copying
 * @return The separator that caused to exit. It can 'tok or '\0'
 */
char CopyTokens(char *pDst, char **ppSrc, char cTok, bool bRemoveSpace)
{
    char *pSrc = *ppSrc;
    bool bDone = false;
    char nResult;
	bool bWordStarted = false;

    while ((*pSrc != '\0') && (bDone == false))
    {
		/* Empty words are not supported by separator space*/
		if ((*pSrc == cTok) && ((bWordStarted == true) ||(cTok != ' ')))
        {
            bDone = true;
            break;
        }
        else if ((*pSrc == ' ') && (bRemoveSpace == true))
        {
            //ignore space;
            pSrc++;
        }
        else
        {
			bWordStarted = true;
            *pDst++ = *pSrc++;
        }
    }

	//remove trailing new line
	while (*(pDst-1) == '\n')
	{
		pDst--;
	}

    nResult = *pSrc++;
    *ppSrc = pSrc;
    *pDst = '\0';

    return nResult;
}
/**
 * Clears the argument list
 */
void CmdParserClearArgs()
{
    int i, j;
    CMD_ARG_INFO *pCmdInfo = &gCmdInfo[0];

    for (i = 0; i < gnNumKeys; i++)
    {
        for (j = 0; j < pCmdInfo->nOccurance; j++)
        {
            pCmdInfo->szArg[j][0] = '\0';
        }
        pCmdInfo->nOccurance = 0;
        pCmdInfo++;
    }

}
/**
 * Clears all buffers used by command parser
 * including the keys
 */
void CmdParserClearAll()
{
    gnNumKeys = 0;

    memset(&gCmdInfo[0], 0, sizeof(gCmdInfo));

    szProgramName = NULL;

    memset(&szCmdBuf[0], 0, sizeof(szCmdBuf));
}

/*
* Removes trailing spaces in astring
* @param ppSrc - Pointer to source string
*/
void RemoveTrailingSpaces(char *pSrc)
{
	int nLength = (int)strlen(pSrc);

	while (pSrc[nLength - 1] == ' ')
	{
		nLength--;
	}

	pSrc[nLength] = '\0';
}

/*
* Removes leading spaces in a string
* @param ppSrc - Pointer to source string
*/
void RemoveLeadingSpaces(char *pSrc)
{
	int nStart = 0;
	int nLength = (int)strlen(pSrc);
	while (pSrc[nStart] == ' ')
	{
		nStart++;
	}
	//strcpy may not support when buffers are overlapping
	if (nStart > 0)
	{
		nLength -= nStart;
		memcpy(pSrc, &pSrc[nStart], nLength);
		pSrc[nLength] = '\0';
	}

}

