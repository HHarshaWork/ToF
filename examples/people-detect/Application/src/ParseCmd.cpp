/**********************************************************
 * Parses Command Line parameters
 **********************************************************/
 /*******************************************************************************
  @file :ParseCmd.cpp

  @brief :Utility functions to parse command line input

  *********************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "ParseArgs.h"
#include "ParseCmd.h"

#ifndef MAX
#define MAX(x,y) (((x) > (y))? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y) (((x) < (y))? (x) : (y))
#endif


  /**
   * This variable stores a list of argument vs what it maps to and the default values
   */
char *keylist =
		"i   | camera | Input file, "
		"o   | ""    | Output file,"
		"n   | -1    | num of frames,"
        "th  | 30    | seconds after object appears after which to track,"
        "d   | 30.0  | threshold in cm for distance moved after which new object is counted,"
        "cm  | 0    | capture mode 0:1MP(1024x1024) 1:QMP(512x512),"
	    "md  | metadata.csv | Metadata output file name,"
        "sp  | COM7  | Serial port name,"
        "sf  | 1     | Frequency in frames to send serial comm,"
        "off | 100   | Height offset in cm to be added to detections to compensate for diff between TOF camera and beamformer,"
        "dm  | 1     | Display mode,"
	    "em  | 0     | Enable Meta Data out,";


/**
 * Initializes the COM port based on name given as input
 *
 * @param argc - Num of arguments
 * @param argv - Array of arguments
 * @return 1 -  Success , 0 Failure 
 */
int CommandParser::Parse(int argc, char **argv)
{
	int result;

    if (CmdParserAddKeys(keylist) != 0)
    {
		printf("Error: CmdLineParser : Some mandatory arguments are missing\n");
        return 1;
    }

    if (CmdParserParseArgv(argc, argv) != 0)
        return 1;

	char* parseString;
    result = CmdParserGetArg("i",
                             &parseString,
                             CMD_PARSER_ARG_TYPE_CHAR_PTR,
                             0);
	InFile.assign(parseString);

    char* OutparseString = "";
    result = CmdParserGetArg("o",
                             &OutparseString,
                             CMD_PARSER_ARG_TYPE_CHAR_PTR,
                             0);

    if (strcmp(OutparseString,"") == 0)
    {
        bEnableOutputFileWrite = 0;
    }
    else
    {
        bEnableOutputFileWrite = 1;
        OutFile.assign(OutparseString);
    }

    result = CmdParserGetArg("n",
                             &nNumFrms,
                             CMD_PARSER_ARG_TYPE_INT,
                             0);

    result = CmdParserGetArg("th",
                            &nTimeAfterTracked,
                            CMD_PARSER_ARG_TYPE_INT,
                            0);

    result = CmdParserGetArg("d",
                            &nDistanceThresholdTrack,
                            CMD_PARSER_ARG_TYPE_FLOAT,
                            0);
    nDistanceThresholdTrack = nDistanceThresholdTrack * 10;//convert cm to mm
	    	
    result = CmdParserGetArg("em",
        &bEnableMetaDataOutput,
        CMD_PARSER_ARG_TYPE_INT,
        0);

    char* csvString = "";
    result = CmdParserGetArg("md",
                             &csvString,
                             CMD_PARSER_ARG_TYPE_CHAR_PTR,
                             0);
    if (strcmp(csvString, "") == 0)
    {
        if (bEnableMetaDataOutput == 1)
        {
            szCsvFileName.assign(OutFile);
            szCsvFileName.append(".csv");
        }
    }
    else
    {
        szCsvFileName.assign(csvString);
    }

    char* comString;
    result = CmdParserGetArg("sp",
                            &comString,
                            CMD_PARSER_ARG_TYPE_CHAR_PTR,
                            0);
    ComPort.assign(comString);

    result = CmdParserGetArg("cm",
                            &nCaptureMode,
                            CMD_PARSER_ARG_TYPE_INT,
                            0);

    result = CmdParserGetArg("off",
        &nHeightOffset,
        CMD_PARSER_ARG_TYPE_INT,
        0);

    //boundary check 
    nHeightOffset = MAX(0, nHeightOffset);
    nHeightOffset = MIN(500, nHeightOffset);

    result = CmdParserGetArg("dm",
        &nDisplayMode,
        CMD_PARSER_ARG_TYPE_INT,
        0);

    if (nDisplayMode < E_DISPLAY_IR || nDisplayMode > E_DISPLAY_SILHOUETTE)
    {
        nDisplayMode = E_DISPLAY_DEPTH;
    }   

    result = CmdParserGetArg("sf",
        &nSerialCommFrameRate,
        CMD_PARSER_ARG_TYPE_INT,
        0);
    //Boundary check for params
    if (nSerialCommFrameRate <= 0 || nSerialCommFrameRate > 10000)
    {
        nSerialCommFrameRate = 1;
    }

    return CMDLINEPARSER_SUCCESS;
}
