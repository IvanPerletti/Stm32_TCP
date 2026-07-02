#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "tcpprotocol.h"

static const char *strNull = "";

static const char *strCommandGet = "GET";
static const char *strCommandSet = "SET";
static const char *strCommandOpen = "OPEN";
static const char *strCommandClose = "CLOSE";
static const char *strCommandSend = "SEND";
static const char *strCommandReceive = "RECEIVE";

static const char *strTargetDin = "DIN";
static const char *strTargetDout = "DOUT";
static const char *strTargetAnalog = "ANALOG";
static const char *strTargetCAN = "CAN";
static const char *strTargetCom = "COM";
static const char *strTargetTest = "TEST";

static const char *strParamSpeed = "SPEED";
static const char *strParamFilter = "FILTER";
static const char *strParamData = "DATA";
static const char *strParamBaud = "BAUD";
static const char *strParamAvailable = "AVAILABLE";
static const char *strParamError = "ERROR";

static const char *strStateOn = "ON";
static const char *strStateOff = "OFF";
static const char *strStateOpened = "OPENED";
static const char *strStateClosed = "CLOSED";
static const char *strStateError = "ERROR";

#ifdef STM32F4XX
TcpProtocolSlave tcpProtocolSlave;
#endif

TcpProtocol::TcpProtocol(void)
{
    fillVectors();
}

void TcpProtocol::fillVectors(void)
{
    strCommand[eCmdUnknown] = strNull;
    strCommand[eCmdGet] = strCommandGet;
    strCommand[eCmdSet] = strCommandSet;
    strCommand[eCmdOpen] = strCommandOpen;
    strCommand[eCmdClose] = strCommandClose;
    strCommand[eCmdSend] = strCommandSend;
    strCommand[eCmdReceive] = strCommandReceive;

    strTarget[eTargetUnknown] = strNull;
    strTarget[eTargetDin] = strTargetDin;
    strTarget[eTargetDout] = strTargetDout;
    strTarget[eTargetAnalog] = strTargetAnalog;
    strTarget[eTargetCAN] = strTargetCAN;
    strTarget[eTargetCom] = strTargetCom;
    strTarget[eTargetTest] = strTargetTest;

    strParam[eParamUnknown] = strNull;
    strParam[eParamSpeed] = strParamSpeed;
    strParam[eParamFilter] = strParamFilter;
    strParam[eParamData] = strParamData;
    strParam[eParamBaud] = strParamBaud;
    strParam[eParamAvailable] = strParamAvailable;
    strParam[eParamError] = strParamError;

    strState[eStateUnknown] = strNull;
    strState[eStateOn] = strStateOn;
    strState[eStateOff] = strStateOff;
    strState[eStateOpened] = strStateOpened;
    strState[eStateClosed] = strStateClosed;
    strState[eStateError] = strStateError;

    strSep = " ";
    strTerm = "\n";
}

int TcpProtocol::verifyToken(char *token, const char *tokens[], int ntokens)
{
    int ind;

    for (ind=0; ind<ntokens; ind++)
    {
        if (strcmp(tokens[ind], token) == 0)
            return ind;
    }
    return 0;
}

bool TcpProtocol::parseParam(char *token)
{
    nparams=0;
    for (int ind=0; ind<MAX_NUM_PARAMS; ind++)
    {
        if ((token = strtok(NULL, (const char *)" \n")) != NULL)
        {
            if (strlen(token) <= MAX_LEN_PARAM)
                strcpy(params[nparams++], token);
            else
                break;
        }
        else
            break;
    }

    return (nparams > 0);
}

/* -----------------------------------------------------------------------------------*/
// TcpProtocolMaster
/* -----------------------------------------------------------------------------------*/

char *TcpProtocolMaster::toCommand(ECommandType command, ETargetType target, int idx)
{
    char str[10];

    strcpy(message, strCommand[command]);
    strcat(message, strSep);
    strcat(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strTerm);

    return message;
}

char *TcpProtocolMaster::toCommand(ECommandType command, ETargetType target, int idx, EStateType state)
{
    char str[10];

    strcpy(message, strCommand[command]);
    strcat(message, strSep);
    strcat(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strSep);
    strcat(message, strState[state]);
    strcat(message, strTerm);

    return message;
}

char *TcpProtocolMaster::toCommand(ECommandType command, ETargetType target, int idx, EParamType param, int nparams, char *params[])
{
    if (nparams <= MAX_NUM_PARAMS)
    {
        char str[MAX_LEN_PARAM+1];

        strcpy(message, strCommand[command]);
        strcat(message, strSep);
        strcat(message, strTarget[target]);
        strcat(message, strSep);
        snprintf(str, sizeof(str), "%d", idx);
        strcat(message, str);
        strcat(message, strSep);
        strcat(message, strParam[param]);
        strcat(message, strSep);
        for (int ind=0; ind<nparams; ind++)
        {
            if (strlen(params[ind]) <= MAX_LEN_PARAM)
            {
                strcat(message, params[ind]);
                if (ind < (nparams - 1))
                    strcat(message, strSep);
            }
        }
        strcat(message, strTerm);

        return message;
    }
    return NULL;
}

bool TcpProtocolMaster::fromAnswer(char *message)
{
    bool ret = false;
    char *token;

    if ((token = strtok(message, (const char *)" ")) != NULL)
    {
        if ((target = (ETargetType)verifyToken(token, strTarget, (int)eTargetMax)) > 0)
        {
            if ((token = strtok(NULL, (const char *)" ")) != NULL)
            {
                if ((idx = atoi(token)) > 0)
                {
                    if ((token = strtok(NULL, (const char *)" \n")) != NULL)
                    {
                        if (target == eTargetDin || target == eTargetDout)
                        {
                            if ((state = (EStateType)verifyToken(token, strState, (int)eStateMax)) > 0)
                                ret = true;
                        }
                        else if (target == eTargetAnalog)
                        {
                            value = strtol(token, NULL, 10);
                            ret = true;
                        }
                        else if (target == eTargetCAN)
                        {
                            if ((param = (EParamType)verifyToken(token, strParam, (int)eParamMax)) > 0)
                            {
                                if (param == eParamData)
                                    ret = parseParam(token);
                            }
                        }
                    }
                }
            }
        }
    }

    return ret;
}

/* -----------------------------------------------------------------------------------*/
// TcpProtocolSlave
/* -----------------------------------------------------------------------------------*/

bool TcpProtocolSlave::fromCommand(char *message)
{
    bool ret = false;
    char *token;

    if ((token = strtok(message, (const char *)" ")) != NULL)
    {
        if ((command = (ECommandType)verifyToken(token, strCommand, (int)eCmdMax)) > 0)
        {
            if ((token = strtok(NULL, (const char *)" ")) != NULL)
            {
                if ((target = (ETargetType)verifyToken(token, strTarget, (int)eTargetMax)) > 0)
                {
                    if ((token = strtok(NULL, (const char *)" \n")) != NULL)
                    {
                        if ((idx = atoi(token)) > 0)
                        {
                            switch (command)
                            {
                            case eCmdGet:
                                ret = (target == eTargetDin || target == eTargetDout || target == eTargetAnalog);
                                break;
                            case eCmdSet:
                                if ((token = strtok(NULL, (const char *)" \n")) != NULL)
                                {
                                    if (target == eTargetDin || target == eTargetDout)
                                    {
                                        if ((state = (EStateType)verifyToken(token, strState, (int)eStateMax)) > 0)
                                            ret = true;
                                    }
                                    else if (target == eTargetAnalog)
                                    {
                                        value = strtol(token, NULL, 10);
                                        ret = true;
                                    }
                                    else if (target == eTargetCAN)
                                    {
                                        if ((param = (EParamType)verifyToken(token, strParam, (int)eParamMax)) == eParamFilter || param == eParamSpeed)
                                            ret = parseParam(token);
                                    }
                                    else if (target == eTargetCom){
                                    	if ((param = (EParamType)verifyToken(token, strParam, (int)eParamMax)) == eParamBaud )
                                    	    ret = parseParam(token);
                                    }
                                }
                                break;
                            case eCmdOpen:
                            case eCmdClose:
                                ret = (target == eTargetCAN || target == eTargetCom);
                                break;
                            case eCmdSend:
                            	if( target == eTargetTest){
                            		ret = true;
                            	}
                            	else if ( target == eTargetCom ){
                            		ret = parseParam(token);
                            	}
                            	else if ((token = strtok(NULL, (const char *)" \n")) != NULL){
                            		if ((param = (EParamType)verifyToken(token, strParam, (int)eParamMax)) == eParamData)
                            			ret = parseParam(token);
                                }

                                break;
                            case eCmdReceive:
                                if ((token = strtok(NULL, (const char *)" \n")) != NULL)
                                                {
                                    if ((param = (EParamType)verifyToken(token, strParam, (int)eParamMax)) == eParamData)
                                                        ret = true;
                                                    }
                                                        break;
                            default:
                                break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
    }

    return ret;
}

char *TcpProtocolSlave::toAnswer(EStateType state)
{
    char str[10];

    strcpy(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strSep);
    strcat(message, strState[state]);
    strcat(message, strTerm);
    return message;
}

char *TcpProtocolSlave::toAnswer(int val)
{
    char str[20];

    strcpy(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", val);
    strcat(message, str);
    strcat(message, strTerm);
    return message;
}

char *TcpProtocolSlave::toAnswer(EParamType param, int val)
{
    char str[20];

    strcpy(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strSep);
    strcat(message, strParam[param]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", val);
    strcat(message, str);
    strcat(message, strTerm);
    return message;
}

char *TcpProtocolSlave::toAnswer(int nparams, char *params[])
{
    if (nparams <= MAX_NUM_PARAMS)
{
        char str[MAX_LEN_PARAM+1];

    strcpy(message, strTarget[target]);
    strcat(message, strSep);
    snprintf(str, sizeof(str), "%d", idx);
    strcat(message, str);
    strcat(message, strSep);
    strcat(message, strParam[param]);
    strcat(message, strSep);
        for (int ind=0; ind<nparams; ind++)
        {
            if (strlen(params[ind]) <= MAX_LEN_PARAM)
            {
                strcat(message, params[ind]);
                if (ind < (nparams - 1))
                    strcat(message, strSep);
            }
        }
    strcat(message, strTerm);
    return message;
    }
    return NULL;
}
