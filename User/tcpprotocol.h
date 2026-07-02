#ifndef TCPCOMMAND_H
#define TCPCOMMAND_H


#define MAX_LEN_MESSAGE 64
#define MAX_NUM_PARAMS  16
#define MAX_LEN_PARAM   16

class TcpProtocol
{
public:
    typedef enum {
        eCmdUnknown = 0,
        eCmdGet,
        eCmdSet,
        eCmdOpen,
        eCmdClose,
        eCmdSend,
        eCmdReceive,
        eCmdMax
    } ECommandType;

    typedef enum {
        eTargetUnknown = 0,
        eTargetDin,
        eTargetDout,
        eTargetAnalog,
        eTargetCAN,
        eTargetCom,
		eTargetTest,
        eTargetMax
    } ETargetType;

    typedef enum {
        eParamUnknown = 0,
        eParamSpeed,
        eParamFilter,
        eParamData,
		eParamBaud,
		eParamAvailable,
		eParamError,
        eParamMax
    } EParamType;

    typedef enum {
        eStateUnknown = 0,
        eStateOn,
        eStateOff,
        eStateOpened,
        eStateClosed,
        eStateError,
        eStateMax
    } EStateType;

public:
    TcpProtocol(void);

    ECommandType getCommand(void) { return command; }
    ETargetType getTarget(void) { return target; }
    int getIdx(void) { return idx; }
    EStateType getState(void) { return state; }
    int getValue(void) { return value; }
    EParamType getParam(void) { return param; }
    int getNParams(void) { return nparams; }
    char *getParams(int idx)
    {
        if (idx < MAX_NUM_PARAMS)
            return params[idx];
        return 0;
    }

protected:
    ECommandType command;
    ETargetType target;
    int idx;
    EParamType param;
    EStateType state;
    long value;
    char message[MAX_LEN_MESSAGE+1];
    int nparams;
    char params[MAX_NUM_PARAMS][MAX_LEN_PARAM+1];

    const char *strCommand[eCmdMax];
    const char *strTarget[eTargetMax];
    const char *strParam[eParamMax];
    const char *strState[eStateMax];
    const char *strSep;
    const char *strTerm;

    void fillVectors(void);
    int verifyToken(char *token, const char *tokens[], int ntokens);
    bool parseParam(char *token);

};

class TcpProtocolMaster : public TcpProtocol
{
public:

    char *toCommand(ECommandType command, ETargetType target, int idx);
    char *toCommand(ECommandType command, ETargetType target, int idx, EStateType state);
    char *toCommand(ECommandType command, ETargetType target, int idx, EParamType param, int nparams, char *params[]);
    bool fromAnswer(char *message);
};

class TcpProtocolSlave : public TcpProtocol
{
public:

    bool fromCommand(char *message);
    char *toAnswer(EStateType state);
    char *toAnswer(int val);
    char *toAnswer(EParamType param, int val);
    char *toAnswer(int nparams, char *params[]);
};

#ifdef STM32F4XX
extern TcpProtocolSlave tcpProtocolSlave;
#endif

#endif // TCPCOMMAND_H
