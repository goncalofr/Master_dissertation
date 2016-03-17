#line 1 "dalf_ext.c"          //work around the __FILE__ screwup on windows, http://www.microchip.com/forums/m746272.aspx
//cannot set breakpoints if this directive is used:
//info: http://www.microchip.com/forums/m105540-print.aspx
//uncomment only when breakpoints are no longer needed
/******************************************************************************
*******************************************************************************
**
**
**                  dalf_ext.c - Generic software extensions and extended 
**                           hardware support for the original Dalf
**                           firmware.
**
**      This module extends the original firmware of the Dalf-1F motor 
**      control board to support generic hardware and software features
**      not included in the original version, that are not exclusive to
**      the T2D module.
**
**      This code was designed originally for the Dalf-1F motor control
**      board, the brain of the T2D module.
**      Original Dalf-1F firmware revision was 1.73.
**      See Dalf-1F owner's manual and the ROVIM T2D documentation for more 
**      details.
**
**          The ROVIM Project
*******************************************************************************
******************************************************************************/

#include "dalf.h"
#include "rovim.h"
#include <string.h>
#include "p18f6722.h"

static BOOL GetGPIOConfigbyName(const rom char* name, IOPinConfig* config);

static BYTE verbosity = VERBOSITY_DISABLED;     //controls the verbosity of the debug information

void (*AckCallback)(void) = NULL;

static BOOL ResourcesLockFlag=FALSE;

static const BYTE StandardCommandsToAllow[]={'C','E','G','H','I','K','L','O','P','R','U','V','X'};
static const BYTE nStandardCommandsToAllow=(BYTE) (sizeof(StandardCommandsToAllow)/sizeof(StandardCommandsToAllow[0]));
//Dalf extended commands ("G #") to block while on lockdown
static const BYTE ExtendedCommandsToAllow[]={0,1,2};
static const BYTE nExtendedCommandsToAllow=(BYTE) (sizeof(ExtendedCommandsToAllow)/sizeof(ExtendedCommandsToAllow[0]));

///////////////////////////////////////////////////////////////////////////////
//Debug reporting features.
///////////////////////////////////////////////////////////////////////////////

void DEBUG_PrintCmd(void)
{
    BYTE i;

    if ((!(verbosity & VERBOSITY_LEVEL_DEBUG)) || (SCFG != TEcfg))
        return;

    DEBUG_MSG("Cmd received: %c,(%d in decimal). # arguments: %d. Arguments: ", CMD, CMD, ARGN);
    for(i=0;i<ARGN;i++)
    {
        printf("%02X, ",ARG[i]);
    }
    printf("\r\n");
}

void SetVerbosity(BYTE level)
{
    verbosity = VERBOSITY_DISABLED;

    if (VERBOSITY_LEVEL_ERROR & level)
        verbosity |= VERBOSITY_LEVEL_ERROR;
    if (VERBOSITY_LEVEL_WARNING & level)
        verbosity |= VERBOSITY_LEVEL_WARNING;
    if (VERBOSITY_LEVEL_STATUS & level)
        verbosity |= VERBOSITY_LEVEL_STATUS;
    if (VERBOSITY_LEVEL_DEBUG & level)
        verbosity |= VERBOSITY_LEVEL_DEBUG;
    if (VERBOSITY_USE_CALL_INFO & level)
        verbosity |= VERBOSITY_USE_CALL_INFO;
    if (VERBOSITY_USE_TIMESTAMP & level)
        verbosity |= VERBOSITY_USE_TIMESTAMP;
}

BYTE GetVerbosity(void)
{
    return verbosity;
}

/*
To convert ticks to ms we divide by 32 instead of 33, to speed the calculation.
This creates an error that maxes at 24ms, or 2.4%. This is an acceptable trade-off
*/
DWORD CalculateDelayMs(PTIME start, PTIME end)
{
    ULONG i=0,j=0;
    if ((start==NULL) || (end==NULL))
    {
        return -1;
    }
    i=TIME_TO_MSEC((*end));
    j=TIME_TO_MSEC((*start));
    if (j > i)
    {
        return -1;
    }
    return (DWORD)(i-j);
}


///////////////////////////////////////////////////////////////////////////////
//Logging features. This module is only compiled if the features are enabled.
///////////////////////////////////////////////////////////////////////////////


#ifdef LOG_ENABLED

void LOG_LogInit(void)
{
    ERROR_MSG("LOG_LogInit not implemented.\r\n");
    return;
}

#endif


///////////////////////////////////////////////////////////////////////////////
//system functions                                                           //
///////////////////////////////////////////////////////////////////////////////

void SystemInitExt(void)
{
    //This has to be the first action of this function!!
    SystemInit();
    //TODO: find out if we just recovered from a hard reset (watchdog) or it's a power-on
    return;
}

#ifdef WATCHDOG_ENABLED

void InitWatchdog(void)
{
    /*We enable only watchdog after system initialization, instead of from power on (On the 
    config register), because when running DEBUG traces, the bootup time will be longer than the
    watchdog timeout, so the system will be constantly rebooting.
    Also, this way I can test this feature using the debugger - much faster and straightforward
    See PIC18F6722 datasheet, Section 25.2 - Watchdog timer for details.*/
    volatile BYTE *wdtcon;
    wdtcon = (volatile BYTE *)0xFD1;
    ((*(volatile BYTE *)wdtcon) = (0x01));
    
}

void KickWatchdog(void)
{
    ClrWdt();
    return;
}

void HardReset(void)
{
    Reset();
}

#endif

/**
\Brief Call the actions required to execute a command
*/
BYTE TeCmdDispatchExt(void)
{
    DEBUG_PrintCmd();
    if( IsStandardCommandLocked(CMD) )
    {
        return eDisable;
    }
    switch(CMD)
    {
        case 'G':
            if (ARGN==0)
            {
                return TeProcessAck();
            }
            if( IsExtendedCommandLocked(ARG[0]) )
            {
                return eDisable;
            }
            switch (ARG[0])
            {
                /*custom functions index:
                this is to be used if we want other custom functions
                other than ROVIM_T2D ones. They must be inserted here, 
                to be parsed before calling the ROVIM dispatch*/
                case 0:
                    return TeProcessAck();
                case 1:
                    return TeDisableAck();
                default:
                    #ifdef DALF_ROVIM_T2D
                    return ROVIM_T2D_CmdDispatch();
                    #else //DALF_ROVIM_T2D
                    return eParseErr
                    #endif //DALF_ROVIM_T2D
            }
            break;
        case 'H':
        #ifdef HELP_ENABLED
            return ShowHelp();
        #else
            return eDisable;
        #endif
            break;
        case 'O':
            #ifdef DALF_ROVIM_T2D
            WARNING_MSG("Motor configurations lost. Make sure to restore them before \
normal operation. You may have to reboot.\r\nYou should use the ROVIM T2D stop motors instead.\r\n");
            #endif //DALF_ROVIM_T2D
            //fall through
        default:
            return TeCmdDispatch();
            break;
    }
    return eParseErr;
}

/**
\Brief Parse the characters from the uart buffer into variables describing a command
*/
BYTE TE_CmdParseExt(void)
{
    return TE_CmdParse();
}

BYTE I2C2CmdDispatchExt(void)
{
    return I2C2CmdDispatch();
}

BYTE TeProcessAck(void)
{
    if(ARGN > 1)
    {
        return eNumArgsErr;
    }

    if(AckCallback != NULL)
    {
        AckCallback();
    }
    //call registered functions to process the ack
    STATUS_MSG("There are no tasks pending user validation to resume.\r\n");
    return NoErr;
}

BYTE TeDisableAck(void)
{
    if(ARGN != 1)
    {
        return eNumArgsErr;
    }
    
    AckCallback=NULL;
    STATUS_MSG("Tasks pending user validation were terminated.\r\n");
}

#ifdef HELP_ENABLED
// Help submodule ----------------------------------------------------------------------------------
BYTE ShowHelp(void)
{
    BYTE i=0;
    #ifdef DALF_ROVIM_T2D
    //temporary, quick help
    MSG("Dalf ROVIM T2D Help.\r\nUsage: H\r\nAvailable help topics:\r\n");
    MSG("\tC\t\t\tRead ADC.\r\n\tE\t\t\tShow encoder count.\r\n\tG[cmd code] [args...]\tCustom \
Commands. See below.\r\n\tH\t\t\tShow this message.\r\n\tI\t\t\tReset software.\r\n\tU\t\t\tShow motor\
 status.\r\n\tV\t\t\tShow motors velocity.\r\n\tX2 [dir] [duty cycle]\tMove direction motor in open \
loop.\r\n[dir] can be: '0' - move to port, or '1' - move to starboard.\r\n\nCustom commands:\r\n");
    MSG("\tG10\t\t\tGo to lockdown.\r\n");
    MSG("\tG11\t\t\tRelease from lockdown, if the system is good to go.\r\n");
    MSG("\tG12 [mtr]\t\tStop motor [mtr]. If [mtr] is omitted, both motors are stopped.\r\n");
    MSG("\tG14 PWM\t\t\tSet the decelerator (braking energy recovery) to [PWM]%%.\r\n");
    MSG("\tG15 PWM\t\t\tSet the accelerator to [PWM]%%.\r\n");
    MSG("\tG16 type speed\t\tSet the vehicle to move in direction [type] at speed [speed].\r\n\
Possible directions are:\r\n0 - Forward;\r\n1 - Reverse;\r\n2 - Neutral (coast);\r\n3 - Hill hold (maintain position).\r\n\
Speed is expressed in tenths of Km/h (km/10/h) and varies between %d and %d.\r\n",0,ROVIM_T2D_MAX_SPEED);
    MSG("\tG17 angle\t\tturn direction to [angle].\r\n\n");
    MSG("Do not forget that all inputs through this interface are in hexadecimal and all outputs of \
the standard commands are also in hexadecimal. All other messages and custom commands outputs are in decimal.\r\n");
    MSG("Example usage:\r\nX2 00 30 - Moves the direction to port at 48%% of full power\r\n\
G16 00 20 - Moves the vehicle forward at ~3.2 Km/h\r\n\G17 1B - Turn the vehicle 16º to starboard.\r\n\n");
    MSG("GPIO pin count on Dalf board J5 and J6 expanders:\r\n(male socket seen from above)\r\n------------------\r\n|1 3 5 ....... 15|\
\r\n|2 4 6 ....... 16|\r\n--------  --------\r\nGPIOs are:\r\n");
    for(i=0;i<ngpios;i++)
    {
        if(strcmppgm("",GPIOsDescription[i].name)!=0)
        {
            MSG("%d - %HS\r\n",(i+1), GPIOsDescription[i].name);
        }
    }
    //True beauty lies in ascii art...
    MSG("\nDirection angle representation (motorcycle seen from above):\r\n            %dº\r\n\
   %dº  \\---|---/ 0º\r\n          \\  |  /  \r\n           \\ | /   \r\n            \\|/\r\n        _ (FRONT) _\r\n\
       |_|---|---|_|\r\n          |     |\r\n  (PORT)  |     |  (STARBOARD)\r\n          |     |\r\n\
        _ |     | _\r\n       |_|-------|_|\r\n           (REAR)  \r\n\n\
The direction angle (the argument for command G17) starts at full starboard - 0º and goes until full port - %dº.\r\n",
 (ROVIM_T2D_DIR_ANGULAR_RANGE/2), ROVIM_T2D_DIR_ANGULAR_RANGE, ROVIM_T2D_DIR_ANGULAR_RANGE);
    MSG("Motor 1 is the traction motor and motor 2 is the direction motor. However, motor 1 must be \
controlled through the custom commands - motor 2 can also be controlled by the standard commands.\r\n");
    MSG("Dalf board led blinking information:\r\nLED1 ON - traction motor is either moving or ready to move through auto mode.\r\n\
LED1 OFF - traction motor is not ready to move with auto mode.\r\nLED2 ON or blinking - direction motor is either moving\
 or being powered.\r\nLED2 OFF - direction motor is not being powered.\r\nLED3 OFF - No errors.\r\n\
LED3 blinking - ROVIM is in lockdown mode.\r\nMTR1 ON green - traction motor is being accelerated\
 on auto mode.\r\nMTR1 ON red - traction motor is being decelerated on auto mode.\r\nMTR2 ON green - direction is moving to port.\r\n\
MTR2 ON red - direction is moving to starboard.\r\nMTR1 and MTR2 intensity depends on the PWM duty cycle.\r\n");
    MSG("For further reference please consult the users manual.\r\n");
    return NoErr;
    #else
    ERROR_MSG("ShowHelp not implemented.\r\n");
    #endif
    return eDisable;
}

void RegisterCmdhelp(void)
{
    ERROR_MSG("RegisterCmdhelp not implemented.\r\n");
    return;
}

#endif

void LockCriticalResourcesAccess(void)
{
    /*given which and how GPIOs are currently used, they do not need special protection. If needed
    in the future, it should be added.*/
    ResourcesLockFlag=TRUE;
    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_LockCriticalResourcesAccess();
    #endif
    return;
}

void UnlockCriticalResourcesAccess(void)
{
    ResourcesLockFlag=FALSE;
    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_UnlockCriticalResourcesAccess();
    #endif
    return;
}

BOOL IsStandardCommandLocked(BYTE cmd)
{
    BYTE i;
    
    if(!ResourcesLockFlag)
    {
        return FALSE;
    }
    
    for(i=0;i<nStandardCommandsToAllow;i++)
    {
        if( cmd==StandardCommandsToAllow[i])
        {
            DEBUG_MSG("Found cmd=%d is in whitelist position %d.\r\n", cmd, i);
            return FALSE;
        }
    }
    ERROR_MSG("Command is locked.\r\n");
    return TRUE;
}

BOOL IsExtendedCommandLocked(BYTE cmd)
{
    BYTE i;
    
    if(!ResourcesLockFlag)
    {
        return FALSE;
    }
    if (cmd >= CUSTOM_CMD_ID_OFFSET)
    {
        //command does not exist in extended command set. nExtendedCommandsToLock[] is incorrect
        DEBUG_MSG("Command does not belong to dalf extended set.\r\n");
        return FALSE;
    }
    
    for(i=0;i<nExtendedCommandsToAllow;i++)
    {
        if( cmd==ExtendedCommandsToAllow[i])
        {
            DEBUG_MSG("Found cmd=%d is in whitelist position %d.\r\n", cmd, i);
            return FALSE;
        }
    }
    DEBUG_MSG("Command is locked.\r\n");
    return TRUE;
}

// GPIO sub-driver ---------------------------------------------------------------------------------
/*Remark: Due to the nature of this application, where all possible GPIOs are known beforehand, 
(since they need to be physically connected) this driver works with a constant set of possible 
GPIOs, while allowing to change each one's configuration. Each GPIO has a default configuration,
that is the mos likely to be use during program operation.*/

//Set a config for an existing GPIO
BOOL SetGPIOConfig(const rom char* name, IOPinConfig* config)
{
    BYTE aux=0, i=0;
    BYTE dir=0;
    BYTE pullup=0;
    BYTE inverted=0;
    //These variables depecting the previous state exist to provide debug information
    BYTE previousDir=0;
    BYTE previousPullup=0;
    BYTE previousInverted=0;
    IOPinConfig defaultConfig={0};
    
    //parameter check
    if(config == NULL)
    {
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    if((config->number > GPIOS_PER_EXPANDER) || (config->number==0))
    {
        ERROR_MSG("GPIO description parameters invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(name == NULL)
    {
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    //Verify the GPIO exists and is coherent with default params
    if(!GetGPIOConfigbyName(name, &defaultConfig))
    {
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        return FALSE;
    }
    //XXX:this should be part of the GPIO id. The config should only the 3 non-constant parameters
    if((defaultConfig.number != config->number) || (defaultConfig.exp != config->exp))
    {
        ERROR_MSG("GPIO description does not match a valid GPIO's. Aborting operation.\r\n");
        return FALSE;
    }
    
    //Get current IOEXP bank config
    if(config->exp == J5){
        previousDir=ReadIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config->number));
        previousInverted=ReadIOExp2(IPOLA + IOEXP_REG_BANK_OFFSET(config->number));
        previousPullup=ReadIOExp2(GPPUA + IOEXP_REG_BANK_OFFSET(config->number));
    }
    else{
        previousDir=ReadIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config->number));
        previousInverted=ReadIOExp1(IPOLA + IOEXP_REG_BANK_OFFSET(config->number));
        previousPullup=ReadIOExp1(GPPUA + IOEXP_REG_BANK_OFFSET(config->number));
    }
    
    //Change the configuration of only the GPIO under treatment
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    CHANGE Nth BIT TO X: number ^= (-x ^ number) & (1 << n);*/
    aux= config->inverted==ON? 0xFF:0;
    inverted = previousInverted ^( (aux ^ previousInverted) & (1U << IOEXP_PIN_BIT_OFFSET(config->number)) );
    aux= config->pullup==ON? 0xFF:0;
    pullup = previousPullup ^( (aux ^ previousPullup) & (1U << IOEXP_PIN_BIT_OFFSET(config->number)) );
    aux= config->dir==IN? 0xFF:0;
    dir = previousDir ^( (aux ^ previousDir) & (1U << IOEXP_PIN_BIT_OFFSET(config->number)) );
    
    //Write the new configuration
    if(config->exp == J5){
        WriteIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config->number), dir);
        WriteIOExp2(IPOLA + IOEXP_REG_BANK_OFFSET(config->number), inverted);
        WriteIOExp2(GPPUA + IOEXP_REG_BANK_OFFSET(config->number), pullup);
    }
    else{
        WriteIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config->number), dir);
        WriteIOExp1(IPOLA + IOEXP_REG_BANK_OFFSET(config->number), inverted);
        WriteIOExp1(GPPUA + IOEXP_REG_BANK_OFFSET(config->number), pullup);
    }
    
    DEBUG_MSG("SetGPIOConfig: exp=J%d, pin=%d, bank=%c, bank bit=%d. Previous: dir=%08b, \
inv=%08b, pull=%08b. Desired: dir=%01b, inv=%01b, pull=%01b. Current: dir=%08b, inv=%08b, pull=%08b.\r\n", \
  (config->exp==J5)?5:6, config->number, ('A'+IOEXP_REG_BANK_OFFSET(config->number)),\
(IOEXP_PIN_BIT_OFFSET(config->number)+1), previousDir, previousInverted, previousPullup, \
(config->dir==IN? 1:0), (config->inverted==ON? 1:0),(config->pullup==ON? 1:0), dir, inverted, pullup);

    /* Uncomment this section if you want to check intermediate calculations
    DEBUG_MSG("SetGPIOConfig: pin=%d, bank offset=%d, bit offset=%d. Previous: dir=%08b, inv=%08b, pull=%08b\
. Intermediate calculations (for dir only): aux=%08b, aux^previousDir=%08b, 1U<<offset=%08b, (aux^previousDir) & (1U<<offset)\
=%08b. Current: dir=%08b, inv=%08b, pull=%08b.\r\n", config->number, IOEXP_REG_BANK_OFFSET(config->number),\
IOEXP_PIN_BIT_OFFSET(config->number), previousDir, previousInverted, previousPullup, aux, (aux ^ previousDir),\
 (1U << IOEXP_PIN_BIT_OFFSET(config->number)), \
 ((aux ^ previousDir) & (1U << IOEXP_PIN_BIT_OFFSET(config->number))), dir, inverted, \
 pullup);*/
 
    return TRUE;
}

//both parameters must point to valid variables. Space won't be allocated here
BOOL GetGPIOConfig(const rom char* name, IOPinConfig* config)
{
    BYTE dir=0, pullup=0, inverted=0, aux=0, i=0;
    
    if(config == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(!GetGPIOConfigbyName(name, config)){
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        config=NULL;
        return FALSE;
    }
    
    //read the IOEXP bank configuration
    if(config->exp == J5){
        dir=ReadIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config->number));
        inverted=ReadIOExp2(IPOLA + IOEXP_REG_BANK_OFFSET(config->number));
        pullup=ReadIOExp2(GPPUA + IOEXP_REG_BANK_OFFSET(config->number));
    }
    else{
        dir=ReadIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config->number));
        inverted=ReadIOExp1(IPOLA + IOEXP_REG_BANK_OFFSET(config->number));
        pullup=ReadIOExp1(GPPUA + IOEXP_REG_BANK_OFFSET(config->number));
    }
    
    //get and translate the configuration for the GPIO under treatment
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    CHECK: bit = (number >> x) & 1;*/
    aux= (dir >> IOEXP_PIN_BIT_OFFSET(config->number)) & 1U;
    config->dir= aux? IN: OUT;
    aux= (inverted >> IOEXP_PIN_BIT_OFFSET(config->number)) & 1U;
    config->inverted= aux? ON: OFF;
    aux= (pullup >> IOEXP_PIN_BIT_OFFSET(config->number)) & 1U;
    config->pullup= aux? ON: OFF;
    
    DEBUG_MSG("GetGPIOConfig: exp=J%d, pin=%d, bank=%c, bank bit=%d. dir=%08b,%d, inv=%08b,%d, pull=%08b,\
%d.\r\n",  (config->exp==J5)?5:6, config->number, ('A'+IOEXP_REG_BANK_OFFSET(config->number)),\
    (IOEXP_PIN_BIT_OFFSET(config->number)+1), dir, config->dir, inverted, config->inverted,\
    pullup, config->pullup);
    return TRUE;
}

BOOL CompareGPIOConfig(IOPinConfig* config1, IOPinConfig* config2)
{
    if(config1 == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    if(config2 == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    if((config1->number > GPIOS_PER_EXPANDER) || (config1->number==0))
    {
        ERROR_MSG("GPIO config parameters invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if((config2->number > GPIOS_PER_EXPANDER) || (config2->number==0))
    {
        ERROR_MSG("GPIO config parameters invalid. Aborting operation.\r\n");
        return FALSE;
    }
    
    DEBUG_MSG("CompareGPIOConfig: 1:exp=J%d,number=%d,dir=%d,pullup=%d,inverted=%d;  2:exp=J%d,\
number=%d,dir=%d,pullup=%d,inverted=%d\r\n",(config1->exp==J5)?5:6, config1->number,config1->dir,\
config1->pullup,config1->inverted, (config2->exp==J5)?5:6, config2->number,config2->dir,config2->pullup,config2->inverted);

    if( memcmp( (const void *)config1, (const void *)config2, sizeof(IOPinConfig)))
    {
        return FALSE; //configurations do not match
    }
    
    return TRUE;
}

//String constants are automatically stored in rom. See MPLAB C-18 Users guide, 2.73.
/*this function searches the list of Registered GPIO names (set during initialization) for a
match to the name provided. If it is found, it returns the GPIO's corresponding configuration,
If not, an error occurs.*/
//This is an sub-driver private function. Parameter check is done before calling it.
static BOOL GetGPIOConfigbyName(const rom char* name, IOPinConfig* config)
{
    BYTE i=0;
    
    if(config == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    
    for(i=0; i<ngpios; i++){
        //Do NOT use strncmppgm, since it requires the 3rd argument to be in rom and fails silently if it isn't correctly provided!
        if(strcmppgm(name, GPIOsDescription[i].name) == 0){
            memcpypgm2ram(config,(const rom void *) &GPIOsDescription[i].config,sizeof(*config));
            //DEBUG_MSG("GetGPIOConfigbyName: matching name found for '%HS'.\r\n", name); //%HS prints a string located in far rom
            return TRUE;
        }
    }

    ERROR_MSG("GetGPIOConfigbyName could not find a matching name for '%HS'.\r\n",name);
    config=NULL;
    return FALSE;
}

//"Rule of thumb: Always read inputs from PORTx and write outputs to LATx. If you need to read what you set an output to, read LATx."
BOOL SetGPIO(const rom char* name)
{
    BYTE i=0;
    BYTE previousValue=0, value;
    BYTE dir=0;
    IOPinConfig config;

    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(!GetGPIOConfigbyName(name, &config)){
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        return FALSE;
    }  
    //Check if the GPIO is configured as input
    if(config.exp == J5){
        dir=ReadIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        dir=ReadIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    dir = (dir >> IOEXP_PIN_BIT_OFFSET(config.number)) & 1U;
    if(dir){
        ERROR_MSG("GPIO configured as input. Aborting operation.\r\n");
        return FALSE;
    }

    //Get current IOEXP bank gpios
    if(config.exp == J5){
        previousValue=ReadIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        previousValue=ReadIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }

    //Change the configuration of only the GPIO under treatment
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    SET: number |= 1 << x;*/
    value = previousValue | (1U << IOEXP_PIN_BIT_OFFSET(config.number));
    
    //Write the new value only if needed
    if(previousValue!=value)
    {
        if(config.exp == J5){
            WriteIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
        }
        else{
            WriteIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
        }
    }
    else
    {
        /*DEBUG_MSG("GPIO value already matches pretended value. No action needed.\r\n");*/
    }
    
    DEBUG_MSG("SetGPIO: exp=J%d, pin=%d, bank %c, bank bit=%d. Previous bit value=%b, current \
bit value=%b.\r\n", (config.exp==J5)?5:6, config.number, ('A' + IOEXP_REG_BANK_OFFSET(config.number)),\
    (IOEXP_PIN_BIT_OFFSET(config.number) +1), previousValue, value);
    return TRUE;
}

BOOL ResetGPIO(const rom char* name)
{
    BYTE i=0;
    BYTE previousValue=0, value=0;
    BYTE dir=0;
    IOPinConfig config;

    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(!GetGPIOConfigbyName(name, &config)){
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        return FALSE;
    }
    //Check if the GPIO is configured as input
    if(config.exp == J5){
        dir=ReadIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        dir=ReadIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    dir = (dir >> IOEXP_PIN_BIT_OFFSET(config.number)) & 1U;
    if(dir){
        ERROR_MSG("GPIO configured as input. Aborting operation.\r\n");
        return FALSE;
    }
    
    //Get current IOEXP bank gpios
    if(config.exp == J5){
        previousValue=ReadIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        previousValue=ReadIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    
    //Change the configuration of only the GPIO under treatment
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    RESET: number &= ~(1 << x);*/
    value = previousValue & (~(1U << IOEXP_PIN_BIT_OFFSET(config.number)));
    
    //Write the new value only if needed
    if(previousValue!=value)
    {
        if(config.exp == J5){
            WriteIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
        }
        else{
            WriteIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
        }
    }
    else
    {
        /*DEBUG_MSG("GPIO value already matches pretended value. No action needed.\r\n");*/
    }

    DEBUG_MSG("ResetGPIO: exp=J%d, pin=%d, bank %c, bank bit=%d. Previous bit value=%b, current \
bit value=%b.\r\n", (config.exp==J5)?5:6, config.number, ('A' + IOEXP_REG_BANK_OFFSET(config.number)),\
    (IOEXP_PIN_BIT_OFFSET(config.number) +1), previousValue, value);
    return TRUE;
}

BOOL ToggleGPIO(const rom char* name)
{
    BYTE i=0;
    BYTE previousValue=0, value=0;
    BYTE dir=0;
    IOPinConfig config;

    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(!GetGPIOConfigbyName(name, &config)){
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        return FALSE;
    }
    //Check if the GPIO is configured as input
    if(config.exp == J5){
        dir=ReadIOExp2(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        dir=ReadIOExp1(IODIRA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    dir = (dir >> IOEXP_PIN_BIT_OFFSET(config.number)) & 1U;
    if(dir){
        ERROR_MSG("GPIO configured as input. Aborting operation.\r\n");
        return FALSE;
    }
    
    //Get current IOEXP bank gpios
    if(config.exp == J5){
        previousValue=ReadIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        previousValue=ReadIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    TOGGLE: number ^= 1 << x;*/
    value = previousValue ^ (1U << IOEXP_PIN_BIT_OFFSET(config.number));
    
    //Write the new value
    if(config.exp == J5){
        WriteIOExp2(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
    }
    else{
        WriteIOExp1(OLATA + IOEXP_REG_BANK_OFFSET(config.number), value);
    }

    DEBUG_MSG("ToggleGPIO: exp=J%d, pin=%d, bank %c, bank bit=%d. Previous bit value=%b, current \
bit value=%b.\r\n", (config.exp==J5)?5:6, config.number, ('A' + IOEXP_REG_BANK_OFFSET(config.number)),\
    (IOEXP_PIN_BIT_OFFSET(config.number) +1), previousValue, value);
    return TRUE;
}

BOOL GetGPIO(const rom char* name, BYTE* value)
{
    BYTE i=0;
    IOPinConfig config;
    BYTE aux=0;
    
    if(name == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    for(i=0;i<IO_PIN_NAME_MAX_LEN;i++)
    {
        if(name[i] == '\0') break;
    }
    if((i==0) || (i==IO_PIN_NAME_MAX_LEN))
    {
        ERROR_MSG("GPIO name invalid. Aborting operation.\r\n");
        return FALSE;
    }
    if(!GetGPIOConfigbyName(name, &config)){
        ERROR_MSG("GPIO does not exist. Aborting operation.\r\n");
        return FALSE;
    }
    if(value == NULL){
        ERROR_MSG("Received NULL input parameter. Aborting operation.\r\n");
        return FALSE;
    }
    
    if(config.exp == J5){
        aux=ReadIOExp2(GPIOA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    else{
        aux=ReadIOExp1(GPIOA + IOEXP_REG_BANK_OFFSET(config.number));
    }
    
    /* For further info on these calculations see:
    http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    CHECK: bit = (number >> x) & 1;*/
    *value= (aux>> IOEXP_PIN_BIT_OFFSET(config.number)) & 1U;

    DEBUG_MSG("GetGPIO: exp=J%d, pin=%d, bank %c, bank bit=%d. Value=%d.\r\n", (config.exp==J5)?5:6,\
    config.number, ('A' + IOEXP_REG_BANK_OFFSET(config.number)),\
    (IOEXP_PIN_BIT_OFFSET(config.number) +1), *value);
    return TRUE;
}

BOOL GetAllGPIO(BYTE* J5A, BYTE* J5B, BYTE* J6A, BYTE* J6B)
{
    *J5A=ReadIOExp2(GPIOA);
    *J6A=ReadIOExp1(GPIOA);
    *J5B=ReadIOExp2(GPIOB);
    *J6B=ReadIOExp1(GPIOB);
    
    return TRUE;
}
