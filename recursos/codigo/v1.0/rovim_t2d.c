#line 1 "rovim_t2d.c"         //work around the __FILE__ screwup on windows, http://www.microchip.com/forums/m746272.aspx
//cannot set breakpoints if this directive is used:
//info: http://www.microchip.com/forums/m105540-print.aspx
//uncomment only when breakpoints are no longer needed
/******************************************************************************
*******************************************************************************
**
**
**                  rovim_t2d.c - "tracção, travagem e direcção" (T2D) 
**                          module of the ROVIM project.
**
**      This module builds on and extends the firmware of the Dalf-1F motor 
**      control board to implement the T2D module of the ROVIM project.
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

#include "p18f6722.h"
#include "rovim.h"
#include "dalf.h"
#include <stdio.h>
#include <string.h>

#pragma romdata DefaultGPIOsDescription
/* Since MCC18 cannot create RAM objects larger than 256 bytes, we must create a specific section
for this data structure. This was done in ROM, since this is only read-only data.
Also, storing the GPIO name in ROM adds a lot of convenience when calling driver functions*/
rom const IOPinDescription DefaultGPIOsDescription[]={
    /*name,                             exp, number,   dir,   pullup, inverted*/
    /*controls progressive braking on SigmaDrive, when on auto mode. Used as PWM switch to feed 
analogue input. This should be used to slow down the vehicle during regular operation.*/
    { "decelerator",                    { J5,    1,      OUT,    OFF,   OFF }},
    /*detects when electric brake has reached unclamp side end-of-travel switch. Logic '1' indicates
 the electric brake is fully unclamped.*/
    { "brake unclamp switch",           { J5,    2,      IN,     OFF,   ON  }},
    /*controls accelerator on SigmaDrive, when on auto mode. Used as PWM switch to feed analogue input.*/
    { "accelerator",                    { J5,    3,      OUT,    OFF,   OFF }},
    /*detects when the SigmaDrive B+ pin is being powered. Logic '1' indicates the SigmaDrive 
is turned on and the fuse and contactor are OK.*/
    { "traction voltage sensor",        { J5,    4,      IN,     OFF,   OFF }},
    /*controls SigmaDrive foot switch, when on auto mode. Active low: logic '0' means the switch is 
pressed and the vehicle is ready to move.*/
    { "activate traction",              { J5,    5,      OUT,    OFF,   OFF }},
    /*detects when electric brake has reached clamp side end-of-travel switch. Logic '1' indicates 
the electric brake is fully clamped and a lockdown cause (emergency switch, dead man trigger, 
etc.) is active.*/
    { "brake clamp switch",             { J5,    6,      IN,     OFF,   OFF }},
    /*controls electric brake unclamp. To properly unclamp electric brake, this signal has to be 
active until unclamp side end-of-travel is reached.*/
    { "brake unclamper",                { J5,    7,      OUT,    OFF,   OFF }},
    /*detects when direction position reaches either of the side end-of-travel switches. Logic '1' 
means end-of-travel is switched on and direction is on an erroneous state.*/
    { "direction error switch",         { J5,    8,     IN,     OFF,   OFF  }},
    /*controls reverse switch on SigmaDrive, when on auto mode. Active low: logic '0' means the 
 switch is pressed and reverse drive is engaged. This switch cannot be active simultaneously with 
 "engage forward", pin 11.*/
    { "engage reverse",                 { J5,    9,      OUT,    OFF,   OFF }},
    /*detects when there is a command to unclamp the brake, either by sw or hw. This may regardless 
off the brake clamp/unclamp switches state. Logic '1' indicates there is such a command.*/
    { "brake unclamp command",          { J5,    10,     IN,     OFF,   OFF }},
    /*controls forward switch on SigmaDrive, when on auto mode. Active low: logic '0' means the 
 switch is pressed and forward drive is engaged. This switch cannot be active simultaneously with 
 "engage reverse", pin 9.*/
    { "engage forward",                 { J5,    11,     OUT,    OFF,   OFF }},
    /*detects when manual/auto switch is on auto mode. When auto mode is engaged, the
 control signals to the SigmaDrive controller must be provided by this program. Logic '1' means auto
 mode is engaged*/
    { "auto mode switch",               { J5,    12,     IN,     OFF,   OFF }},
    /*controls electric brake clamping. To brake electric brake, only a pulse from this signal is 
needed.*/
    { "brake clamper",                  { J5,    13,     OUT,    OFF,   OFF }},
    /*detects an emergency stop condition, regarless of the source. Logic '1' indicates the switch 
is pressed and the brake is trying to clamp.*/
    { "emergency stop condition",       { J5,    14,     IN,     OFF,   OFF }},
    /*controls traction controller handbrake feature. Logic '1' means the handbrake is engaged and
the vehicle will hold its position.*/
    { "engage handbrake",               { J5,    15,     OUT,    OFF,   OFF }},
    /*Unused*/
    { "",                               { J5,    16,     IN,     OFF,   OFF }},
};

const rom BYTE nDefaultgpios=(BYTE) (sizeof(DefaultGPIOsDescription)/sizeof(DefaultGPIOsDescription[0]));
#pragma romdata //resume rom data allocation on the standard section

static BOOL ResourcesLockFlag=FALSE;
static const BYTE ROVIM_T2D_CommandsToAllow[]={
    ROVIM_T2D_LOCKDOWN_CMD_CODE,
    ROVIM_T2D_RELEASE_CMD_CODE,
    ROVIM_T2D_SOFTSTOP_CMD_CODE,
    ROVIM_T2D_ACCELERATE_CMD_CODE,
    ROVIM_T2D_DECELERATE_CMD_CODE,
    ROVIM_T2D_SET_MOVEMENT_CMD_CODE,
    ROVIM_T2D_DEBUG_CTRL_CMD_CODE
    };
static const BYTE ROVIM_T2D_nCommandsToAllow=(BYTE) (sizeof(ROVIM_T2D_CommandsToAllow)/sizeof(ROVIM_T2D_CommandsToAllow[0]));

WORD    ROVIM_T2D_sysmonitorcount;            // ROVIM T2D system state monitoring timeout counter;
WORD    ROVIM_T2D_pwmrefreshcount;            // ROVIM T2D PWM refresh timeout counter;
BOOL    ManualSysMonitoring=FALSE;
BYTE    DebugPWM=0;

//The pointer to easily switch configurations
rom const IOPinDescription* GPIOsDescription = DefaultGPIOsDescription;
BYTE ngpios=0;

//Duty cycle for the traction accelerador and decelerator 
static BYTE AccDutyCycle=0;
static BYTE DccDutyCycle=0;
static unsigned int WaitForAccDccDecay=0;
static BYTE PeriodCnt=0;

static BOOL ResetPrintFlags=FALSE;

//Brake locl/unlock flags
static BOOL UnlockingBrake=FALSE;
BOOL inLockdown=FALSE;
BOOL autoMode=FALSE;
BOOL SigmaDError=FALSE;
static WORD MotorStressMonitor[3]={0,0,0};

//vehicle movement description
static long settlingTime=0;
static movement desiredMovement={0};
BYTE movementType=HILLHOLD;
long /*acc1=0,*/vel1=0;
BOOL ForcePrintMsg=FALSE;

#if 0
static const BYTE SpeedToDutyCycleNoLoadLUTSigmaDConf1[50]=
/*Speed (Km/10/h):*/
/*duty cycle (%):*/{0,};
static const BYTE *SpeedToDutyCycle;
static const BYTE SpeedToDutyCycleLen=0;
#endif
static BYTE SpeedDCScaling=12;  //equals to 1
    
//configure basic ROVIM features needed early on. To be called as soon as possible
void ROVIM_T2D_Init(void)
{
    ROVIM_T2D_sysmonitorcount=-1; //"disable" IO exp sampling for now
    SetVerbosity (INIT_VERBOSITY_LEVEL);
    ROVIM_T2D_ConfigSerialPort();
    ROVIM_T2D_ConfigGPIOs();
    ROVIM_T2D_LockUnusedResourcesAccess();
    //Initial values for the GPIOS will be set on the lockdown version
    ROVIM_T2D_ConfigDefaultParamBlock();

    DEBUG_MSG("ROVIM T2D initialization complete.\r\n");
    return;
}

void ROVIM_T2D_LockUnusedResourcesAccess(void)
{
    ERROR_MSG("LockUnusedResourcesAccess not implemented. Don't worry about it.\r\n");
}

void ROVIM_T2D_ConfigSerialPort(void)
{
    BYTE nBR=0;
    /*If the serial port isn't configured with the correct baud rate, configure it now and reboot, to
    force new configuration. This is only needed when the default parameter block is restored.*/
    nBR = ReadExtEE_Byte(0x006D);
    if(nBR != ROVIM_T2D_NBR)
    {
        WARNING_MSG("Configuring dalf baud rate parameter, nBR, to %d (see dalf owner's manual \
for details). If you're reading this, you have to reconfigure the terminal emulator to the new \
baud rate.\r\n", ROVIM_T2D_NBR);
        WriteExtEE_Byte(0x006D,ROVIM_T2D_NBR);
        Reset();
    }
    STATUS_MSG("Dalf baud rate parameter set to %d. If you can read this properly, you don't \
need to worry about it nor the gibberish printed above, if any.\r\n",ROVIM_T2D_NBR);
}

void ROVIM_T2D_ConfigDefaultParamBlock(void)
{
    ROVIM_T2D_ConfigDirParamBlock();
    
    //Traction encoder set up
    //set TPR1 LSB
    DEBUG_MSG("Setting TPR1 to %d ticks/rev. LSB=0x%02X, MSB=0x%02X.\r\n",ROVIM_T2D_TRACTION_TPR, ROVIM_T2D_TRACTION_TPR, 0);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x40;
    ARG[3]=ROVIM_T2D_TRACTION_TPR;
    TeCmdDispatchExt();
    //set TPR1 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x41;
    ARG[3]=0;
    TeCmdDispatchExt();
    //set VSP1
    DEBUG_MSG("Setting VSP1 to %d ms.\r\n",ROVIM_T2D_VSP1);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x37;
    ARG[3]=ROVIM_T2D_VSP1;
    TeCmdDispatchExt();
    //TODO: VBCAL
    //set VBWARN LSB
    DEBUG_MSG("Setting VBWARN to %d mV. LSB=0x%02X, MSB=0x%02X.\r\n",ROVIM_T2D_VBWARN, (BYTE) (ROVIM_T2D_VBWARN>>8), (BYTE) (ROVIM_T2D_VBWARN & 0xFF));
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x29;
    ARG[3]=(BYTE) (ROVIM_T2D_VBWARN>>8);
    TeCmdDispatchExt();
    //set VBWARN MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x2A;
    ARG[3]=(BYTE) (ROVIM_T2D_VBWARN & 0xFF);
    TeCmdDispatchExt();
    
    /*//XXX: temp.
    WARNING_MSG("For debug purposes, setting VBWARN to a low value. Remove when you're finished debugging\r\n");
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x29;
    ARG[3]=0;
    TeCmdDispatchExt();
    //set VBWARN MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x2A;
    ARG[3]=0;
    TeCmdDispatchExt();*/
}

void ROVIM_T2D_ConfigDirParamBlock(void)
{
    //set MAXERR LSB
    DEBUG_MSG("Setting MAXERR to 0x%02X. LSB=0x%02X, MSB=0x%02X.\r\n",
    (ROVIM_T2D_DIR_TICK_UPPER_LIMIT-ROVIM_T2D_DIR_TICK_LOWER_LIMIT), 
    (ROVIM_T2D_DIR_TICK_UPPER_LIMIT-ROVIM_T2D_DIR_TICK_LOWER_LIMIT), 0);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x2C;
    ARG[3]=(ROVIM_T2D_DIR_TICK_UPPER_LIMIT-ROVIM_T2D_DIR_TICK_LOWER_LIMIT);
    TeCmdDispatchExt();
    //set MAXERR MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x2D;
    ARG[3]=0;
    TeCmdDispatchExt();
    //set MTR2_MODE1
    DEBUG_MSG("Setting MTR2_MODE1 to 0x%02X.\r\n",ROVIM_T2D_DIR_MODE1);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x46;
    ARG[3]=ROVIM_T2D_DIR_MODE1;
    TeCmdDispatchExt();
    //set MTR2_MODE2
    DEBUG_MSG("Setting MTR2_MODE2 to 0x%02X.\r\n",ROVIM_T2D_DIR_MODE2);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x47;
    ARG[3]=ROVIM_T2D_DIR_MODE2;
    TeCmdDispatchExt();
    //set MTR2_MODE3
    DEBUG_MSG("Setting MTR2_MODE3 to 0x%02X.\r\n",ROVIM_T2D_DIR_MODE3);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x48;
    ARG[3]=ROVIM_T2D_DIR_MODE3;
    TeCmdDispatchExt();
    //set ACC2 LSB
    DEBUG_MSG("Setting ACC2 to %d ticks/VSP2^2. LSB=0x%02X, MSB=0x%02X.\r\n",ACC2, (BYTE) (ACC2>>8), (BYTE) (ACC2 & 0xFF));
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x49;
    ARG[3]=(BYTE) (ACC2 & 0xFF);
    TeCmdDispatchExt();
    //set ACC2 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x4A;
    ARG[3]=(BYTE) (ACC2>>8);
    TeCmdDispatchExt();
    //set VMID2 LSB
    DEBUG_MSG("Setting VMID2 to %d ticks/VSP2. LSB=0x%02X, MSB=0x%02X.\r\n",VMID2, (BYTE) (VMID2>>8), (BYTE) (VMID2 & 0xFF));
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x4B;
    ARG[3]=(BYTE) (VMID2 & 0xFF);
    TeCmdDispatchExt();
    //set VMID2 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x4C ;
    ARG[3]=(BYTE) (VMID2>>8);
    TeCmdDispatchExt();
    //set VSP2
    DEBUG_MSG("Setting VSP2 to %d ms.\r\n",ROVIM_T2D_DIR_VSP);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x4D;
    ARG[3]=ROVIM_T2D_DIR_VSP;
    TeCmdDispatchExt();
    //set VMIN2
    DEBUG_MSG("Setting VMIN2 to %d %%.\r\n",ROVIM_T2D_DIR_MIN_PWM);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x54;
    ARG[3]=ROVIM_T2D_DIR_MIN_PWM;
    TeCmdDispatchExt();
    //set VMAX2
    DEBUG_MSG("Setting VMAX2 to %d %%.\r\n",ROVIM_T2D_DIR_MAX_PWM);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x55;
    ARG[3]=ROVIM_T2D_DIR_MAX_PWM;
    TeCmdDispatchExt();
    //set TPR2 LSB
    DEBUG_MSG("Setting TPR2 to %d ticks/rev. LSB=0x%02X, MSB=0x%02X.\r\n",256, 0, 1);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x56;
    ARG[3]=0x00;    //According to Dalf OM, TRP must be set to 0x100 for analog encoders
    TeCmdDispatchExt();
    //set TPR2 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x57;
    ARG[3]=0x01;    //According to Dalf OM, TRP must be set to 0x100 for analog encoders
    TeCmdDispatchExt();
    //set MIN2 LSB
    DEBUG_MSG("Setting MIN2 to 0x%2X. LSB=0x%02X, MSB=0x%02X.\r\n",ROVIM_T2D_DIR_TICK_LOWER_LIMIT, ROVIM_T2D_DIR_TICK_LOWER_LIMIT, 0);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x58;
    ARG[3]=ROVIM_T2D_DIR_TICK_LOWER_LIMIT;
    TeCmdDispatchExt();
    //set MIN2 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x59;
    ARG[3]=0;
    TeCmdDispatchExt();
    //set MAX2 LSB
    DEBUG_MSG("Setting MAX2 to 0x%2X. LSB=0x%02X, MSB=0x%02X.\r\n",ROVIM_T2D_DIR_TICK_UPPER_LIMIT, ROVIM_T2D_DIR_TICK_UPPER_LIMIT, 0);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x5A;
    ARG[3]=ROVIM_T2D_DIR_TICK_UPPER_LIMIT;
    TeCmdDispatchExt();
    //set MAX2 MSB
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x5B;
    ARG[3]=0;
    TeCmdDispatchExt();
    //set DMAX
    DEBUG_MSG("Setting DMAX to 0x%02X.\r\n",ROVIM_T2D_DMAX);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x75;
    ARG[3]=ROVIM_T2D_DMAX;
    TeCmdDispatchExt();
    //set FENBL
    DEBUG_MSG("Setting FENBL to 0x%02X.\r\n",ROVIM_T2D_FENBL);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x76;
    ARG[3]=ROVIM_T2D_FENBL;
    TeCmdDispatchExt();
    //set DECAY
    DEBUG_MSG("Setting DECAY to 0x%02X [If ADC sample time params are default: Fc=-ln(DECAY/256)*220/2/pi].\r\n",ROVIM_T2D_DECAY);
    CMD='W';
    ARGN=4;
    ARG[0]=1;
    ARG[1]=01;
    ARG[2]=0x77;
    ARG[3]=ROVIM_T2D_DECAY;
    TeCmdDispatchExt();
}

//Start all remaining ROVIM features.
void ROVIM_T2D_Start(void)
{
    ROVIM_T2D_sysmonitorcount = ROVIM_T2D_SYSTEM_MONITOR_PERIOD;
    ROVIM_T2D_pwmrefreshcount = ROVIM_T2D_PWM_REFRESH_PERIOD;
}

void ROVIM_T2D_ConfigGPIOs(void)
{
    BYTE i=0;
    IOPinConfig config={0};
    
    GPIOsDescription = DefaultGPIOsDescription;
    ngpios= nDefaultgpios;

    for (i=0; i<ngpios; i++) {
        //copy configuration to data ram before calling setup function
        if(strcmppgm("",GPIOsDescription[i].name)==0)
        {
            DEBUG_MSG("GPIO number %d unused.\r\n",GPIOsDescription[i].config.number);
            continue;
        }
        memcpypgm2ram(&config,(const rom void *) &GPIOsDescription[i].config,sizeof(config));
        SetGPIOConfig(GPIOsDescription[i].name,&config);
    }
    
    return;
}

void ROVIM_T2D_Greeting(void)
{
    if(SCFG == TEcfg) 
    { // If Terminal Emulator Interface
        Greeting();
        printf("ROVIM T2D Brain\r\n");
        printf("ROVIM T2D Software Ver:%2u.%u\r\n",ROVIM_T2D_SW_MAJOR_ID, ROVIM_T2D_SW_MINOR_ID);   // ROVIM Software ID
        printf("ROVIM T2D Release date: "ROVIM_T2D_RELEASE_DATE"\r\n");                              // ROVIM Software release date
        printf("ROVIM T2D Contact(s):\r\n"ROVIM_T2D_CONTACTS"\r\n");                                // ROVIM Contacts
        printf("\r\n");
    }
}

BYTE ROVIM_T2D_CmdDispatch(void)
{
    if( ROVIM_T2D_IsCommandLocked(ARG[0]) )
    {
        return eDisable;
    }
    ResetPrintFlags=TRUE;
    switch(ARG[0])
    {
        case ROVIM_T2D_LOCKDOWN_CMD_CODE:
            return ROVIM_T2D_Lockdown();
        case ROVIM_T2D_RELEASE_CMD_CODE:
            return ROVIM_T2D_ReleaseFromLockdown();
        case ROVIM_T2D_SOFTSTOP_CMD_CODE:
            return ROVIM_T2D_SoftStop();
        case ROVIM_T2D_CONTROL_GPIO_CMD_CODE:
            return ROVIM_T2D_ControlGPIO();
        case ROVIM_T2D_ACCELERATE_CMD_CODE:
            return ROVIM_T2D_Accelerate();
        case ROVIM_T2D_DECELERATE_CMD_CODE:
            return ROVIM_T2D_Decelerate();
        case ROVIM_T2D_SET_MOVEMENT_CMD_CODE:
            return ROVIM_T2D_SetMovement();
        case ROVIM_T2D_TURN_CMD_CODE:
            return ROVIM_T2D_Turn();
        case ROVIM_T2D_DEBUG_CTRL_CMD_CODE:
            return ROVIM_T2D_DebugControl();
        //Add more ROVIM commands here
        default:
            ERROR_MSG("Command does not exist.\r\n");
            return eParseErr;
    }
    ResetPrintFlags=FALSE;
    ERROR_MSG("Unexpected program execution.\r\n");
    return eDisable;
}

void ROVIM_T2D_LockCriticalResourcesAccess(void)
{
    ResourcesLockFlag=TRUE;
    return;
}

void ROVIM_T2D_UnlockCriticalResourcesAccess(void)
{
    ResourcesLockFlag=FALSE;
    return;
}

BOOL ROVIM_T2D_IsCommandLocked(BYTE cmd)
{
    BYTE i;
    
    if(!ResourcesLockFlag)
    {
        return FALSE;
    }
    
    for(i=0;i<ROVIM_T2D_nCommandsToAllow;i++)
    {
        if( cmd==ROVIM_T2D_CommandsToAllow[i])
        {
            DEBUG_MSG("Found cmd=%d is in whitelist position %d.\r\n", cmd, i);
            return FALSE;
        }
    }
    ERROR_MSG("Command is locked.\r\n");
    return TRUE;
}

BOOL ROVIM_T2D_FinishReleaseFromLockdown(void)
{
    UnlockCriticalResourcesAccess();
    //ROVIM_T2D_ConfigDefaultParamBlock(); //see if with soft stop this isn't needed
    ResetGPIO("brake unclamper");
    inLockdown=FALSE;
    LedErr &= ~Lckmsk;
    STATUS_MSG("ROVIM is now ready to move. Restart traction controller to clear any remaining \
error\r\n");
    
    return TRUE;
}

BOOL ROVIM_T2D_LockBrake(void)
{
    DEBUG_MSG("Locking emergency brake now.\r\n");
    SetGPIO("brake clamper");
    ResetGPIO("brake unclamper");
    
    return TRUE;
}

BOOL ROVIM_T2D_UnlockBrake(void)
{   
    //Uncomment this message when braking is done automatically
    //DEBUG_MSG("Unlocking brake now.\r\n");
    ResetGPIO("brake clamper");
    /*Depending on hw configuration, the unclamp pin may be unwired, so force this action to be done
    manually. Regardless of that, the unlock procedure is the same*/
    SetGPIO("brake unclamper");

    return TRUE;
}

//my birthday's june 16th. I have an amazon wishlist. No pressure, though...
BOOL ROVIM_T2D_ValidateState(void)
{
    BYTE directionError=0;
    BYTE emergencyStop=0;
    WORD delay=0;
    TIME now;
    //Since the system always goes to lockdown on power up, this initial value is accurate enough
    static TIME before={0};
    TIME EmergencyConditionTestStart;

    
    /*I'm choosing not to monitor the brake clamp & unclamp & unclamp command here because:
    the user may start unclamping before sending the command; while unclamping, there is nothing 
    I can do to stop that; after unclamping, if there is still a related (with these 3 GPIOs) 
    error condition, it will be picked up by the monitor task*/
    
    GetGPIO("direction error switch",&directionError);
    if(directionError)
    {
        ERROR_MSG("The direction end-of-travel switch is still ON.\r\n");
        DEBUG_MSG("directionError=%d.\r\n",directionError);
        return FALSE;
    }
    if(vel1)
    {
        ERROR_MSG("The traction is still moving.\r\n");
        DEBUG_MSG("vel1=%ld.\r\n",vel1);
        return FALSE;
    }
    if(Power2)
    {
        ERROR_MSG("The direction is still being powered.\r\n");
        DEBUG_MSG("Power2=%d, (Mtr2_Flags1 & pida_Msk)=%d.\r\n", Power2, (Mtr2_Flags1 & pida_Msk));
        return FALSE;
    }
    
    /*Make sure the brake clamper GPIO is off before testing the emergency stop condition*/
    GetTime(&now);
    delay=CalculateDelayMs(&before, &now);
    if(ROVIM_T2D_BRAKE_CLAMP_TIME > delay)
    {
        ERROR_MSG("Brake clamper monostable is still ON. Do not forget that it activates every \
time you call this command. Wait %d ms before trying again.\r\n",
        ROVIM_T2D_BRAKE_CLAMP_TIME);
        DEBUG_MSG("delay=%d,timeout=%d.\r\n",delay, ROVIM_T2D_BRAKE_CLAMP_TIME);
        return FALSE;
    }
    /*So, the software is too fast to bring down the brake clamper output and test the emergency condition
    input sequentially. So we must wait for the output to go down. We do that with some busy waiting.
    Because life's long enough to do some busy waiting */
    GetTime(&before);
    ResetGPIO("brake clamper");
    SetDelay(&before, 0,msec_100);
    while(!Timeout(&before));
    GetGPIO("emergency stop condition",&emergencyStop);
    /*We shouldn't decide here to release the brake, so we just put it back and start counting for the next time*/
    SetGPIO("brake clamper");
    GetTime(&before);   
    
    if(emergencyStop)
    {
        ERROR_MSG("There is still an emergency stop condition active.\r\n");
        return FALSE;
    }
    
    DEBUG_MSG("Inputs are good. Make sure outputs are, too.\r\n");
    //Stop motors
    SoftStop(2);
    SoftStop(3);

    DEBUG_MSG("Vehicle is ready to be unclamped.\r\n");
    
    return TRUE;
}


//--------------------------------Periodic tasks of system monitoring-------------------------------
void ROVIM_T2D_MonitorSystem(void)
{
    BYTE previousVerbosity=0;
    BYTE error=0;
    BOOL finishUnlocking=FALSE;
    
    TIME start={0}, stop={0};
    static TIME prev={0};
    DWORD delay=0;
    static DWORD maxDelay=0;
    GetTime(&start);
    
    previousVerbosity = GetVerbosity();
    if (!ManualSysMonitoring)   //in manual mode we do not need to restrict verbosity
    {
        /*We only want to let pass warning and errors. Status*/
        SetVerbosity(VERBOSITY_LEVEL_ERROR | VERBOSITY_LEVEL_WARNING);
    }
    
    //Look for unrecoverable error conditions
    if( ROVIM_T2D_DetectFatalError(ROVIM_T2D_SYSTEM_MONITOR_PERIOD))
    {
        Reset();
    }
    
    //Look for severe error conditions
    error  = ROVIM_T2D_DetectTractionEror(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    error |= ROVIM_T2D_DetectBrakingError(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    error |= ROVIM_T2D_DetectDirectionEror(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    if(error)
    {
        ROVIM_T2D_Lockdown();
    }
    else
    {
        DEBUG_MSG("Yay, there is no need to go to lockdown.\r\n");
    }
    
    //look for and act on non-severe error conditions
    ROVIM_T2D_MonitorTractionWarning(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    ROVIM_T2D_MonitorBrakingWarning(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    ROVIM_T2D_MonitorDirectionWarning(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    
    //ROVIM_T2D_MonitorEmergencyConditionInspection(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    //look for and act on brake unlock completion
    finishUnlocking=ROVIM_T2D_DetectBrakeUnlock(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);

    //Detect and act on assynchronous switch to manual mode
    ROVIM_T2D_MonitorManualMode(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    
    ROVIM_T2D_DetectSigmaDError(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    
    ROVIM_T2D_PendingCmd(ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    
    if (!ManualSysMonitoring)
    {
        SetVerbosity(previousVerbosity);
    }
    
    if(finishUnlocking)
    {
        //finished unlocking brake inside the safety timeout
        ROVIM_T2D_FinishReleaseFromLockdown();
        UnlockingBrake=FALSE;
    }
    
    if(ForcePrintMsg) ForcePrintMsg=FALSE;
    
    //time measurements - debug purposes
    GetTime(&stop);
    delay=CalculateDelayMs(&start,&stop);
    if(delay > maxDelay)
    {
        maxDelay=delay;
        DEBUG_MSG("Monitor task max delay so far: %ld ms.\r\n",maxDelay);
    }
    //DEBUG_MSG("Monitor task delay: %ld ms.\r\n",delay);
    delay=CalculateDelayMs(&prev,&start);
    if(delay < ROVIM_T2D_SYSTEM_MONITOR_PERIOD)
    {
        DEBUG_MSG("Monitoring period smaller than expected. measured=%ld, expected=%ld.\r\n", delay, ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    }
    //DEBUG_MSG("Monitoring period measured=%ld, expected=%ld.\r\n", delay, (long)ROVIM_T2D_SYSTEM_MONITOR_PERIOD);
    GetTime(&prev);
    
    return;
}

    //////////////////////////////////////////////////////////////////////
    //      P E N D I N G    C M D    S E R V I C E S                   //
    //                                                                  //
    //  If motor now stopped (Power1=0 or Power2=0), and pending        //
    //  cmd is awaiting motor stopped, do it now.                       //
    //////////////////////////////////////////////////////////////////////
void ROVIM_T2D_PendingCmd(BYTE period)
{
    if ( (desiredMovement.type==FORWARD) && (!vel1)  && (movementType!=FORWARD) )
    {
        //pins are active low. Order of activation is important here.
        SetGPIO("engage handbrake");
        ResetGPIO("activate traction");
        SetGPIO("engage reverse");
        ResetGPIO("engage forward");
        ROVIM_T2D_SetSpeed(desiredMovement.speed);
        movementType=FORWARD;
        STATUS_MSG("Vehicle moving forward at approximately %d km/h/10.\r\n",desiredMovement.speed);
    }
    
    if ( (desiredMovement.type==REVERSE) && (!vel1)  && (movementType!=REVERSE) )
    {
        //pins are active low. Order of activation is important here.
        SetGPIO("engage handbrake");
        ResetGPIO("activate traction");
        SetGPIO("engage forward");
        ResetGPIO("engage reverse");
        ROVIM_T2D_SetSpeed(desiredMovement.speed);
        movementType=REVERSE;
        STATUS_MSG("Vehicle moving backwards at approximately %d km/h/10.\r\n",desiredMovement.speed);
    }
    DEBUG_MSG("desired type=%d, speed=%d, current type=%d, vel1=%ld, FW=%d,RV=%d.\r\n",
    desiredMovement.type, desiredMovement.speed, movementType, vel1, FORWARD,REVERSE);
    //neutral mode can be done immediately. There's no need to wait until the vehicle stops.
}

void ROVIM_T2D_DetectSigmaDError(BYTE period)
{
    static long ledOffTime=0;
    static BOOL printMsg=TRUE;
    WORD led=0;
    
    
    if(ForcePrintMsg) printMsg=TRUE;
    
    led=AdcConvert((WORD)SigmaDLed);
    
    /*FYI: Led Off: V ~=4900 mV; Led ON: V~=2300 mV.*/
    if(led > 4500) //this threshold != 5V is to avoid false positives (we are reading analogue)
    {
        ledOffTime+=(long)period;
    }
    else
    {
        ledOffTime=0;
        SigmaDError=TRUE;
        if(printMsg)
        {
            WARNING_MSG("The SigmaDrive traction controller either is turned off or has encountered an error.\r\n");
            printMsg=FALSE;
        }
    }
    
    if(ledOffTime >= (long)ROVIM_T2D_SIGMAD_ERROR_TIMEOUT)
    {
        SigmaDError=FALSE;
        printMsg=TRUE;
    }
}

void ROVIM_T2D_MonitorManualMode(BYTE period)
{
    BYTE autoSwitch=0;
    BYTE dcc=0, acc=0, fw=0, rv=0, handbrake=0;
    
    GetGPIO("auto mode switch",&autoSwitch);
    GetGPIO("decelerator",&dcc);
    GetGPIO("accelerator",&acc);
    GetGPIO("engage forward",&fw);
    GetGPIO("engage reverse",&rv);
    GetGPIO("engage handbrake",&handbrake);
    
    if(!autoSwitch)
    {
        /*Make sure the outputs are in a good state. If we don't do this, the vehicle may start 
        moving unexpectedly when the user switches to auto mode. We must keep monitoring these 
        variables because we must assume the user is stupid and will try to send commands while 
        on manual mode, and we have no interrupt associated with this pin.*/
        if(Power2)
        {
            WARNING_MSG("Detected incoherent direction outputs because of manual mode. Resetting them.\r\n");
            SoftStop(2);
        }
        if((dcc) || (acc) || (!fw) || (!rv) || (!handbrake))
        {
            WARNING_MSG("Detected incoherent traction outputs because of manual mode. Resetting them.\r\n");
            SoftStop(3);
        }
        autoMode=FALSE;
    }
    else
    {
        autoMode=TRUE;
        DEBUG_MSG("Detected automatic mode of operation.\r\n");
    }
}

//detects when traction system has encountered a serious error. It is in a Not OK state
BOOL ROVIM_T2D_DetectTractionEror(BYTE period)
{
    static WORD movingOnHoldTimeout=0;
    static BOOL printMsg=TRUE;
    
    if(ForcePrintMsg) printMsg=TRUE;
    
    /*XXX: Not ready yet. Must be done only with closed loop speed control
    if ((desiredMovement.speed < vel1) && (settlingTime > ROVIM_T2D_MAX_SETTLING_TIME))
    {
        ERROR_MSG("Vehicle is taking too long to reduce its speed.\r\n");
        DEBUG_MSG("Desired vel=%d, vel=%ld, settlingTime=%d.\r\n", desiredMovement.speed, vel1, settlingTime);
        return TRUE;
    }*/
    if (vel1 > ROVIM_T2D_CRITICAL_SPEED)
    {
        if(printMsg)
        {
            ERROR_MSG("Vehicle is moving too fast.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("vel=%ld, max speed=%ld.\r\n", vel1, ROVIM_T2D_CRITICAL_SPEED);
        return TRUE;
    }
    /* We cannot get accurate enough acceleration readings from the speed encoder we have and it's current mounting to do this check
    if ((abs(acc1)) > ROVIM_T2D_CRASH_ACC_THRESHOLD)
    {
        if(printMsg)
        {
            ERROR_MSG("Vehicle is accelerating too fast.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("acc=%ld, max acc=%d.\r\n", acc1, ROVIM_T2D_CRASH_ACC_THRESHOLD);
        return TRUE;
    }*/
    if ((desiredMovement.type==HILLHOLD) && (vel1) && (autoMode))
    {
        movingOnHoldTimeout+=period;
    }
    else
    {
        movingOnHoldTimeout=0;
    }
    if (movingOnHoldTimeout>ROVIM_T2D_MOVING_ON_HOLD_TIMEOUT)
    {
        if(printMsg)
        {
            ERROR_MSG("Vehicle should be on hold, yet it is moving.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("movement type=%d, vel=%ld, timeout=%d.\r\n", desiredMovement.type, vel1, ROVIM_T2D_MOVING_ON_HOLD_TIMEOUT);
        return TRUE;
    }

    printMsg=TRUE;
    DEBUG_MSG("No traction errors detected.\r\n");
    return FALSE;
}

//detects when braking system has encountered a serious error. It is in a Not OK state
BOOL ROVIM_T2D_DetectBrakingError(BYTE period)
{
    BYTE endOfTravelClamp=0, endOfTravelUnclamp=0, unclampAction=0;
    BYTE emergencyStop=0;
    static BOOL printMsg=TRUE;
    
    if(ForcePrintMsg) printMsg=TRUE;
    
    GetGPIO("brake clamp switch",&endOfTravelClamp);
    GetGPIO("brake unclamp switch",&endOfTravelUnclamp);
    GetGPIO("emergency stop condition",&emergencyStop);
    GetGPIO("brake unclamper",&unclampAction);
    
    if((emergencyStop) && (!UnlockingBrake))
    {
        //detected emergency condition - Error!
        if(printMsg)
        {
            ERROR_MSG("There is an emergency stop condition.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("emergency command=%d. Remember this will occur while you have the 'brake clamper' ON.\r\n", emergencyStop);
        return TRUE;
    }
    if((!endOfTravelClamp) && (!endOfTravelUnclamp) && (!unclampAction))
    {
        if(printMsg)
        {
            ERROR_MSG("Emergency brake is neither locked nor unlocked.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("clamp end-of-travel=%d, unclamp end-of-travel=%d, unclamping=%d.\r\n",endOfTravelClamp, endOfTravelUnclamp,unclampAction);
        return TRUE;
    }
    
    printMsg=TRUE;
    DEBUG_MSG("No braking errors detected.\r\n");
    return FALSE;
}

//detects when direction system has encountered a serious error. It is in a Not OK state
BOOL ROVIM_T2D_DetectDirectionEror(BYTE period)
{
    BYTE endOfTravel=0;
    static BOOL printMsg=TRUE;

    if(ForcePrintMsg) printMsg=TRUE;
    
    /* This check only makes sense if we can cut power to the direction motor when going to lockdown.
    Currently we can't do so, so this stays commented.
    pos=DirPos;
    if ((pos > ROVIM_T2D_DIRECTION_CRITICAL_UPPER_POSITION) || 
        (pos < ROVIM_T2D_DIRECTION_CRITICAL_LOWER_POSITION))
    {
        //reached soft direction position limit - Error!
        return TRUE;
    }*/
    
    GetGPIO("direction error switch",&endOfTravel);
    if (endOfTravel)
    {
        /*This should only be relevant in auto mode, but since this condition is hardwired to 
        force the vehicle to go to lockdown, we let the software follow along*/
        //reached hard direction position limit - Error!
        if(printMsg)
        {
            ERROR_MSG("Direction has reached one end-of-travel switch.\r\n");
            printMsg=FALSE;
        }
        DEBUG_MSG("Switch=%d.\r\n", endOfTravel);
        return TRUE;
    }
    
    printMsg=TRUE;
    DEBUG_MSG("No direction errors detected.\r\n");
    return FALSE;
}

void ROVIM_T2D_MonitorTractionWarning(BYTE period)
{
    //DEBUG_MSG("No traction warnings detected.\r\n");
}

void ROVIM_T2D_MonitorBrakingWarning(BYTE period)
{
    //DEBUG_MSG("No braking warnings detected.\r\n");
}

void ROVIM_T2D_MonitorDirectionWarning(BYTE period)
{
    /* Detect if the motor is under a big load for a long period of time.
    Since we do not have current sensors for the motors, we try to emulate that monitoring the
    PWM duty cycle. If it stays for too long too high, it means the system is not being able to
    reach a stable state, and the behaviour is in some sort erroneous.
    We chose the absolute number (since power up) of stressed "samples" as a metric instead of
    the relative (since last move command) because it reduces complexity (communication between
    functions is simplified). Also we believe in the long term this count should accuratelly reflect
    the amount of stress the motor is having.
    NOTE: We should be doing this check every at VSP intervals...*/
    /*
    //Only detect the stress if the motor if PID is active
    if (Mtr2_Flags1 & pida_Msk)
    {
        // Due to the nature of PID control, the motor may not be at full power and still be in stress
        if (Power2 > (VMAX1/ROVIM_T2D_DIRECTION_MOTOR_STRESS_DUTY_CYCLE_THRESHOLD))
        {
            MotorStressMonitor[2]++;
        }
        if (MotorStressMonitor[2] > ROVIM_T2D_DIRECTION_MOTOR_STRESS_CNT_THRESHOLD)
        {
            //reached direction motor overstress condition - Error!
            ERROR_MSG("ROVIM direction motor is too stressed. Going to stop it for precaution.\r\n");
            DEBUG_MSG("MotorStressMonitor[2]=%d, threshold # stress events=%d.\r\n", 
            MotorStressMonitor[2], ROVIM_T2D_DIRECTION_MOTOR_STRESS_CNT_THRESHOLD);
            return;
        }
    }*/
    //DEBUG_MSG("No direction warnings detected.\r\n");
}

//detects when the system has encountered an error from which it cannot recover
BOOL ROVIM_T2D_DetectFatalError(WORD period)
{
    static WORD unclampActionActiveTime=0;
    BYTE unclampAction=0;
    BYTE endOfTravelClamp=0;
    BYTE emergencyStop=0;
    
    /*TODO:
    GetGPIO("brake clamp switch",&endOfTravelClamp);
    GetGPIO("emergency stop condition",&emergencyStop);
    GetGPIO("brake unclamp command",&unclampAction);
    
    if(unclampAction)
    {
        unclampActionActiveTime+=period;
    }
    else
    {
        unclampActionActiveTime=0;
    }
    if(unclampActionActiveTime > ROVIM_T2D_FATAL_UNCLAMP_TIMEOUT)
    {
        FATAL_ERROR_MSG("User is pressing the button for too long - or something else really, really \
bad happened.\r\n");
        DEBUG_MSG("Timeout=%d, time passed=%d.\r\n",ROVIM_T2D_FATAL_UNCLAMP_TIMEOUT, unclampActionActiveTime));
        return TRUE;
    }*/
    
    /* Need to have a timeout here, because of the inertia
    GetGPIO("traction voltage sensor", &tractionVoltage);
    if ((!tractionVoltage) && (vel1))
    {
        //This means either the contactor, fuse or battery for the traction system is NOK. The vehicle is not in neutral
        FATAL_ERROR_MSG("Vehicle is moving, yet is not being powered.");
        DEBUG_MSG("VB+=%d, vel=%ld.\r\n", tractionVoltage, vel1);
        return TRUE;
    }*/
    DEBUG_MSG("No fatal errors detected.\r\n");
    return FALSE;
}

#if 0
BOOL ROVIM_T2D_MonitorEmergencyConditionInspection(WORD period)
{
    static WORD timeoutCount=0;
    BYTE emergencyStop=0;
    
    if(!InspectingEmergencyCondition)
    {
        timeoutCount=0;
        return FALSE;
    }
    
    timeoutCount+=period;
    /*Before we can test the brake clamper switch, we must make sure the emergency stop condition is off.
    But what we really want to know is exactly that, so we test it directly instead of the switch*/
    GetGPIO("emergency stop condition",&emergencyStop);
    if ((!emergencyStop) && (timeoutCount <= ROVIM_T2D_TEST_EMERGENCY_CONDITION_TIMEOUT))
    {
        //this must be done outside this function so that the status msg gets printed
        return TRUE;
    }
    if (timeoutCount > ROVIM_T2D_TEST_EMERGENCY_CONDITION_TIMEOUT)
    {
        //timeout expired. Going back to full lockdown
        SetGPIO("brake clamper");
        ResetGPIO("brake unclamper");
        ERROR_MSG("Took too long to test for an emergency stop condition. try again\r\n");
        DEBUG_MSG("stop condition=%d, time passed=%d > timeout=%d, inspecting emergency stop=%d.\r\n",emergencyStop, timeoutCount, ROVIM_T2D_TEST_EMERGENCY_CONDITION_TIMEOUT, InspectingEmergencyCondition);
        InspectingEmergencyCondition=FALSE;
    }
    return FALSE;
    
    
    
    GetGPIO("emergency stop condition",&emergencyStop);
    /*We shouldn't decide here to release the brake, so we just put it back and start counting for the next time*/
    SetGPIO("brake clamper");
    GetTime(&before);   
    
    if(emergencyStop)
    {
        ERROR_MSG("There is still an emergency stop condition active.\r\n");
        return FALSE;
    }
    
    DEBUG_MSG("Inputs are good. Make sure outputs are, too.\r\n");
    //Stop motors
    SoftStop(2);
    SoftStop(3);

    DEBUG_MSG("Vehicle is ready to be unclamped.\r\n");
}
#endif

BOOL ROVIM_T2D_DetectBrakeUnlock(WORD period)
{
    static WORD timeoutCount=0;
    BYTE endOfTravelUnclamp=0;
    
    if(!UnlockingBrake)
    {
        timeoutCount=0;
        return FALSE;
    }
    
    timeoutCount+=period;
    GetGPIO("brake unclamp switch",&endOfTravelUnclamp);
    if ((endOfTravelUnclamp) && (timeoutCount <= ROVIM_T2D_BRAKE_UNLOCK_TIMEOUT))
    {
        //this must be done outside this function so that the status msg gets printed
        return TRUE;
    }
    if (timeoutCount > ROVIM_T2D_BRAKE_UNLOCK_TIMEOUT)
    {
        //timeout expired. Going back to full lockdown
        SetGPIO("brake clamper");
        ResetGPIO("brake unclamper");
        ERROR_MSG("Brake unlocking took too long. Restart the procedure.\r\n");
        DEBUG_MSG("brake unlock end-of-travel=%d, time passed=%d > timeout=%d, unlocking=%d.\r\n",endOfTravelUnclamp, timeoutCount, ROVIM_T2D_BRAKE_UNLOCK_TIMEOUT, UnlockingBrake);
        UnlockingBrake=FALSE;
    }
    return FALSE;
}


//--------------------------------------------------------------------------------------------------
//Maintain PWM signal used by ROVIM T2D motors controlled by GPIO
//motors controller by the Dalf defaullt motor interface are outside the scope of this function
void ROVIM_T2D_ServicePWM(void)
{
    static BYTE accDutyCtrl=0, dccDutyCtrl=0;
    static WORD iDebug=1, jDebug=1; //debug stuff
    static BYTE printctrl=0, dccprintctrl=0; //debug stuff
    BYTE previousVerbosity=0;
    TIME start={0}, stop={0};
    DWORD delay=0;
    static DWORD maxDelay=0;
    GetTime(&start);

    previousVerbosity = GetVerbosity();
    SetVerbosity(VERBOSITY_LEVEL_ERROR | VERBOSITY_LEVEL_WARNING);
    /*DebugPWM and printctrl mechanism:
    DebugPWM is the global debug enable. OFF= normal operation. ON= debug mode
    printctrl controls the prints, to avoid too much info. Only on the first time a new if case is
    entered is debug info printed. This flag is reset on the next if case entered.
    dccprintctrl is like printctrl, but for dcc only. This is needed because Acc and Dcc ifs are not
    mutually exclusive.
    DebugPWM is also a counter controlling the amount of time debug is enabled.*/
    if(DebugPWM)
    {
        //debug stuff
        if (!printctrl)
        {
            printctrl=0xFF;
        }
        if(!dccprintctrl)
        {
            dccprintctrl=0xFF;
        }
    }
    else
    {
        //debug stuff
        printctrl=0;
        dccprintctrl=0;
    }
    /*DEBUG_MSG("Running PWM refresh thread. accDutyCtrl=%d, dccDutyCtrl=%d, PeriodCnt=%d, \
movementType=%d, WaitForAccDccDecay=%d, AccDutyCycle=%d, DccDutyCycle=%d.\r\n", accDutyCtrl, 
dccDutyCtrl, PeriodCnt, movementType, WaitForAccDccDecay, AccDutyCycle, DccDutyCycle); //debug stuff*/
    
    /*
    //debug stuff
    //In order to not overload the serial with prints, we do some mambo-jambo here
    if(jDebug>100)iDebug=1;
    if(jDebug>10000){jDebug=1; DebugPWM--;}
    if(DebugPWM)
    {
        jDebug++;
        iDebug++;
    }*/
    
    if(movementType==HILLHOLD)
    {
        //if(!(iDebug%4))MSG("On HILLHOLD.\r\n"); //debug stuff
        if ((DebugPWM) && (printctrl & 0x01))
        {
            //debug stuff
            MSG("Vehicle on hill hold. type=%d.\r\n",movementType);
            printctrl=0xFE;
        }
        if(DebugPWM) DebugPWM--;//otherwise this if case will hang forever with no new information
        //make sure we have the signals down
        ResetGPIO("decelerator");
        ResetGPIO("accelerator");
        goto exit;
    }
    if(!autoMode)
    {
        //if(!(iDebug%4))MSG("On manual.\r\n"); //debug stuff
        if ((DebugPWM) && (printctrl & 0x02))
        {
            //debug stuff
            MSG("Vehicle on manual. type=%d.\r\n",autoMode);
            printctrl=0xFD;
        }
        if(DebugPWM) DebugPWM--;//otherwise this if case will hang forever with no new information
        //make sure we have the signals down
        ResetGPIO("decelerator");
        ResetGPIO("accelerator");
        goto exit;
    }
    if(WaitForAccDccDecay!=0)
    {
        /* We should not have both signals active at the same time. We wait until the previous one
        goes to 0 (or close to it), to activate the new one*/
        //if(!(iDebug%4))MSG("Decay!=0,=%d.\r\n",WaitForAccDccDecay); //debug stuff
        if ((DebugPWM) && (printctrl & 0x04))
        {
            //debug stuff
            MSG("Waiting for Acc/Dcc signal to fall before rising the other. \
WaitForAccDccDecay=%d.\r\n", WaitForAccDccDecay);
            printctrl=0xFB;
        }
        WaitForAccDccDecay--;
        ResetGPIO("decelerator");
        ResetGPIO("accelerator");
        
        goto exit;
    }
    if(PeriodCnt >= 100)
    {
        //if(!(iDebug%4))MSG("PeriodCnt=%d, >= 100.\r\n",PeriodCnt); //debug stuff
        if ((DebugPWM) && (printctrl & 0x08))
        {
            //debug stuff
            MSG("Resetting period counter. PeriodCnt=%d.\r\n",PeriodCnt);
            printctrl=0xF7;
            DebugPWM--;
        }
        accDutyCtrl=0;
        dccDutyCtrl=0;
        PeriodCnt=0;
    }
    PeriodCnt++;
    //accelerator pwm
    if(AccDutyCycle <= accDutyCtrl)
    {
        //if(!(iDebug%4))MSG("AccDutyCycle<=Ctrl,Ctrl=%d,Duty=%d.\r\n",accDutyCtrl,AccDutyCycle); //debug stuff
        if ((DebugPWM) &&(printctrl & 0x10))
        {
            //debug stuff
            MSG("Acc=0 now. AccDutyCycle=%d, accDutyCtrl=%d, PeriodCnt=%d.\r\n", AccDutyCycle, accDutyCtrl, PeriodCnt);
            printctrl=0xEF;
        }
        ResetGPIO("accelerator");
    }
    else
    {
        //if(!(iDebug%4))MSG("AccDutyCycle>Ctrl,Ctrl=%d,Duty=%d.\r\n",accDutyCtrl,AccDutyCycle); //debug stuff
        if ((DebugPWM) && (printctrl & 0x20))
        {
            //debug stuff
            MSG("Acc=1 now. AccDutyCycle=%d, accDutyCtrl=%d, PeriodCnt=%d.\r\n", AccDutyCycle, accDutyCtrl, PeriodCnt);
            printctrl=0xDF;
        }
        SetGPIO("accelerator");
        accDutyCtrl++;
    }
    //decelerator pwm
    if(DccDutyCycle <= dccDutyCtrl)
    {
        //if(!(iDebug%4))MSG("DccDutyCycle<=Ctrl,Ctrl=%d,Duty=%d.\r\n",dccDutyCtrl,DccDutyCycle); //debug stuff
        if ((DebugPWM) && (dccprintctrl & 0x01))
        {
            //debug stuff
            MSG("Dcc=0 now. DccDutyCycle=%d, dccDutyCtrl=%d, PeriodCnt=%d.\r\n", DccDutyCycle, dccDutyCtrl, PeriodCnt);
            dccprintctrl=0xFE;
        }
        ResetGPIO("decelerator");
    }
    else
    {
        //if(!(iDebug%4))MSG("DccDutyCycle>Ctrl,Ctrl=%d,Duty=%d.\r\n",dccDutyCtrl,DccDutyCycle); //debug stuff
        if ((DebugPWM) && (dccprintctrl & 0x02))
        {
            //debug stuff
            MSG("Dcc=1 now. DccDutyCycle=%d, dccDutyCtrl=%d, PeriodCnt=%d.\r\n", DccDutyCycle, dccDutyCtrl, PeriodCnt);
            dccprintctrl=0xFD;
        }
        SetGPIO("decelerator");
        dccDutyCtrl++;
    }
    
    /*We use a label here because there are many exit points and the exit code is quite large*/
exit:
    SetVerbosity(previousVerbosity);
    GetTime(&stop);
    delay=CalculateDelayMs(&start,&stop);
    if(delay > maxDelay)
    {
        maxDelay=delay;
        DEBUG_MSG("PWM refresh task max delay so far: %d ms.\r\n",maxDelay);
    }
}

void ROVIM_T2D_UpdateVelocity1(void)
{
    long temp1=0, temp2=0, velRPM=0;
    long currVel=0;
    static long n1Vel=0, n2Vel=0, n3Vel=0;
    static long sumE1=0;
    static long t=0;
    short long E1=0;
    static long diffE1=0;   //tick count during VSP. Same as V1, but V1 seems much more inaccurate, so we use this
    static BYTE debugcnt=0, end=1; //debug stuff
    /*static long t2=0;
    static long sumE12=0;*/
    
    
    UpdateVelocity1();
    
    INTCONbits.GIEH = 0;    // Disable high priority interrupts
    E1=encode1;
    INTCONbits.GIEH = 1;    // Enable high priority interrupts
    
    diffE1=E1-diffE1;
    if(diffE1<0)
        diffE1=0;
    
    sumE1+=(long)diffE1;
    if(sumE1<0)
        sumE1=0;
    
    t+=(long)VSP1;
    diffE1=E1;
    
    /* Velocity calculation scheme: We need enough samples to produce an accurate enough velocity reading.
    If the vehicle is moving too slowly, we just won't get them. So I say to only calculate the speed
    if we have a minimum number of new samples collected, or the timeout to get them has expired.
    Still, we may get sudden speed readings changes, that are not a representation of the physical
    process, i.e. the vehícle is moving relativelly steady. This may be due to several factors, 
    such as the accelerator DAC or chain slack. So, to the actual speed acessible to the rest of the
    software is filtered through digital recursive filter with a decay parameter, just like the
    stock dalf firmware does for the ADC readings.
    The properties of the encoder sensor used and its mounting make the calculation of the 
    acceleration pretty useless, since it produces even more inaccurate readings than the speed. So
    we are not going to use it.
    */
    if( (sumE1 >= ROVIM_T2D_VEL1_CALC_MIN_TICK_CNT) || (t >= (6*(long)VSP1)) )
    {
        temp1 = sumE1 * (long)60000 ;
        temp2 = t * TPR1 ;
        velRPM = temp1/temp2;   //I don't feeel like fixing the rounding mistake
        //vel (Km/10/h) = vel (RPM) * Perimeter (cm) * 3.6 /60 (s/min) / 100 (m/cm) / 10
        temp1 = velRPM * ROVIM_T2D_WHEEL_PERIMETER * 36;
        temp2 = 6000;
        currVel=temp1/temp2;   //I don't feeel like fixing the rounding mistake
        
        
        vel1=(currVel<<3) + (n1Vel<<2) + (n2Vel<<1) + (n3Vel<<1);
        vel1=vel1>>4;
        n3Vel=n2Vel;
        n2Vel=n1Vel;
        n1Vel = currVel;
        
        /* acceleration not used - for now
        //acc(Km/10/h/s) = dV(km/h/10) / dt(ms) * 1000(ms/s)
        temp1 = (vel1-n1Vel)*1000;
        temp2 = t;
        acc1 = temp1/temp2; //I don't feeel like fixing the rounding mistake*/
        
        debugcnt++;
        if(debugcnt>=end)
        {
            debugcnt=0;
            DEBUG_MSG("E1=%ld, V1=%ld, t=%ld, sum=%ld velRPM=%ld, vel1=%ld, n1Vel=%ld, currvel=%ld, n2Vel=%ld, n3Vel=%ld.\r\n",
            diffE1, (long)V1, t, sumE1, velRPM, vel1, n1Vel, currVel, n2Vel, n3Vel);
        }
        
        sumE1=0;
        t=0;
    }
    /*t2+=(long)VSP1;
    if(t2==60000)
    {
        sumE12=(long)E1-sumE12;
        ERROR_MSG("ticks=%ld.\r\n",sumE12);
        sumE12=E1;
        t2=0;
    }*/
}

void ROVIM_T2D_FullBrake(void)
{
    DEBUG_MSG("Applying full brake.\r\n");
    CMD='G';
    ARG[0]=ROVIM_T2D_DECELERATE_CMD_CODE;
    ARG[1]=100;
    ARGN=2;
    TeCmdDispatchExt();
}

void ROVIM_T2D_SetSpeed(BYTE speed)
{
    BYTE dutyCycle=0;
    long temp=0;
    
    /*temp=(long)speed>>1;
    if(temp>=SpeedToDutyCycleLen)
    dutyCycle=SpeedToDutyCycle[temp]; //get the duty cycle from the LUT
    temp=(long)dutyCyle*SpeedDCScaling/10; //Scale the conversion. This can be changed in real time
    dutyCycle=(BYTE) temp;
    if (dutyCycle > 100) dutyCycle=100;*/
    //XXX: fix this
    
    temp=speed*573;
    temp=temp+27959;
    temp=temp/1000;
    if ((speed<ROVIM_T2D_LOWER_SPEED_LIMIT) || (temp<0))
    {
        temp=0;
    }
    if(temp>100)
    {
        temp=100;
    }
    dutyCycle=(BYTE) temp;
    
    CMD='G';
    ARG[0]=ROVIM_T2D_ACCELERATE_CMD_CODE;
    ARG[1]=dutyCycle;
    ARGN=2;
    TeCmdDispatchExt();
}

//just like in dalf, this function only indicates the PWM set by the board, 
//it doesn't translate actual voltage fed to the motor (it may be disconnected, for example)
BYTE ROVIM_T2D_LightPWMLed(BYTE dutyCycle, BYTE direction)
{
    if(dutyCycle>100)
    {
        WARNING_MSG("Duty cycle larger than 100 %%.\r\n");
        dutyCycle=100;
    }
    if(direction>REVERSE)
    {
        ERROR_MSG("Unexpected direction.\r\n");
        return eParmErr;
    }
    /*To control the PWM led, it seems we need to actualy use the dalf firmware to drive the PWM.
    Since it isn't connected, there's no problem.*/
    CMD='X';
    ARG[0]=1;
    ARG[1]=direction;
    ARG[2]=dutyCycle;
    ARGN=3;
    TeCmdDispatchExt();
}

void    ROVIM_T2D_ServiceLED(void)        // Periodic LED service
{
///////////////////////////////////////////////////////////////////////////////
//           patterns on LED1 and LED2 indicating motor status:              //
//                                                                           //
//  LED1                                                                     //
//  OFF:        Motor is not good to go (maybe still be ON)                  //
//  ON:         Motor is good to go.                                         //
//                                                                           //
//  LED2                                                                     //
//  OFF:        Power=0. No Power applied. ---------------- Off              //
//  FAST BLINK: Power>0, TGA active.  Power applied. ------- TGA Active      //
//  SLOW BLINK: Power>0, TGA inactive, Power applied, V>0. - Open loop move. //
//  ON:         Power>0, TGA inactive, Power applied, V=0. - Stalled.        //
//                                                                           //
//                 Three possible patterns on LED3                           //
//  OFF:          No Error.                                                  //
//  FAST BLINK:   ROVIM is in lockdown mode                                  //
//  SLOW BLINK:   R/C signal loss (possibly transient condition)             //
//  ON:           Low Batt (VBATT < VBWARN)                                  //
///////////////////////////////////////////////////////////////////////////////

    BYTE VB=0;
    BYTE previousVerbosity=0;
    
    previousVerbosity = GetVerbosity();
    SetVerbosity(VERBOSITY_LEVEL_ERROR | VERBOSITY_LEVEL_WARNING);
    // Reset counter and do right shift (with wrap) on led scheduler.
    ledshift >>= 1; if(!ledshift) ledshift = 0x80000000;

    // Determine appropriate LED1 pattern
    GetGPIO("traction voltage sensor",&VB);
    
    /* LED1 ON: motor is ready to go*/
    if( (!inLockdown) && ( (movementType!=HILLHOLD) && (autoMode) ) && VB && (!SigmaDError) )
        grn1pattern = LED_MTR_STALL;
    /* Not used for now
    else if ( (movementType==NEUTRAL) && (!manualMode) )
        grn1pattern = LED_MTR_TGA;
    else if ( ((movementType==FORWARD) || (movementType==REVERSE)) && (!manualMode) )
        grn1pattern = LED_MTR_OPENLP;*/
    /*LED1 OFF: some condition is preventing the led to go*/
    else
        grn1pattern = LED_MTR_OFF;

    // Determine appropriate LED2 pattern
    if(!Power2)                             grn2pattern = LED_MTR_OFF;
    else if(Mtr2_Flags1 & tga_Msk)          grn2pattern = LED_MTR_TGA;
    else if(V2)                             grn2pattern = LED_MTR_OPENLP;
    else                                    grn2pattern = LED_MTR_STALL;

    // Determine appropriate LED3 pattern
    if(LedErr==0)                           redpattern = LED_FULLOFF;
    else if(LedErr & Lckmsk)                redpattern = ROVIM_T2D_LED_LOCKDOWN;
    else if(LedErr & (SL1msk) + SL2msk)     redpattern = LED_SIGNAL_LOSS;
    else if(LedErr & VBATTmsk)              redpattern = LED_VBATT;


    // Adjust LED's as required.
    if(ledshift & grn1pattern) _LED1_ON; else _LED1_OFF;
    if(ledshift & grn2pattern) _LED2_ON; else _LED2_OFF;
    if(ledshift & redpattern)  _LED3_ON; else _LED3_OFF;
    SetVerbosity(previousVerbosity);
}

//-----------------------------Commands accessible from the command line or I2C---------------------

BYTE ROVIM_T2D_Lockdown(void)
{
    if(UnlockingBrake)
    {
        STATUS_MSG("Aborting current emergency brake unlock.\r\n");
        inLockdown=FALSE;
        LedErr &= ~Lckmsk;
        UnlockingBrake=FALSE;
    }
    
    if(inLockdown)
    {
       STATUS_MSG("Already in Lockdown mode.\r\n");
       return NoErr;
    }
    
    STATUS_MSG("Going to lock down mode.\r\n");
    
    //Once we engage the handbrake, the traction motor (the most critical) cannot work, so we're safe
    ROVIM_T2D_LockBrake();
    
    //Stop motors controlled through dalf's firmware
    SoftStop(2);
    //Stop traction motor and set it to hold position
    SoftStop(3);
    
    LockCriticalResourcesAccess();
    
    inLockdown=TRUE;
    LedErr|=Lckmsk;
    STATUS_MSG("ROVIM now in lockdown mode. Emergency brake will finish locking and vehicle will \
not be able to move while on this state.\r\n");
    
    return NoErr;
}

BYTE ROVIM_T2D_ReleaseFromLockdown(void)
{
    if(!inLockdown)
    {
        STATUS_MSG("ROVIM is already good to go.\r\n");
        return NoErr;
    }
    if(UnlockingBrake)
    {
        STATUS_MSG("ROVIM is already ready - you have to manually unlock the emergency brake now.\r\n");
        return NoErr;
    }
    
    if(!ROVIM_T2D_ValidateState())
    {
        ERROR_MSG("ROVIM is not ready to go. make sure \
every safety system is OK and try again.\r\nFor a list of the safety checks to pass before the \
vehicle can move, consult the user manual, or activate debug messages.\r\n");
        return eErr;
    }

    STATUS_MSG("Manually unclamp the emergency brake within %d ms, using the identified button \
on the control panel.\r\n\You must press it for longer than %d ms to deactivate all safety \
systems.\r\nThe system will become operational once it detects it is fully unclamped.\r\n", 
ROVIM_T2D_BRAKE_UNLOCK_TIMEOUT, ROVIM_T2D_BRAKE_CLAMP_TIME);
    ROVIM_T2D_UnlockBrake();
    UnlockingBrake=TRUE;
    
    return NoErr;
}

BYTE ROVIM_T2D_SoftStop(void)
{
    BYTE mtr=0;

    if (ARGN > 2)
    {
        return eNumArgsErr;
    }
    
    if (ARGN==2)
    {
        mtr=ARG[1];
        if(mtr>2)
        {
            ERROR_MSG("Motor number must be %d (traction) or %d (direction). If no motor is specified, all will be stopped.\r\n",1,2);
            return eParmErr;
        }
        if(mtr==1) mtr=3;   //motor 1 doesn't really exist
        SoftStop(mtr);
        STATUS_MSG("Motor %d stopped.\r\n", mtr);
    }
    else
    {
        SoftStop(2);
        SoftStop(3);
        STATUS_MSG("Motors stopped.\r\n");
    }
    
    return NoErr;
}

/*Allows access to GPIO sub-driver*/
BYTE ROVIM_T2D_ControlGPIO(void)
{
    BYTE GPIONumber=0;
    BYTE value=0;
    BYTE option=0;
    
    if (ARGN != 3)
    {
        return eNumArgsErr;
    }
    GPIONumber=ARG[2];
    option=ARG[1];
    if ((GPIONumber == 0) || (GPIONumber > ngpios))
    {
        ERROR_MSG("GPIO %d doesn't exist.\r\n",GPIONumber);
        return eParmErr;
    }
    
    GPIONumber--;
    
    switch(option)
    {
        case 1: 
            SetGPIO(GPIOsDescription[GPIONumber].name);
            STATUS_MSG("\"%HS\" value set to 1.\r\n",GPIOsDescription[GPIONumber].name);
            return NoErr;
        case 2:
            ResetGPIO(GPIOsDescription[GPIONumber].name);
            STATUS_MSG("\"%HS\" value reset to 0.\r\n",GPIOsDescription[GPIONumber].name);
            return NoErr;
        case 3:
            ToggleGPIO(GPIOsDescription[GPIONumber].name);
            STATUS_MSG("\"%HS\" value toggled.\r\n",GPIOsDescription[GPIONumber].name);
            return NoErr;
        case 4:
            GetGPIO(GPIOsDescription[GPIONumber].name, &value);
            STATUS_MSG("\"%HS\" value is %d.\r\n",GPIOsDescription[GPIONumber].name,value);
            return NoErr;
        default:
            ERROR_MSG("GPIO action not valid.\r\\n");
            return eParmErr;
    }
}

BYTE ROVIM_T2D_Accelerate(void)
{
    BYTE dutyCycle=0;
    
    if (ARGN != 2)
    {
        return eNumArgsErr;
    }
    
    dutyCycle=ARG[1];
    if (dutyCycle > 100)
    {
        ERROR_MSG("Duty cycle can only vary between 0 and 100%%.\r\n");
        return eParmErr;
    }
    
    if (!autoMode)
    {
        ERROR_MSG("You cannot control the vehicle with the micro controller while on manual mode.\r\n");
        return eParmErr;
    }
    
    AccDutyCycle=dutyCycle;
    //Used to variate PWM led of motor1, to have a visual traction power indicator.
    ROVIM_T2D_LightPWMLed(AccDutyCycle, FORWARD);
    if (DccDutyCycle!=0)
    {
        //Wait 2*tau before starting to accelerate
        WaitForAccDccDecay=2*ROVIM_T2D_TIME_CONSTANT_ACC_DCC_DAC/ROVIM_T2D_PWM_REFRESH_PERIOD;
        DEBUG_MSG("Waiting for %d ms to avoid raising accelerator and decelerator simultaneously. \
WaitForAccDccDecay=%u.\r\n", WaitForAccDccDecay, (WaitForAccDccDecay*ROVIM_T2D_PWM_REFRESH_PERIOD));
    }
    DccDutyCycle=0;
    /*If we don't reset the PWM period counter, we may have the duty cycle raise up to twice our spec
    (worst case) if the change happens when this counter is already very ahead. This way, worst case
    is the first run of the thread (ROVIM_T2D_PWM_REFRESH_PERIOD ms longer than spec).*/
    PeriodCnt=100;
    STATUS_MSG("Accelerator set to %d%%.\r\n",AccDutyCycle);
    
    return NoErr;
}

BYTE ROVIM_T2D_Decelerate(void)
{
    BYTE dutyCycle=0;
    
    if (ARGN != 2)
    {
        return eNumArgsErr;
    }
    dutyCycle=ARG[1];
    if (dutyCycle > 100)
    {
        ERROR_MSG("Duty cycle can only vary between 0 and 100%%.\r\n");
        return eParmErr;
    }
    
    if (!autoMode)
    {
        ERROR_MSG("You cannot control the vehicle with the micro controller while on manual mode.\r\n");
        return eParmErr;
    }
    
    DccDutyCycle=dutyCycle;
    ROVIM_T2D_LightPWMLed(DccDutyCycle, REVERSE);
    if (AccDutyCycle!=0)
    {
        //Wait 2*tau before starting to decelerate
        WaitForAccDccDecay=2*ROVIM_T2D_TIME_CONSTANT_ACC_DCC_DAC/ROVIM_T2D_PWM_REFRESH_PERIOD;
        DEBUG_MSG("Waiting for %d ms to avoid raising accelerator and decelerator simultaneously. \
WaitForAccDccDecay=%u.\r\n", WaitForAccDccDecay, (WaitForAccDccDecay*ROVIM_T2D_PWM_REFRESH_PERIOD));
    }
    AccDutyCycle=0;
    /*If we don't reset the PWM period counter, we may have the duty cycle raise up to twice our spec
    (worst case) if the change happens when this counter is already very ahead. This way, worst case
    is the first run of the thread (ROVIM_T2D_PWM_REFRESH_PERIOD ms longer than spec).*/
    PeriodCnt=100;
    STATUS_MSG("Decelerator set to %d%%.\r\n",DccDutyCycle);
    
    return NoErr;
}

BYTE ROVIM_T2D_SetMovement(void)
{
    BYTE speed=0;   //in km/h/10
    BYTE direction=0;
    
    switch (ARGN)
    {
        case 1:
            //Hold (handbrake) - default: No need for any arguments
            direction=HILLHOLD;
            speed=SPEEDZERO;
            break;
        case 2:
            //Neutral or hold: specify only the type of movement - speed is zero
            direction=ARG[1];
            speed=SPEEDZERO;
            break;
        case 3:
            //Any kind of movement: specify the type of movement and the desired speed
            direction=ARG[1];
            speed=ARG[2];
            break;
        default:
            DEBUG_MSG("Wrong number of arguments.\r\n");
            return eNumArgsErr;
    }
    
    if (direction > 3)
    {
        ERROR_MSG("Direction must be %d (forward), %d (reverse), %d (neutral) or %d (hold).\r\n", 
        FORWARD, REVERSE, NEUTRAL, HILLHOLD);
        return eParmErr;
    }
    if ( (speed > ROVIM_T2D_MAX_SPEED) || ((direction <2) && (speed < ROVIM_T2D_LOWER_SPEED_LIMIT)) )
    {
        //in neutral and hold cases, speed is allways 0, wich may be <ROVIM_T2D_LOWER_SPEED_LIMIT
        ERROR_MSG("Speed must vary in 1/10 km/h between %d km/h/10 and %d km/h/10.\r\n",
        ROVIM_T2D_LOWER_SPEED_LIMIT, ROVIM_T2D_MAX_SPEED);
        DEBUG_MSG("speed=%d, direction=%d.\r\n",speed, direction);
        return eParmErr;
    }
    if ( (direction!=HILLHOLD) && (!autoMode) )
    {
        ERROR_MSG("You cannot move the vehicle with the micro controller while on manual mode.\r\n");
        return eParmErr;
    }
    if ( (direction!=HILLHOLD) && (SigmaDError) )
    {
        ERROR_MSG("There is an error with the traction controller, or it is turned OFF. Wait %ld \
ms after solving the error before retrying.\r\n",(long)ROVIM_T2D_SIGMAD_ERROR_TIMEOUT);
        return eParmErr;
    }
    
    switch(direction)
    {
        case HILLHOLD: //Hold
            SetGPIO("engage forward");
            SetGPIO("engage reverse");
            SetGPIO("engage handbrake");
            ResetGPIO("activate traction");
            
            movementType=HILLHOLD;
            desiredMovement.type=HILLHOLD;
            desiredMovement.speed=SPEEDZERO;
            CMD='G';
            ARG[0]=ROVIM_T2D_ACCELERATE_CMD_CODE;
            ARG[1]=0;
            ARGN=2;
            TeCmdDispatchExt();
            /*The trace for this case is debug, but for the others is status, because this command is
            called from anywhere is the code to make sure the outputs are as expected. This creates
            some unexpected messages during normal program execution. The others cases do not have this problem*/
            DEBUG_MSG("Setting vehicle on hill hold. It will hold position once it reaches a standstill.\r\n");
            DEBUG_MSG("desired type=%d, speed=%d, current type=%d.\r\n",desiredMovement.type, desiredMovement.speed, movementType);
            break;
        case NEUTRAL: //Neutral
            ResetGPIO("engage handbrake");
            SetGPIO("engage forward");
            SetGPIO("engage reverse");
            SetGPIO("activate traction");

            movementType=NEUTRAL;
            desiredMovement.type=NEUTRAL;
            desiredMovement.speed=SPEEDZERO;    //actually, we don't care
            CMD='G';
            ARG[0]=ROVIM_T2D_ACCELERATE_CMD_CODE;
            ARG[1]=0;
            ARGN=2;
            TeCmdDispatchExt();
            STATUS_MSG("Vehicle set in neutral.\r\n");
            DEBUG_MSG("desired type=%d, speed=%d, current type=%d.\r\n",desiredMovement.type, desiredMovement.speed, movementType);
            break;
        case FORWARD: //Forward
            WARNING_MSG("Speed control done only in open loop currently. Check manually if the set speed matches your request.\r\n");
            desiredMovement.type=FORWARD;
            desiredMovement.speed=speed;
            if ((vel1) && (movementType!=desiredMovement.type))
            {
                DEBUG_MSG("Bringing vehicle to a standstill before moving forward. Please wait.\r\n");
                ROVIM_T2D_FullBrake();
            }
            if (movementType==desiredMovement.type)
            {
                //If we're already moving in the direction we want, just accelerate
                ROVIM_T2D_SetSpeed(desiredMovement.speed);
                STATUS_MSG("Vehicle moving forward at approximately %d km/h/10.\r\n",desiredMovement.speed);
            }
            DEBUG_MSG("desired type=%d, speed=%d, current type=%d.\r\n",desiredMovement.type, desiredMovement.speed, movementType);
            break;
        case REVERSE: //Reverse
            WARNING_MSG("Speed control done only in open loop currently. Check manually if the set speed matches your request.\r\n");
            desiredMovement.type=REVERSE;
            desiredMovement.speed=speed;
            if ((vel1) && (movementType!=desiredMovement.type))
            {
                DEBUG_MSG("Bringing vehicle to a standstill before moving in reverse. Please wait.\r\n");
                ROVIM_T2D_FullBrake();
            }
            if (movementType==desiredMovement.type)
            {
                //If we're already moving in the direction we want, just accelerate
                ROVIM_T2D_SetSpeed(desiredMovement.speed);
                STATUS_MSG("Vehicle moving backwards at approximately %d km/h/10.\r\n",desiredMovement.speed);
            }
            DEBUG_MSG("desired type=%d, speed=%d, current type=%d.\r\n",desiredMovement.type, desiredMovement.speed, movementType);
            break;
        default:
            ERROR_MSG("Unexpected argument value.\r\n");
            break;
    }
    
    return NoErr;
}

BYTE ROVIM_T2D_Turn(void)
{
    BYTE fullAngle=0;
    long centerAngle=0;
    short long y=0;
    long temp=0;
    
    if ((ARGN != 2) && (ARGN != 1))
    {
        return eNumArgsErr;
    }
    if (ARGN == 1)
    {
        SoftStop(2);
        STATUS_MSG("Direction motor stopped.\r\n");
        return NoErr;
    }
    
    fullAngle=ARG[1];
    if (fullAngle > ROVIM_T2D_DIR_ANGULAR_RANGE)
    {
        ERROR_MSG("Turn angle can only vary between 0º (full port) and %dº (full starboard).\r\n", ROVIM_T2D_DIR_ANGULAR_RANGE);
        return eParmErr;
    }
    if (!vel1)
    {
        ERROR_MSG("Vehicle must be moving when turning. Set the vehicle to move and retry.\r\n");
        return eErr;
    }
    
    if (!autoMode)
    {
        ERROR_MSG("You cannot move the vehicle with the micro controller while on manual mode.\r\n");
        return eParmErr;
    }
    /*Since the direction range is not evenly balanced (it turns more to one side than to the other
    we use the center angle for intermediate calculations, to get a more accurate result.
    Also, to the user, it's the angle relative to straight line movement the most important*/
    centerAngle=(long)ROVIM_T2D_DIR_ANGULAR_RANGE>>1;
    centerAngle=(long)fullAngle-centerAngle;
    if (centerAngle > 0)
    {
        STATUS_MSG("Turning vehicle %ldº to port (%ldº full angle).\r\n",centerAngle, (long)fullAngle);
    }
    else if (centerAngle == 0)
    {
        STATUS_MSG("Pointing vehicle in a straight line (%ldº full angle).\r\n", (long)fullAngle);
    }
    else
    {
        STATUS_MSG("Turning vehicle %ldº to starboard (%ldº full angle).\r\n", (long)abs(centerAngle),(long)fullAngle);
    }
    
    temp=(long) (ROVIM_T2D_DIR_TICK_UPPER_LIMIT-ROVIM_T2D_DIR_TICK_LOWER_LIMIT)*centerAngle/**2*/;
    temp=(long) temp/ROVIM_T2D_DIR_ANGULAR_RANGE/*/2*/;
    temp=(long) temp + ROVIM_T2D_DIR_CENTER_TICK_CNT;
    
    if(temp<ROVIM_T2D_DIR_TICK_LOWER_LIMIT)
    {
       DEBUG_MSG("target position out of lower bound. This may happen if the limits are unbalanced, \
or due to erroneous calculations. full turn angle=%ld, bound=%d.\r\n",temp, ROVIM_T2D_DIR_TICK_LOWER_LIMIT); 
        temp=ROVIM_T2D_DIR_TICK_LOWER_LIMIT;
    }
    if(temp>ROVIM_T2D_DIR_TICK_UPPER_LIMIT)
    {
        DEBUG_MSG("target position out of upper bound. This may happen if the limits are unbalanced, \
or due to erroneous calculations. full turn angle=%ld, bound=%d.\r\n",temp, ROVIM_T2D_DIR_TICK_UPPER_LIMIT); 
        temp=ROVIM_T2D_DIR_TICK_UPPER_LIMIT; 
    }
    y=(short long) temp;
    
    DEBUG_MSG("y=%ld, temp=%ld, ang range=%d, tick range=%d, angle=%d, VMID2=%d, ACC2=%d.\r\n", 
    (long)y, (long)temp, (BYTE) ROVIM_T2D_DIR_ANGULAR_RANGE, 
    (BYTE)(ROVIM_T2D_DIR_TICK_UPPER_LIMIT-ROVIM_T2D_DIR_TICK_LOWER_LIMIT), (BYTE) centerAngle, VMID2, ACC2);
    
    MoveMtrClosedLoop(2, y, VMID2, ACC2);
    
    return NoErr;
}

BYTE ROVIM_T2D_DebugControl(void)
{
    BYTE option=0;
    BYTE resetNotKick=0;
    BYTE accNotDcc=0;
    BYTE dutyCycle=0;
    BYTE verbosity=0;
    BYTE temp=0;
    BYTE scalingFactor=0;
    
    if (ARGN < 2)
    {
        return eNumArgsErr;
    }
    option=ARG[1];
    switch(option)
    {
        case 1: //Get verbosity level
            verbosity=GetVerbosity();
            FATAL_ERROR_MSG("Not really an error.\r\nCurrent verbosity=%d.\r\n", verbosity);
            break;
        case 2: //Set verbosity level
            if(ARGN != 3)
            {
                ERROR_MSG("You must specify the verbosity you want to set.\r\n");
                return eNumArgsErr;
            }
            verbosity=ARG[2];
            SetVerbosity(verbosity);
            FATAL_ERROR_MSG("Not really an error.\r\nCurrent verbosity=%d.\r\n", verbosity);
            break;
        case 3: //Toggle between automatic (default) and manual execution of the System monitoring task
            ManualSysMonitoring=~ManualSysMonitoring;
            if(ManualSysMonitoring)
            {
                STATUS_MSG("ROVIM T2D system monitoring done manually now. Use this command \
with the respective argument to run another time the T2D system monitoring task.\r\n");
            }
            else
            {
                STATUS_MSG("ROVIM T2D system monitoring reverted to default (automatic).\r\n");
            }
            break;
        case 4: //run the system monitor task, if in manual mode
            if(ManualSysMonitoring)
            {
                STATUS_MSG("Running ROVIM T2D system monitoring one time.\r\n");
                ROVIM_T2D_MonitorSystem();
            }
            else
            {
                ERROR_MSG("ROVIM T2D system monitoring is not set to manual mode.\r\n");
            }
            break;
        case 5: 
            if (ARGN == 5)
            {
                DebugPWM=ARG[2]+1;
                accNotDcc=ARG[3];
                dutyCycle=ARG[4];
                STATUS_MSG("Running ROVIM T2D PWM generator debug with period count %d. ACC?:%d, DCC?:%d, \
Dutycyle=%d.\r\n",(DebugPWM-1), accNotDcc, (!accNotDcc), dutyCycle);
                CMD='G';
                ARG[0]=accNotDcc?ROVIM_T2D_ACCELERATE_CMD_CODE:ROVIM_T2D_DECELERATE_CMD_CODE;
                ARG[1]=dutyCycle;
                ARGN=2;
                TeCmdDispatchExt();
                break;
            }
            else if (ARGN == 3)
            {
                DebugPWM=ARG[2]+1;
                STATUS_MSG("Running ROVIM T2D PWM generator debug with period count %d.\r\n",(DebugPWM-1));
                break;
            }
            ERROR_MSG("You have to specify the number of periods you want to see the PWM generator \
debug info and (optional), weather you want to accelerate or decelerate and the desired duty cycle.\
 The PWM generator is not controlled by this command, only it's debug information. You may have to\
 activate debug traces.\r\n");
            return eNumArgsErr;
        case 6: //Control the watchdog
            #ifdef WATCHDOG_ENABLED
            if (ARGN < 3)
            {
                ERROR_MSG("You must specify if you want to kick the watchdog or reset the system through it.\r\n");
                return eNumArgsErr;
            }
            resetNotKick=ARG[2];
            if(resetNotKick)
            {
                STATUS_MSG("Resetting system through watchdog.\r\n");
                HardReset();
            }
            else
            {
                STATUS_MSG("Kicking watchdog. Kick period=%d ms. For actual watchdog timer timeout period, consult config.h\r\n",WATCHDOG_PERIOD);
                KickWatchdog();
            }
            break;
            #else //WATCHDOG_ENABLED
            STATUS_MSG("Watchdog is disabled. Recompile.\r\n");
            break;
            #endif //WATCHDOG_ENABLED
        case 7: //Lock/Unlock resources
            ResourcesLockFlag=~ResourcesLockFlag;
            STATUS_MSG("Resources Lock Flag toggled to=%d.\r\n",ResourcesLockFlag);
            break;
        case 8: //Calibrate speed to duty cycle conversion
            if (ARGN == 3)
            {
                scalingFactor=ARG[2];
                SpeedDCScaling=scalingFactor;
                STATUS_MSG("Speed to duty cycle scaling factor set to %d.\r\n",(SpeedDCScaling/10) );
                break;
            }
            else
            {
                STATUS_MSG("Current speed to duty cycle scaling factor: %d.\r\nCurrent LUT:\r\n",(SpeedDCScaling/10) );
                /*for(i=0;i<SpeedToDutyCycleLen;i++)
                {
                    MSG("%d, ",SpeedToDutyCycle[i]);
                }
                MSG("\r\nLUT size=%d.\r\n",SpeedToDutyCycleLen);*/
                break;
            }
        case 9: //Manipulate error information print control variables
            ForcePrintMsg=TRUE;
            STATUS_MSG("Printing errors on periodic tasks one time.\r\n");
            break;
        default:
            ERROR_MSG("Option does not exist.\r\n");
            break;
    }
    return NoErr;
}
