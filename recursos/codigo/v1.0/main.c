#line 1 "main.c"          //work around the __FILE__ screwup on windows, http://www.microchip.com/forums/m746272.aspx
//cannot set breakpoints if this directive is used:
//info: http://www.microchip.com/forums/m105540-print.aspx
//uncomment only when breakpoints are no longer needed
/******************************************************************************
*******************************************************************************
**
**
**                  main.c
**
**      This is the main.c file of the Dalf-1F v1.73 firmware, modified
**      for the ROVIM project.
**
**      See Dalf-1F owner's manual and the ROVIM T2D documentation for more
**      details.
**
**          The ROVIM Project
**
**      This is a derivation from a file provided by Embedded Electronics, LLC
********************************************************************************
********************************************************************************/

/*xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;xx                                                                          xx
;xx           M       M        AAAAA       IIIIIII      N     N              xx
;xx           M M   M M       A     A         I         N N   N              xx
;xx           M   M   M       AAAAAAA         I         N  N  N              xx
;xx           M       M       A     A         I         N   N N              xx
;xx           M       M       A     A      IIIIIII      N     N              xx
;xx                                                                          xx
;xx     =================================================================    xx
;xx     This is the C Language "backbone" for the Dalf Motor Application.    xx
;xx     The code shown here includes: main loop, some power up init,         xx
;xx     interrupt service dispatch, and service requests initiated by the    xx
;xx     various interrupt service handlers.                                  xx
;xx                                                                          xx
;xx     The low level code (interrupt services, PID, Trajectory Generator,   xx
;xx     C library routines, various utilities) is written in PIC Assembly    xx
;xx     with the actual code located in the <dalf.lib> file.                 xx
;xx     =================================================================    xx
;xx                                                                          xx
;xx                                                                          xx
;xx                         Microchip tools                                  xx
;xx     +--------------------------------------------------------------+     xx
;xx     |  Tool        Ver        Description                          |     xx
;xx     |  --------   -----  ----------------------------------------  |     xx
;xx     |  MPLAB IDE  8.00   Integrated Development Environment        |     xx
;xx     |  MCC18      3.10   C Compiler for the PIC18F device family   |     xx
;xx     |  MPASM      5.10   Assembler                                 |     xx
;xx     |  MPLINK     4.10   Linker                                    |     xx
;xx     |  MPLIB      4.10   Librarian                                 |     xx
;xx     +--------------------------------------------------------------+     xx
;xx                                                                          xx
;xx                         - H I S T O R Y -                                xx
;xx                                                                          xx
;xx    DATE    WHO       DESCRIPTION                                         xx
;xx  --------  ---  -------------------------------------------------------  xx
;xx  09/15/06  SMB  Ver 1.40.  First production code release.                xx
;xx  11/30/06  SMB  Ver 1.50.  New Servo modes, analog feedback, et. al.     xx
;xx  06/08/07  SMB  Ver 1.60.  Serial comm fix.                              xx
;xx  06/28/07  SMB  Ver 1.62.  Beta for 1.70 Release.  Improved sync         xx
;xx  08/07/07  SMB  Ver 1.70.  New PotMix mode. First GUI compatible ver.    xx
;xx  11/05/07  SMB  Ver 1.71.  Minor changes to accomodate latest Microchip  xx
;xx                            tools (required for use with Version 1.71     xx
;xx                            development files).                           xx
;xx                                                                          xx
;xx   FILE: <main.c>                                                         xx
;xx                                                                          xx
;xx  (c) Copyright 2006 Embedded Electronics LLC, All rights reserved        xx
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
*/
#include    "p18f6722.h"                        /* Microcontroller specific definitions */
#include    "config.h"                          /* Fuse Settings */
#include    "dalf.h"
#include    <stdio.h>

//choose the configuration profile
#include    "rovim_t2d.h"                           /* description and configuration of the ROVIM system */

#ifdef DALF_TEST_ENABLED
    #include    "dalf_test.h"                           /* testing and debugging features */
#endif

/* Function Prototypes */
void    ISRHI ( void );
void    ISRLO ( void );
#ifdef DALF_ROVIM_T2D
void    INT1_ISR_SINGLE_SOURCE (void);
#endif //DALF_ROVIM_T2D
//void  Greeting(void);             // Terminal emulator greeting

    // Command Handling
void    SerialCmdDispatch(void);    // USART1 or I2C2 control
void    PotOrRc(BYTE mtr);          // Pot or RC control 
void    PotCcmd(BYTE mtr);          // Cmd: PotC
void    PotFcmd(BYTE mtr);          // Cmd: PotF
void    PotServoCmd(BYTE mtr);      // Cmd: PotServo
void    PotMixCmd(void);            // Cmd: Pot Mix
void    RcCmd(BYTE mtr);            // Cmd: R/C Normal
void    RcMixCmd(void);             // Cmd: R/C Mix
void    RcServoCmd(BYTE mtr);       // Cmd: R/C Servo 
void    CheckCmdTimeout(void);      // Check for Cmd Interface Timeout

    // Conversion 
BYTE    PotCtoPWM(BYTE adc);        // Map PotC position to PWM (0-100%)
BYTE    PotFtoPWM(BYTE adc);        // Map PotF position to PWM (0-100%)
                                    // Pulse width to +/-PWM [-100, 100]
int     RCtoPWM(WORD p, WORD rcmin, WORD rcmax, WORD rcm);
//WORD    AdcConvert(WORD Adc);       // ADC reading to millivolts
WORD    PulseConvert(WORD Pulse);   // PulseWidth ticks to microseconds
WORD    AdcToVbatt(WORD v6);        // AN6 mV to Vbatt mV

    // Move Motor 
//void    SoftStop(BYTE mtr);         // Stop Motor, leaving mode unchanged.
//void    MoveMtrOpenLoop(BYTE mtr, BYTE dir, BYTE spd, BYTE slew);
//void    MoveMtrClosedLoop(BYTE mtr, short long tgt, WORD v, WORD a);

    // Output
//int       printf(const rom char *fmt, ...);
void    ReturnData(void);       // Return data and/or status to cmd initiator.
void    DispCHR(void);          // xCHR 
void    DispE(void);            // Motor Position (encoder) 
void    DispV(void);            // Motor Velocity
void    DispADC(void);          // ADC Snapshot (AN0 .. AN6)
void    DispRC(void);           // RC Snapshot (Ch0 .. Ch2)
void    DispMEM(void);          // Single Memory Byte.
void    DispIO(void);           // Single IOEXP register.
void    DispSTAT(void);         // Motor Status
void    MtrStatTE(BYTE mtr);    // Motor Status Utility for TE
void    DispPID(void);          // Motor PID runtime parameters
void    DispHMS(void);          // HH:MM:SS (RTC Time)
void    DispExtEEBLOCK(void);   // Block of External EEPROM starting at 'pBYTE'
void    DispRAMBLOCK(void);     // Block of RAM starting at 'pBYTE' 
void    DispIntEEBLOCK(void);   // Block of Internal EEPROM starting at 'pBYTE' 
void    Tune1(void);            // Mtr1 PID Tuning Aid: CmdQ Verbose output 
void    Tune2(void);            // Mtr2 PID Tuning Aid: CmdQ Verbose output 

    // Miscellaneous
void    ResetPIC(void);         // Reset Dalf Board 
void    WinkLEDS(void);         // Briefly blink programmable LED's 
void    InitLED(void);          // Initialization for on-board LED's
void    ServiceLED(void);       // Periodic LED service

    // Over Current
void    Motor1Current(void);    // Voltage Window Transition - OverCurrent
void    Motor2Current(void);    // Voltage Window Transition - OverCurrent

    // Communication
void    GetApiPktCkSum(void);   // Compute and store Api Pkt CkSum byte
void    GetI2C2PktCkSum(void);  // Compute and store I2C2 Pkt CkSum Byte
void    SendApiPkt(void);       // Transmit API Pkt to Host
void    SendI2C2Pkt(void);      // Transmit I2C2 Pkt[] to Host
void    WriteSSP2BUF(BYTE chr); // Load I2C2 Transmit Buffer






// ****************************************************************************
// **             ERAM: Runtime environment copy of Parameter Block          **
// ****************************************************************************
extern BYTE fPWM, AD_ACQ, AD_CNV, AD_GAP, SYSMODE;
extern BYTE X1_IODIRA, X1_IOPOLA, X1_GPINTENA, X1_DEFVALA, X1_INTCONA;
extern BYTE X1_IOCON, X1_GPPUA, X1_GPIOA, X1_OLATA;
extern BYTE X1_IODIRB, X1_IOPOLB, X1_GPINTENB, X1_DEFVALB, X1_INTCONB;
extern BYTE X1_GPPUB, X1_GPIOB, X1_OLATB;
extern BYTE X2_IODIRA, X2_IOPOLA, X2_GPINTENA, X2_DEFVALA, X2_INTCONA;
extern BYTE X2_IOCON, X2_GPPUA, X2_GPIOA, X2_OLATA;
extern BYTE X2_IODIRB, X2_IOPOLB, X2_GPINTENB, X2_DEFVALB, X2_INTCONB;
extern BYTE X2_GPPUB, X2_GPIOB, X2_OLATB;

extern WORD VBCAL;
extern WORD VBWARN;

extern BYTE AMINP;
extern WORD MAXERR, MAXSUM;

extern BYTE MTR1_MODE1, MTR1_MODE2, MTR1_MODE3;
extern WORD ACC1, VMID1;
//extern BYTE VSP1;
extern WORD KP1, KI1, KD1;
//extern BYTE VMIN1, VMAX1;
//extern WORD TPR1;
extern WORD MIN1, MAX1;

// extern BYTE  MTR2_MODE1, MTR2_MODE2, MTR2_MODE3;
//extern WORD ACC2, VMID2;
//extern BYTE VSP2;
extern WORD KP2, KI2, KD2;
extern BYTE VMIN2, VMAX2;
extern WORD TPR2;
extern WORD MIN2, MAX2;

extern WORD RC1MIN, RC1MAX, RC2MIN, RC2MAX, RC3MIN, RC3MAX;
extern BYTE RCD;

extern BYTE POT1A, POT1B, POT2A, POT2B;

extern BYTE nBR, NID, RX1TO;
extern WORD NPID;
extern BYTE Dev_DALF;

extern BYTE RCSP;
extern BYTE PSP;
extern BYTE DMAX;

extern BYTE FENBL, DECAY;

extern BYTE CMDSP, CMDTIME;

extern BYTE ERAM7A;     //UNUSED//
extern BYTE ERAM7B;     //UNUSED//
extern BYTE ERAM7C;     //UNUSED//
extern BYTE ERAM7D;     //UNUSED//
extern BYTE ERAM7E;     //UNUSED//
extern BYTE ERAM7F;     //UNUSED//
//-----------------------------------------------------------






//*****************************************
//**    Other Assembler RAM Variables    **
//*****************************************
extern  BYTE    PIC_DEVID1, PIC_DEVID2;     // PIC Revision
extern  BYTE    BOARD_ID;                   // Board ID char
extern  BYTE    MAJOR_ID;                   // Software major ID byte
extern  BYTE    MINOR_ID;                   // Software minor ID byte
extern  WORD    USERID;                     // PIC_USERID[3..0] concatenated
//extern    BYTE    SCFG;                       // Serial Configuration (1..3)

extern  WORD    SERVICE, SERVICE_REQ;       // Requests from ISR's
extern  BYTE    TIMESVC, TIMESVC_REQ;       // Timed requests
//extern    BYTE    SECS, MINS, HOURS;      // RTC variables
//extern    ULONG   Seconds;                // Seconds since boot 
//extern  BYTE    ADC0[7];                    // ADC readings AN0..AN6
extern  BYTE    Pkt[134];                   // Max size (API) N+6 = 128+6 = 134
extern  WORD    DispSvc;                    // "Display" service requests
extern  BYTE    DataReq;                    // Arg used in servicing printf
extern  BYTE    xCHR;                       // Arg used in servicing printf
extern  BYTE    BlkLen;                     // Arg used in Memory Blk Read
extern  WORD    PulseWidth1;                // EX0 (units: 1.28 uSec)
extern  WORD    PulseWidth2;                // EX1 (units: 1.28 uSec)
extern  WORD    PulseWidth3;                // EX3 (Units: 1.28 uSec)
extern  BYTE    NewPulse;                   // R/C Signal loss detect bits
extern  WORD    RC1m,RC2m,RC3m;             // Slope: PWM% vs PWidth
extern  WORD    RC1M,RC2M;                  // Slope: ServoPos'n vs PWidth
extern  WORD    RC1center;
extern  WORD    RC2center;
extern  WORD    RC3center;                  // Pulse range center (uSec)

extern  WORD    Pid1Limit;                  // Motor1 Tuning Output Control
extern  WORD    npid1;                      // Motor1 PID Tuning Output count
extern  BYTE    Mtr1_Flags1;                // Motor1 flags1
extern  BYTE    Mtr1_Flags2;                // See dalf.h
//extern  BYTE    S1,Power1;                  // SPD: [0..100%], [0..VMAX1%]
//extern  short long  encode1;                // Mtr1 position encoder
//extern  short long  V1;                   // Mtr1 Velocity
extern  short long  x1pos;                  // Motor1 PID Target
extern  short long  Err1;                   // Motor1 PID Err
extern  short long  ErrDiff1;               // Motor1 PID Err Diff (dErr/dT)
extern  short long  ErrSum1;                // Motor1 PID Err Sum

extern  WORD    Pid2Limit;                  // Motor2 Tuning Output Control
extern  WORD    npid2;                      // Motor2 PID Tuning: Output count.
//extern  BYTE    Mtr2_Flags1;              // Motor2 flags1 
//extern    BYTE    Mtr2_Flags2;            // See dalf.h
//extern  BYTE    S2,Power2;                  // SPD: [0..100%], [0..VMAX2%]
extern  short long  encode2;                // Mtr2 position encoder
//extern  short long  V2;                   // Mtr2 Velocity
extern  short long  x2pos;                  // Motor2 PID Targets
extern  short long  Err2;                   // Motor2 PID Err
extern  short long  ErrDiff2;               // Motor2 PID Err Diff (dErr/dT)
extern  short long  ErrSum2;                // Motor2 PID Err Sum


//extern    BYTE    CMD,ARG[16],ARGN;
extern  BYTE    ReturnN;                    // Commmand Interface 
//extern    BYTE    CmdSource;

//extern  BYTE    LedErr;                     // "1" bits flag err cond'n

extern  BYTE    xbyte;                      // printf() arg - Hex Byte
extern  BYTE    ISR_Flags;                  // Flags set/cleared by ISR
extern  BYTE    I2C2_PktStatus;             // Flags for I2C2 Interface
extern  BYTE    I2C2_State;                 // SLAVE ISR State.

extern  BYTE    Cmd_Ticks;                  // Cmd Interface Timeout ticker


//*****************************************
//**          Other Variables            **
//*****************************************

#ifdef WATCHDOG_ENABLED
    WORD watchdogcount = WATCHDOG_PERIOD;
#endif

//**********************
//  Global Variables  **
//**********************
WORD    RC[3]={1500,1500,1500}; // R/C pulse widths in microseconds.
int     pwm1,pwm2;              // R/C pwm conversions.

BYTE    *pBYTE;                 // Memory pointer   
BYTE    serialcmd = FALSE;      // Flag: serial interface cmd received.
BYTE    PktLen;                 // Length of Pkt.

// LED variables
ULONG   ledshift;       // On/Off LED bit shifter (shifted bit is time period)
ULONG   grn1pattern;    // LED1: On/Off LED bit pattern
ULONG   grn2pattern;    // LED2: On/Off LED bit pattern 
ULONG   redpattern;     // LED3: On/Off LED bit pattern
BYTE    ledcount;       // LED Service Counter

// TIME variables
TIME    Delay1;

// Motor Control Modes
enum MotorModes
{
    POT_CENTER_OFF,     //0 PotC: [-100%, 100%] with center off.
    POT_WITH_SWITCH,    //1 PotF: [0%, 100%] with separate direction switch.
    RADIO_NORMAL,       //2 R/C Interface, Normal (TANK) mode.
    RADIO_MIX,          //3 R/C Interface, Mix Mode.
    RADIO_SERVO,        //4 R/C Servo Interface.
    POT_SERVO,          //5 POT Servo Interface.
    POT_MIX,            //6 POT Mix Mode.
    RS232               //7 Serial cmd interface.
};


//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
#pragma code INTHI = 0x808
//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
//----------------------------------------------------------------------------
// High priority interrupt vector
void IntVecHi (void)
{
  _asm
    goto ISRHI //jump to interrupt routine
  _endasm
}
//----------------------------------------------------------------------------


//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
#pragma code INTLO = 0x818
//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
//----------------------------------------------------------------------------
void IntVecLo (void)
{
  _asm
    goto ISRLO //jump to interrupt routine
  _endasm
}
//----------------------------------------------------------------------------


//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
#pragma code MAIN
//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
//-----------------------------------------------------------------------------
// EnableInterrupts - Configure and then enable active interrupts
//
//  Currently all supported interrupts are high priority interrupts.
//
//  CAVEAT EMPTOR: To reduce interrupt latency, none of the floating point
//      registers are saved during context save.  So, .. if you replace or
//      add interrupt handlers with ones that use these registers you will
//      need to change the #pragma directive for ISRHI().
//-----------------------------------------------------------------------------
void EnableInterrupts(void)
{
// Disable All Interrupts First //
    INTCONbits.GIEL = 0;
    INTCONbits.GIEH = 0;

    RCONbits.IPEN = 1;      // Enable Interrupt Priority Mode
//  INTCON2 = 0xFF;

///////////////////////////////
// Clear All Interrupt Flags //
///////////////////////////////
    INTCONbits.RBIF = 0;        // RB   - PORT B Change
    INTCONbits.TMR0IF = 0;      // TMR0
    INTCONbits.INT0IF = 0;      // INT0
    INTCON3bits.INT1IF = 0;     // INT1
    INTCON3bits.INT2IF = 0;     // INT2
    INTCON3bits.INT3IF = 0;     // INT3

    PIR1 = 0;                   // PSP  - Parallel Slave Port
                                // AD   - ADC
                                // RC1  - USART #1 Receive
                                // TX1  - USART #1 Transmit
                                // SSP1 - MSSP #1
                                // CCP1 - Capture/Compare/PWM #1
                                // TMR2 
                                // TMR1

    PIR2 = 0;                   // OSCF - Oscillator Fail
                                // CM   - Comparator
                                // UNUSED
                                // EE   - EEPROM/Flash Write
                                // BCL1 - MSSP #1 Bus Collision
                                // HLVD - High/Low Voltage Detect
                                // TMR3
                                // CCP2 - Capture/Compare/PWM #2

    PIR3 = 0;                   // SSP2 - MSSP #2
                                // BCL2 - MSSP #2 Bus Collision
                                // RC2  - USART #2 Receive
                                // TX2  - USART #2 Transmit
                                // TMR4
                                // CCP5 - Capture/Compare/PWM #5
                                // CCP4 - Capture/Compare/PWM #4
                                // CCP3 - Capture/Compare/PWM #3


//////////////////////////////////
// Configure interrupt priority //
//                              //
// NOTE: INT0 always HI         //
//////////////////////////////////
    INTCON2bits.TMR0IP = 1;     // TMR0: HI
    INTCON2bits.INT3IP = 1;     // INT3: HI
    INTCON2bits.RBIP = 1;       // RB:   HI

    INTCON3bits.INT2IP = 1;     // INT2: HI
    INTCON3bits.INT1IP = 1;     // INT1: HI


    IPR1 = 0xFF;                // PSP:  HI
                                // AD:   HI
                                // RC1:  HI
                                // TX1:  HI
                                // SSP1: HI
                                // CCP1: HI
                                // TMR2: HI
                                // TMR1: HI

    IPR2 = 0xFF;                // OSCF: HI
                                // CM:   HI
                                // UNUSED
                                // EE:   HI
                                // BCL1: HI
                                // HLVD: HI
                                // TMR3: HI
                                // CCP2: HI

    IPR3 = 0xFF;                // SSP2: HI
                                // BCL2: HI
                                // RC2:  HI
                                // TX2:  HI
                                // TMR4: HI
                                // CCP5: HI
                                // CCP4: HI
                                // CCP3: HI


///////////////////////////
// Configure Edge Detect //
///////////////////////////
    INTCON2bits.INTEDG0 = 1;    // INT0: Rising edge
    INTCON2bits.INTEDG1 = 1;    // INT1: Rising edge
    INTCON2bits.INTEDG2 = 1;    // INT2: Rising edge
    INTCON2bits.INTEDG3 = 1;    // INT3: Rising edge


////////////////////////////////
// Enable Selected Interrupts //
////////////////////////////////
    INTCONbits.TMR0IE = 1;      // TMR0: Heartbeat: 1ms

                                // INT0: AB2 - Mtr2 quadrature encoder
    if(MTR2_MODE1 & analogfbMsk)
        INTCONbits.INT0IE = 0;
    else INTCONbits.INT0IE = 1;

                                // INT1: AB1 - Mtr1 quadrature encoder
    if(MTR1_MODE1 & analogfbMsk)
        INTCON3bits.INT1IE = 0;
    else INTCON3bits.INT1IE = 1;

    if(MTR2_MODE3 & OcieMsk)    // INT2: I2  - Mtr2 current sense
        INTCON3bits.INT2IE = 1;
    else INTCON3bits.INT2IE=0;

    if(MTR1_MODE3 & OcieMsk)    // INT3: I1  - Mtr1 current sense
        INTCON3bits.INT3IE = 1;
    else INTCON3bits.INT3IE=0;

    INTCONbits.RBIE = 0;        // RB:   EX2


    PIE1bits.PSPIE = 0;         // PSP:  UNUSED
    PIE1bits.ADIE = 0;          // AD:   UNUSED
    PIE1bits.RC1IE = 1;         // RC1:  RC1 - Cmd Interface
    PIE1bits.TX1IE = 1;         // TX1:  TX1 - Cmd Interface
    PIE1bits.SSP1IE = 0;        // SSP1: UNUSED
    PIE1bits.CCP1IE = 1;        // CCP1: EX0 Pulse capture (R/C Mtr1)
    PIE1bits.TMR2IE = 1;        // TMR2: ADC State Machine
    PIE1bits.TMR1IE = 1;        // TMR1: RTC; Timed delays

    PIE2bits.OSCFIE = 0;        // OSCF: UNUSED
    PIE2bits.CMIE = 0;          // CM:   UNUSED
                                // UNUSED
    PIE2bits.EEIE = 0;          // EE:   UNUSED
    PIE2bits.BCL1IE = 0;        // BCL1: UNUSED
    PIE2bits.HLVDIE = 0;        // HLVD: UNUSED
    PIE2bits.TMR3IE = 0;        // TMR3: Capture/Compare - All CCP Modules
    PIE2bits.CCP2IE = 1;        // CCP2: EX1 Pulse capture (R/C Mtr2)

    PIE3bits.SSP2IE = 1;        // SSP2: Secondary I2C bus.
    PIE3bits.BCL2IE = 0;        // BCL2: UNUSED
    PIE3bits.RC2IE = 0;         // RC2:  UNUSED
    PIE3bits.TX2IE = 0;         // TX2:  UNUSED
    PIE3bits.TMR4IE = 0;        // TMR4: PWM - All CCP Modules
    PIE3bits.CCP5IE = 0;        // CCP5: PWM1 - Mtr1 speed control
    PIE3bits.CCP4IE = 0;        // CCP4: PWM2 - Mtr2 speed control
    PIE3bits.CCP3IE = 0;        // CCP3: EX3


    TXSTA1bits.TXEN = 1;        // USART1: Tx enable (sets TX1IF)
    INTCON2bits.RBPU = 1;       // RB pull-ups disabled.

//////////////////////////////
// Enable Global Interrupts //
//////////////////////////////
    INTCONbits.GIEL = 0;    // Disable Low priority interrupts
    INTCONbits.GIEH = 1;    // Enable high priority interrupts
}
//-----------------------------------------------------------------------------


///////////////////////////////////////////////////////////////////////////////
// _user_putc - Single char output in an application defined manner
//
//      Printf (et. al.) to an "output stream".  MCC18 defines 2 such streams:
//
//             _H_USER -  Output via user-defined function _user_putc.
//             _H_USART - Output via library output function _usart_putc.
//
//      By defining _user_putc, Printf funnels all its output thru this fcn.
//      The use of TxChr() in this way allows all USART1 output (including that
//      from the assembler) to be interrupt driven using a common transmit
//      buffer (Tx1_Buff[]).
//-----------------------------------------------------------------------------
int _user_putc (char c)
{
    return ( (int) TxChr(c) );  
}
//-----------------------------------------------------------------------------
void    Greeting(void)
{
    if(SCFG == TEcfg) 
    { // If Terminal Emulator Interface
        printf("\r\n");
        printf("Dalf-1%c\r\n", BOARD_ID);                           // Hardware ID
        printf("18F6722 Rev:%02X\r\n", (PIC_DEVID1 & 0x1F));        // Micro ID
        printf("Software Ver:%2u.%02u\r\n",MAJOR_ID, MINOR_ID);     // Software ID
        printf("User: %05u\r\n",USERID);                            // User ID
    }
}
//-----------------------------------------------------------------------------
void    SerialCmdDispatch(void)
{ // If a serial command has been received, try to execute it.
    BYTE err;

    if(serialcmd)
    { // If cmd received by one of serial interfaces
        //-------------------------------------
        if( CmdSource == API_SrcMsk )
        { // If API
            err = ApiCmdDispatch();
        }
        //-------------------------------------
        else if (CmdSource == TE_SrcMsk)
        { // Terminal Emulator Interface
            err = TeCmdDispatchExt();
            if (err)
            {
                if (err==eParseErr)     printf("Parse Err\r\nEnter 'H' for help\r\n");
                if (err==eNumArgsErr)   printf("#Args Err\r\nEnter 'H' for help\r\n");
                if (err==eParmErr)      printf("Parm Err\r\nEnter 'H' for help\r\n");
                if (err==eModeErr)      printf("Mode Err\r\n");
                if (err==eDisable)      printf("Disabled\r\n");
            }
        }
        //-------------------------------------
        else if (CmdSource == I2C2_SrcMsk)  err = I2C2CmdDispatchExt();
        //-------------------------------------

        serialcmd=FALSE;        // reset flag
        if(!err) Cmd_Ticks = CMDTIME;   // Reset Cmd Interface Timeout ticker
    }
}
//-----------------------------------------------------------------------------
void    PotOrRc(BYTE mtr)
{ // Does nothing unless RC or POT mode and service is due (TIMESVC).
    BYTE mode2, MtrMode;


    if(mtr==1)  mode2=MTR1_MODE2;   else mode2=MTR2_MODE2;
    if((mode2 & ManualMsk)==0) MtrMode = RS232; // If not RC or POT
    else
    {
        if(mode2 & PotcMsk)         MtrMode = POT_CENTER_OFF;
        if(mode2 & PotfMsk)         MtrMode = POT_WITH_SWITCH;
        if(mode2 & RcNrmMsk)        MtrMode = RADIO_NORMAL;
        if(mode2 & RcMixMsk)        MtrMode = RADIO_MIX;
        if(mode2 & RcServoMsk)      MtrMode = RADIO_SERVO;
        if(mode2 & PotServoMsk)     MtrMode = POT_SERVO;
        if(mode2 & PotMixMsk)       MtrMode = POT_MIX;
    }

    switch(MtrMode)
    {
    //----------------------------------------------------
    case POT_CENTER_OFF:                            // Open Loop
        if(TIMESVC & PspMsk)    PotCcmd(mtr);
        break;
    //----------------------------------------------------
    case POT_WITH_SWITCH:                           // Open Loop
        if(TIMESVC & PspMsk)    PotFcmd(mtr);
        break;
    //----------------------------------------------------
    case POT_SERVO:                                 // Closed Loop
        if(TIMESVC & PspMsk)    PotServoCmd(mtr);
        break;
    //----------------------------------------------------
    case RADIO_NORMAL:                              // Open Loop
        if(TIMESVC & RcspMsk)   RcCmd(mtr);
        break;
    //----------------------------------------------------
    case RADIO_MIX:                                 // Open Loop
        if((TIMESVC & RcspMsk) && (mtr==1)) RcMixCmd();
        break;
    //----------------------------------------------------
    case RADIO_SERVO:                               // Closed Loop
        if(TIMESVC & RcspMsk)   RcServoCmd(mtr);
        break;
    //----------------------------------------------------
    case POT_MIX:                                   // Open Loop
        if((TIMESVC & PspMsk) && (mtr==1))  PotMixCmd();
        break;
    //----------------------------------------------------
    default:
        break;
    }
}
//-----------------------------------------------------------------------------
void    PotCcmd(BYTE mtr)
{
    BYTE dir,spd,adc;

    CmdSource = POT_SrcMsk;
    if(SWITCH0)
    {
        if(mtr==1) adc=ADC0[0]; else adc=ADC0[1];
        spd = PotCtoPWM(adc);
        if((spd) && !(adc&0x80)) dir=REVERSE;   // If 0<spd  AND adc<0x080
        else dir=FORWARD;                       // else
        MoveMtrOpenLoop(mtr, dir, spd, AMINP);
    }
    else    SoftStop(mtr);      // Schedule Motor Stop
}
//-----------------------------------------------------------------------------
void    PotFcmd(BYTE mtr)
{
    BYTE dir,spd,adc;

    CmdSource = POT_SrcMsk;
    if(SWITCH0)
    {
        if(mtr==1) adc=ADC0[0]; else adc=ADC0[1];
        spd = PotFtoPWM(adc);
        if( spd && !SWITCH1) dir=REVERSE;   // if 0<spd AND sw1=0
        else dir=FORWARD;                   // else
        MoveMtrOpenLoop(mtr, dir, spd, AMINP);
    }
    else    SoftStop(mtr);      // Schedule Motor Stop
}
//-----------------------------------------------------------------------------
void PotServoCmd(BYTE mtr)
{
    ///////////////////////////////////////////////////////////////////////////////
    // If enabled by switch (PORTD.0), ...                                       //
    //   Map ADC reading "pot" in [0..255] to y position in [MIN..MAX] and use   //
    //   PID to move to position y.  Initial move uses Trajectory Generator for  //
    //   smooth startup.  After that, repeated PID w/o err sum term.             //
    ///////////////////////////////////////////////////////////////////////////////
    BYTE pot,state,flags;
    short long y;
    WORD min,max,vmid,acc;

    if(mtr==1)
    { // If Motor1
        min=MIN1; max=MAX1; vmid=VMID1; acc=ACC1;
        flags=Mtr1_Flags1; pot=ADC0[0];
    }
    else
    { // Else Motor2
        min=MIN2; max=MAX2; vmid=VMID2; acc=ACC2;
        flags=Mtr2_Flags1; pot=ADC0[1];
    }

    if(SWITCH0)
    { // If Enabled by external switch
        y = max - min;
        y = (y*pot)/255 + min;          // target position

        state=0;
        if( flags & pida_Msk ) state+=1;    // If PID active
        if( flags & tga_Msk ) state+=2;     // If TGA active
        switch(state)
        {
            case 0: // tga OFF, pid OFF
                CmdSource = POT_SrcMsk;
                MoveMtrClosedLoop(mtr, y, vmid, acc);
                break;

            case 1: // tga OFF, pid ON
                if(mtr==1)
                { // If Motor1
                    x1pos=y;                    // New pid target
                    Mtr1_Flags1 |= sumhoa_Msk;  // Don't accumulate PID Err Sums
                }
                else
                { // Else Motor2
                    x2pos=y;
                    Mtr2_Flags1 |= sumhoa_Msk;
                }
                break;

            case 2: // tga ON, pid OFF
                break;                      // Just exit

            case 3: // tga ON, pid ON
                break;                      // Exit. Let 1st target be reached.
            default:
                break;
        } // state
    } // SWITCH0
    else
    {
        CmdSource = POT_SrcMsk;
        SoftStop(mtr);      // Schedule Motor Stop
    }
}
//-----------------------------------------------------------------------------
void    RcCmd(BYTE mtr)
{
    ///////////////////////////////////////////////////////////////////
    //  Maps measured pulse width pw to open loop motor control      //
    //  parameters SPD and DIR which are used to issue the motor     //
    //  open loop command.  AMINP is used as the slew rate.  The     //
    //  mapping is the normal one (sometimes called "tank mode").    //
    //                                                               //
    //  RCm is the precomputed mapping slope scaled by 1024.  See    //
    //  RCtoPWM() for details.                                       //
    ///////////////////////////////////////////////////////////////////
    BYTE dir, spd;
    WORD rcmin,rcmax,slope,pw;
    int pwm;

    //////////////////////////////////////////////////////
    //  First a bit of setup and signal loss detection  //
    //////////////////////////////////////////////////////
    if(mtr==1)
    { // If Motor1
        rcmin=RC1MIN; rcmax=RC1MAX; slope=RC1m; pw=RC[0];
        if((NewPulse & NewRC1msk) == 0)     // If Signal Loss, ..
        {
            // Schedule Mtr1 Stop
            CmdSource = RC_SrcMsk;
            SoftStop(0x01);
            LedErr |= SL1msk;       // Record in LedErr
            return;                 // .. and exit.
        }
        else
        {
            NewPulse &= ~NewRC1msk; // Reset NewPulse.RC1
            LedErr &= ~SL1msk;      // Reset also in LedErr
        }
    }
    else
    { // Else Motor2
        rcmin=RC2MIN; rcmax=RC2MAX; slope=RC2m; pw=RC[1];
        if((NewPulse & NewRC2msk) == 0)     // If Signal Loss, ..
        {
            // Schedule Mtr2 Stop
            CmdSource = RC_SrcMsk;
            SoftStop(0x02);
            LedErr |= SL2msk;       // Record in LedErr
            return;                 // .. and exit.
        }
        else
        {
            NewPulse &= ~NewRC2msk; // Reset NewPulse
            LedErr &= ~SL2msk;      // Reset also in LedErr
        }
    }

    
    // Get pwm value [-100, 100] corresponding to pulse width.
    pwm = RCtoPWM(pw, rcmin, rcmax, slope);

    // Convert pwm value into DIR and SPD control parameters.
    if (pwm < 0) {dir = REVERSE; spd = -pwm;}
    else {dir = FORWARD; spd = pwm;}

    // Issue Mtr Move Cmd (Open Loop) 
    CmdSource = RC_SrcMsk;
    MoveMtrOpenLoop(mtr, dir, spd, AMINP);
}
//-----------------------------------------------------------------------------
void    RcMixCmd(void)
{
    /////////////////////////////////////////////////////////////////
    //  Maps measured pulse widths RC[0], RC[1] to corresponding   //
    //  global pwm values pwm1 and pwm2.  These are then "mixed"   //
    //  to produce open loop motor command parameters for both     //
    //  motors.  AMINP is used as the slew rate.                   //
    //                                                             //
    //  Parms: RC1MIN, RC1MAX, and RC1m affect the pwm1 mapping.   //
    //  Parms: RC2MIN, RC2MAX, and RC2m affect the pwm2 mapping.   //
    //                                                             //
    //  Just exits if either motor not in RCMIX mode.              //
    /////////////////////////////////////////////////////////////////
    BYTE dir1, spd1, dir2, spd2;
    int pwm;

    //////////////////////////////////////////////////////
    //  First a bit of setup and signal loss detection  //
    //////////////////////////////////////////////////////
    if(  ((NewPulse & NewRC1msk)==0) || ((NewPulse & NewRC2msk) == 0) )
    { // If Signal Loss, ...
        if((NewPulse & NewRC1msk)==0)   LedErr |=SL1msk;    // Record in LedErr
        if((NewPulse & NewRC2msk)==0)   LedErr |=SL2msk;
        CmdSource = RC_SrcMsk;
        SoftStop(0x01);         // Schedule stop of Mtr1
        SoftStop(0x02);         // Schedule stop of Mtr2
        return;                 // .. and exit.
    }
    else
    {
        NewPulse &= ~(NewRC1msk + NewRC2msk);   // Reset NewPulse.(RC1+RC2)
        LedErr &= ~(SL1msk + SL2msk);           // Reset also in LedErr
    }
    if(!(MTR1_MODE2 & RcMixMsk))    return;     // Just exit if not RCMIX.
    if(!(MTR2_MODE2 & RcMixMsk))    return;     // Just exit if not RCMIX.


    // Get pwm values [-100, 100] corresponding to pulse widths.
    pwm1 = RCtoPWM(RC[0], RC1MIN, RC1MAX, RC1m);
    pwm2 = RCtoPWM(RC[1], RC2MIN, RC2MAX, RC2m);

    // Convert pwm values into DIR and SPD control parameters.
    pwm = pwm1 - pwm2;
    if (pwm < -100) pwm = -100;
    if (pwm > 100) pwm = 100;
    if (pwm < 0) {dir1 = REVERSE; spd1 = -pwm;}
    else {dir1 = FORWARD; spd1 = pwm;}

    pwm = pwm1 + pwm2;
    if (pwm < -100) pwm = -100;
    if (pwm > 100) pwm = 100;
    if (pwm < 0) {dir2 = REVERSE; spd2 = -pwm;}
    else {dir2 = FORWARD; spd2 = pwm;}

    // Issue Mtr1 Move Cmd (Open Loop) 
    CmdSource = RC_SrcMsk;
    MoveMtrOpenLoop(0x01, dir1, spd1, AMINP);

    // Issue Mtr2 Move Cmd (Open Loop) 
    CmdSource = RC_SrcMsk;
    MoveMtrOpenLoop(0x02, dir2, spd2, AMINP);
}
//-----------------------------------------------------------------------------
void PotMixCmd(void)
{
    ///////////////////////////////////////////////////////////////////
    //  Maps measured adc values ADC0[0], ADC1[1] to corresponding   //
    //  pwm values pwm1 and pwm2.  These values are then "mixed"     //
    //  to produce open loop motor command parameters for both       //
    //  motors.  AMINP is used as the slew rate.                     //
    //                                                               //
    //  NOTES:                                                       //
    //  1) Just exits if either motor not in POTMIX mode.            //
    //  2) Configure Mtr1 as right motor as seen from drivers seat.  //
    ///////////////////////////////////////////////////////////////////
    BYTE dir1, spd1, dir2, spd2, adc;
    int pwm;

    if(!(MTR1_MODE2 & PotMixMsk))   return;     // Just exit if not POTMIX.
    if(!(MTR2_MODE2 & PotMixMsk))   return;     // Just exit if not POTMIX.

    /////////////////////////////////////////////////////////////
    //  Capture Pot readings and convert to [-100%, 100%] pwm  //
    /////////////////////////////////////////////////////////////
    adc = ADC0[0];
    pwm1 = PotCtoPWM(adc);              // [0,100%]
    if(adc<0x80) pwm1 = -pwm1;          // pwm1: [-100%, 100%]
    adc = ADC0[1];
    pwm2 = PotCtoPWM(adc);              // [0,100%]
    if(adc<0x80) pwm2 = -pwm2;          // pwm2: [-100%, 100%]

    //////////////////////////////////////////////////////////////
    // Convert pwm values into DIR and SPD control parameters.  //
    //////////////////////////////////////////////////////////////
    pwm = pwm1 - pwm2;
    if (pwm < -100) pwm = -100;
    if (pwm > 100) pwm = 100;
    if (pwm < 0) {dir1 = REVERSE; spd1 = -pwm;}
    else {dir1 = FORWARD; spd1 = pwm;}

    pwm = pwm1 + pwm2;
    if (pwm < -100) pwm = -100;
    if (pwm > 100) pwm = 100;
    if (pwm < 0) {dir2 = REVERSE; spd2 = -pwm;}
    else {dir2 = FORWARD; spd2 = pwm;}

    if(SWITCH0)
    {   // Safety check: On/Off switch
        // Issue Mtr1 Move Cmd (Open Loop) 
        CmdSource = POT_SrcMsk;
        MoveMtrOpenLoop(0x01, dir1, spd1, AMINP);

        // Issue Mtr2 Move Cmd (Open Loop) 
        CmdSource = POT_SrcMsk;
        MoveMtrOpenLoop(0x02, dir2, spd2, AMINP);
    }
    else
    {
        CmdSource = POT_SrcMsk;
        SoftStop(1);        // Schedule Motor Stop
        CmdSource = POT_SrcMsk;
        SoftStop(2);        // Schedule Motor Stop
    }
}
//-----------------------------------------------------------------------------
void RcServoCmd(BYTE mtr)
{
    /////////////////////////////////////////////////////////////////////////
    //  Maps measured R/C pulse width pw= in [RCMIN..RCMAX] to y in        //
    //  in [MIN..MAX] and use PID to move motor to position y.             //
    //                                                                     //
    //  Initial move uses Trajectory Generator for smooth startup.         //
    //  After that, repeated PID w/o err sum term.                         //
    //                                                                     //
    //             slope = 100*(MAX-MIN)/RCMAX-RCMIN)                      //
    //                                                                     //
    //  Precomputed mapping slope scaled by 100.  Avoids some unnecessary  //
    //  runtime computations.                                              //
    /////////////////////////////////////////////////////////////////////////
    BYTE state, flags;
    WORD pw,min,rcmax,rcmin,vmid,acc,slope;
    short long y;

    //////////////////////////////////////////////////////
    //  First a bit of setup and signal loss detection  //
    //////////////////////////////////////////////////////
    if(mtr==1)
    { // If Motor1
        min=MIN1; rcmax=RC1MAX; rcmin=RC1MIN; vmid=VMID1; acc=ACC1;
        flags=Mtr1_Flags1; slope=RC1M;  pw=RC[0];

        if((NewPulse & NewRC1msk) == 0)     // If Signal Loss, ..
        {
            CmdSource = RC_SrcMsk;
            SoftStop(0x01);         // Schedule stop of Mtr
            LedErr |= SL1msk;       // Record in LedErr
            return;                 // ... and exit
        }
        else
        {
            NewPulse &= ~NewRC1msk; // Reset NewPulse.RC1
            LedErr &= ~SL1msk;      // Reset also in LedErr
        }
    }
    else
    { // Else Motor2
        min=MIN2; rcmax=RC2MAX; rcmin=RC2MIN; vmid=VMID2; acc=ACC2;
        flags=Mtr2_Flags1;  slope=RC2M; pw=RC[1];

        if((NewPulse & NewRC2msk)==0)       // If Signal Loss, ..
        {
            CmdSource = RC_SrcMsk;
            SoftStop(0x02);         // Schedule stop of Mtr
            LedErr |= SL2msk;       // Record in LedErr
            return;                 // ... and exit
        }
        else
        {
            NewPulse &= ~NewRC2msk; // Reset NewPulse.RC2
            LedErr &= ~SL2msk;      // Reset also in LedErr
        }
    }


    // Clip pulse width to [RCMIN..RCMAX] range.
    if(pw < rcmin) pw=rcmin;
    if(pw > rcmax) pw=rcmax;
    y = (slope*(pw - rcmin)/100)+min;       // target position

    // Mapping and dispatch based on PID and TG
    state=0;
    if( flags & pida_Msk )  state+=1;       // if PID active
    if( flags & tga_Msk )   state+=2;       // if TG active
    switch(state)
    {
        case 0: // tga OFF, pid OFF
            CmdSource = RC_SrcMsk;
            MoveMtrClosedLoop(mtr, y, vmid, acc);
            break;

        case 1: // tga OFF, pid ON
            if(mtr==1)
            { // If Motor1
                x1pos=y;                    // New pid target
                Mtr1_Flags1 |= sumhoa_Msk;  // Don't accumulate PID Err Sums
            }
            else
            { // Else Motor2
                x2pos=y;
                Mtr2_Flags1 |= sumhoa_Msk;
            }
            break;

        case 2: // tga ON, pid OFF
            break;                      // Just exit

        case 3: // tga ON, pid ON
            break;                      // Exit. Let 1st target be reached.
        default:
            break;
    } // state
}
//-----------------------------------------------------------------------------
void CheckCmdTimeout(void)
{
    //////////////////////////////////////////////////////////////
    //  Check for timeout on the Serial CMD Interfaces          //
    //                                                          //
    //  (Don't stop motor if in one of the R/C or POT modes)    //
    //                                                          //
    //  If timeout: Stop motors not under POT or R/C controls.  //
    //              Reset timeout counter.                      //
    //                                                          //
    //        Else: Decrement timeout counter.                  //
    //////////////////////////////////////////////////////////////
    if(SYSMODE & CmdToMsk)  // If timeout feature enabled
    {
        if(Cmd_Ticks == 0)
        {
            if(!(MTR1_MODE2 & ManualMsk)) // if not R/C or POT controlled
            {
                CmdSource = TE_SrcMsk;
                SoftStop(1);
            }
            if(!(MTR2_MODE2 & ManualMsk)) // if not R/C or POT controlled
            {
                CmdSource = TE_SrcMsk;
                SoftStop(2);
            }
            Cmd_Ticks = CMDTIME;
        }
        else Cmd_Ticks--;
    }
}
//-----------------------------------------------------------------------------






//-----------------------------------------------------------------------------
BYTE    PotCtoPWM(BYTE adc)
{
    WORD y=212;
    ///////////////////////////////////////////////////////////////////////
    // Maps pot ADC reading into 0-100% speed assuming center position   //
    // adc = 0x7F should be mapped to 0% with speed increasing linearly  //
    // to either side of zero.  With a deadband around center reading    //
    // of +/- 0x06, the slope is                                         //
    //                                                                   //
    //                 Slope = 100/(121) ~ 212/256                       //
    ///////////////////////////////////////////////////////////////////////
    if(adc>0x85) y=(y*(adc-0x085))>>8;
    else if(adc<0x7A) y=(y*(0x7A-adc))>>8;
    else    y=0;
    if(y>100) y=100;        // clip to full range.
    return  (BYTE) y;
}
//-----------------------------------------------------------------------------
BYTE    PotFtoPWM(BYTE adc)
{
    WORD y=103;
    ///////////////////////////////////////////////////////////////////////
    // Maps pot ADC reading into 0-100% speed assuming full range on the //
    // pot reading [0..255] maps to [0..100%] speed setting.  A separate //
    // switch controls direction.                                        //
    //                                                                   //
    // adc is mapped linearly to full pwm range.                         //
    // Slope = 100/249 ~ 103/256                                         //
    //                                                                   //
    // Also provides deadband around 0x00 reading of + 0x06              //
    ///////////////////////////////////////////////////////////////////////
    if(adc>0x06) y=(y*(adc-0x06))>>8;
    else y=0;
    if(y>100) y=100;        // clip to full range.
    return  (BYTE) y;
}
//-----------------------------------------------------------------------------
int RCtoPWM(WORD p, WORD rcmin, WORD rcmax, WORD rcm)
{
    ///////////////////////////////////////////////////////////////////
    //       p = measured pulse width (microseconds)                 //
    //       rcmin = minimum pulse width (microseconds)              //
    //       rcmax = maximum pulse width (microseconds)              //
    //       rcm = scaled mapping slope of PWM% vs pulse width       //
    //                                                               //
    //         Returns signed integer in range [-100, 100].          //
    //                                                               //
    //  rcm is the precomputed mapping slope scaled by 1024.         //
    //  Usage avoids some unnecessary runtime computations.          //
    //                                                               //
    //          rcm = [200/(rcmax - rcmin - d)]*1024                 //
    //                                                               //
    //            d = RCD (R/C Deadband parameter)                   //
    ///////////////////////////////////////////////////////////////////
    long z1;
    WORD cx,dx;
    int rtnval;

    if (p<rcmin) return(-100);
    if (p>rcmax) return(100);

    cx = (rcmin+rcmax)>>1;      // cx = Center of expected pulse width range
    dx = RCD>>1;                // dx = deadband/2
    if ( p<(cx-dx) )
    { // return( [rcm/1024]*(p-rcmin) - 100 )
        z1 = p-rcmin;
        z1 = z1*(long)rcm;
        z1 = z1>>10;                    // unscale
        rtnval = (int)z1 - 100;
        if (rtnval<-100) return(-100);
        return(rtnval);
    }
    else if ( p>(cx+dx) )
    { // return( [rcm/1024]*(p-(cx+dx)) )
        z1 = p - (cx+dx);
        z1 = z1*(long)rcm;
        z1 = z1>>10;                    // unscale
        rtnval = (int)z1;
        if (rtnval>100) return(100);
        return(rtnval);
    }
    else return(0);
}
//-----------------------------------------------------------------------------
WORD AdcConvert(WORD Adc)
////////////////////////////////////////////////////////////////////
//  Converts ADC reading [0..255] to [0..5000] millivolts.        //
//      V(mV) = (5000/255)*AdcReading.                            //
//      Note: 5000/255 = 19.607843.. = 19 + 155/256 + 0.002374..  //
////////////////////////////////////////////////////////////////////
{
    return (19*Adc + ((155*Adc)>>8));
}
//-----------------------------------------------------------------------------
WORD PulseConvert(WORD Pulse)
{
////////////////////////////////////////////////////////////////////
//  Converts Pulse Width to units of 1.00 uSec                    //
//      (Argument is in units of 0.80 uSec)                       //
//                                                                //
//  Pulse_uSec = 0.80 * PulseWidth.                               //
//             = (204/256 + 204/256^2 + 0.000012..) * PulseWidth  //
//                                                                //
////////////////////////////////////////////////////////////////////
    unsigned short long p=Pulse;
    p=p*204;
    return( (WORD)((p>>8) + (p>>16)) );
}
//-----------------------------------------------------------------------------
WORD AdcToVbatt(WORD v6)
{
    //////////////////////////////////////////////////////////////////
    //          Compute actual VBATT voltage (millivolts)           //
    //                                                              //
    // VBATT volt divider constant:  r=R3/(R3+R4)=0.130435          //
    // is used to translate voltage V6 measured by ADC on AN6       //
    // into actual VBATT voltage.                                   //
    //                                                              //
    //                VBATT = (1/r) * V6                            //
    //                                                              //
    //   Note: 1/r ~ 7.666.. = 7 + 170/256 + 0.002604..             //
    //                                                              //
    //   To deal with resistor variability, the scaled calibration  //
    //   constant VBCAL is stored in the Parameter Block.           //
    //                                                              //
    //           VBCAL = 256 * (1/r)  [default: 0x07AA]             //
    //                                                              //
    //   Note: VBCAL adjustment in the Parameter Block enables fine //
    //   tuning in this computation of the VBATT voltage.           //
    //////////////////////////////////////////////////////////////////
    unsigned short long temp;
    temp = v6;                          // V6 (mv)
    return( (WORD)((VBCAL*temp)>>8) );  // Multiply and unscale
}
//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
void SoftStop(BYTE mtr)
{
    /////////////////////////////////////////////////////////////
    // Schedules ramped stop of the motor.                     //
    //                                                         //
    // This function uses Cmd_X to stop the motor.  Unlike     //
    // Cmd_O, it will not disable R/C and POT mode operations  //
    // provided that CmdSource is defined properly.            //
    /////////////////////////////////////////////////////////////
    #ifdef DALF_ROVIM_T2D
    if(mtr==3)
    {
        //Stop traction motor and set it on hill hold
        CMD = 'G';
        ARG[0] = ROVIM_T2D_SET_MOVEMENT_CMD_CODE;
        ARGN = 1;
        TeCmdDispatchExt();
        return;
    }
    #endif  //DALF_ROVIM_T2D
    CMD = 'X';
    ARG[0] = mtr;
    ARG[1] = FORWARD;
    ARG[2] = SPEEDZERO;
    ARG[3] = AMINP;
    ARGN = 0x04;
    TeCmdDispatchExt();
}
//-----------------------------------------------------------------------------
void MoveMtrOpenLoop(BYTE mtr, BYTE dir, BYTE spd, BYTE slew)
{
    CMD = 'X';
    ARG[0] = mtr;
    ARG[1] = dir;
    ARG[2] = spd;
    ARG[3] = slew;
    ARGN = 0x04;
    TeCmdDispatchExt();
}
//-----------------------------------------------------------------------------
void MoveMtrClosedLoop(BYTE mtr, short long tgt, WORD v, WORD a)
{
    CMD = 'Y';
    ARG[0] = mtr;
    ARG[1] = (tgt & 0xFF0000)>>16;
    ARG[2] = (tgt & 0xFF00)>>8;
    ARG[3] = tgt & 0xFF;
    ARG[4] = (BYTE) (v>>8);
    ARG[5] = (BYTE) (v & 0xFF);
    ARG[6] = (BYTE) (a>>8);
    ARG[7] = (BYTE) (a & 0xFF);
    ARGN = 0x08;
    TeCmdDispatchExt();
}
//-----------------------------------------------------------------------------

void ReturnData(void)
{   // Return data and/or status to command initiator.
    if  (DispSvc)
    { // If anything to do ...
    ///////////////////////////////////////////////////////////////////////////
    //                     - -  C A U T I O N - -                            //
    //   DispCHR() is used exclusively to send a single byte (ACKNOWLEDGE)   //
    //   back to the command initiator (Serial Command Monitor).  The byte   //
    //   is either chr_OK (Success) or the ErrCode (Failure) value which     //
    //   serves to identify the problem.                                     //
    //                                                                       //
    //   In order for commands that return data to respond in the correct    //
    //   sequence (first ACKNOWLEDGE then packetized data), DispCHR must     //
    //   remain first in this list of routines.                              //
    ///////////////////////////////////////////////////////////////////////////
        if (DispSvc & CHRreqMsk)        DispCHR();
        if (DispSvc & EreqMsk)          DispE();
        if (DispSvc & VreqMsk)          DispV();
        if (DispSvc & ADCreqMsk)        DispADC();
        if (DispSvc & RCreqMsk)         DispRC();
        if (DispSvc & HMSreqMsk)        DispHMS();
        if (DispSvc & MEMBYTEreqMsk)    DispMEM();
        if (DispSvc & IOBYTEreqMsk)     DispIO();

        if (DispSvc & STATreqMsk)       DispSTAT();
        if (DispSvc & PIDreqMsk)        DispPID();
        if (DispSvc & RAMBLKreqMsk)     DispRAMBLOCK();
        if (DispSvc & EXTEEBLKreqMsk)   DispExtEEBLOCK();
        if (DispSvc & INTEEBLKreqMsk)   DispIntEEBLOCK();
        if (DispSvc & RESETreqMsk)      ResetPIC();
    } // If anything to do ...
}
//-----------------------------------------------------------------------------
void DispCHR(void) 
{ // Conditionally Transmit xCHR during serial {API, I2C2} cmd processing.
    DispSvc &= ~CHRreqMsk;      // Clear request flag

    // If API, always transmit xCHR
    if(CmdSource == API_SrcMsk) printf("%c",xCHR);

    // If I2C2, transmit xCHR only if (Error) OR (NoError and no data).
    else if(CmdSource == I2C2_SrcMsk)
    {
        if( (xCHR != chr_OK) || (ReturnN == 0) )
        {
            Pkt[0]=xCHR;            // In case AutoStart needed.
            WriteSSP2BUF(Pkt[0]);   // Start Transmission
        }
    }
}
//-----------------------------------------------------------------------------
void DispE(void)
{
    short long  E1, E2, E;
    ////////////////////////////////////////////////////////////////////
    // Remark: The 24-bit, 2's complement encoder counter is updated  //
    // within ISR's.  The ISR's are briefly disabled here to avoid    //
    // potential glitch in captured values of encoder counters.       //
    ////////////////////////////////////////////////////////////////////
    DispSvc &= ~EreqMsk;        // Clear request flag

    INTCONbits.GIEH = 0;    // Disable high priority interrupts
    E1=encode1;
    E2=encode2;
    INTCONbits.GIEH = 1;    // Enable high priority interrupts
    
    //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode
        if(DataReq==0xFF)
        { // Send Both Motor Positions; Mtr1 first.
            PktLen=12;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='E';
            Pkt[3]=0x06;
            Pkt[4]=(E1 & 0xFF);
            Pkt[5]=((E1>>8) & 0xFF);
            Pkt[6]=((E1>>16) & 0xFF);
            Pkt[7]=(E2 & 0xFF);
            Pkt[8]=((E2>>8) & 0xFF);
            Pkt[9]=((E2>>16) & 0xFF);
            Pkt[11]=chr_ETX;
            SendApiPkt();
        }
        else
        { // Single Motor request
            if(DataReq==0x01) E=E1; else E=E2;
            PktLen=9;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='E';
            Pkt[3]=0x03;
            Pkt[4]=(E & 0xFF);
            Pkt[5]=((E>>8) & 0xFF);
            Pkt[6]=((E>>16) & 0xFF);
            Pkt[8]=chr_ETX;
            SendApiPkt();
        }
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        if((DataReq==0x01) || (DataReq==0xFF))
        {
            if(MTR1_MODE3 & PosDecMsk)  printf("E1: %08Hd\r\n", E1);
            else                        printf("E1: %06HX\r\n", E1);
        }
        if((DataReq==0x02) || (DataReq==0xFF))
        {
            if(MTR2_MODE3 & PosDecMsk)  printf("E2: %08Hd\r\n", E2);
            else                        printf("E2: %06HX\r\n", E2);
        }
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        if(DataReq==0xFF)
        { // Send Both Motor Positions; Mtr1 first.
            PktLen=9;
            Pkt[0]='E';
            Pkt[1]=0x06;
            Pkt[2]=(E1 & 0xFF);
            Pkt[3]=((E1>>8) & 0xFF);
            Pkt[4]=((E1>>16) & 0xFF);
            Pkt[5]=(E2 & 0xFF);
            Pkt[6]=((E2>>8) & 0xFF);
            Pkt[7]=((E2>>16) & 0xFF);
            SendI2C2Pkt();              // Transmit Pkt[].
        }
        else
        { // Single Motor request
            if(DataReq==0x01) E=E1; else E=E2;
            PktLen=6;
            Pkt[0]='E';
            Pkt[1]=0x03;
            Pkt[2]=(E & 0xFF);
            Pkt[3]=((E>>8) & 0xFF);
            Pkt[4]=((E>>16) & 0xFF);
            SendI2C2Pkt();              // Transmit Pkt[].
        }
    } // If I2C2
}
//-----------------------------------------------------------------------------
void DispV(void)
{
    short long  V;
    long temp1,temp2,velRPM;

    DispSvc &= ~VreqMsk;        // Clear request flag

    if(CmdSource == API_SrcMsk)
    { // API Mode
        if(DataReq==0xFF)
        { // Send Both Motor Velocities; Mtr1 first.
            PktLen=12;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='V';
            Pkt[3]=0x06;
            Pkt[4]=(V1 & 0xFF);
            Pkt[5]=((V1>>8) & 0xFF);
            Pkt[6]=((V1>>16) & 0xFF);
            Pkt[7]=(V2 & 0xFF);
            Pkt[8]=((V2>>8) & 0xFF);
            Pkt[9]=((V2>>16) & 0xFF);
            Pkt[11]=chr_ETX;
            SendApiPkt();
        }
        else
        { // Single Motor request
            if(DataReq==0x01) V=V1; else V=V2;
            PktLen=9;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='V';
            Pkt[3]=0x03;
            Pkt[4]=(V & 0xFF);
            Pkt[5]=((V>>8) & 0xFF);
            Pkt[6]=((V>>16) & 0xFF);
            Pkt[8]=chr_ETX;
            SendApiPkt();
        }
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
//////////////////////////////////////////////////////////////////////////////
//  Terminal Emulator interface velocity is shown in two units:             //
//                     Ticks/VSP(HEX; 24 bits)                              //
//                     RPM(Decimal)                                         //
//                                                                          //
//  V(RPM) = V(Ticks/VSP)*[1/VSP](VSP/ms)* 60000(ms/MIN)*[1/TPR](rev/Ticks) //
//////////////////////////////////////////////////////////////////////////////
        if( (DataReq==0x01) || (DataReq==0xFF) )
        { // If Motor 1 or both
            temp1 = (long)V1 * 60000;
            temp2 = (long)TPR1 * VSP1;
            velRPM = temp1/temp2;
            if(MTR1_MODE3 & VelDecMask)
                 printf("V1: %08Hd (%5ld rpm)", V1, velRPM);
            else printf("V1: %06HX (%5ld rpm)", V1, velRPM);
            if(vel1)    //If we are in ROVIM_T2D_MODE, print additional info
                 printf(" (%5ld Km/10/h)\r\n", vel1);
            else printf("\r\n");
        }
        if( (DataReq==0x02) || (DataReq==0xFF) )
        { // If Motor 2 or both
            temp1 = (long)V2 * 60000;
            temp2 = (long)TPR2 * VSP2;
            velRPM = temp1/temp2;
            if(MTR2_MODE3 & VelDecMask)
                 printf("V2: %08Hd (%5ld rpm)\r\n", V2,velRPM);
            else printf("V2: %06HX (%5ld rpm)\r\n", V2,velRPM);
            
        }
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        if(DataReq==0xFF)
        { // Send Both Motor Velocities; Mtr1 first.
            PktLen=9;
            Pkt[0]='V';
            Pkt[1]=0x06;
            Pkt[2]=(V1 & 0xFF);
            Pkt[3]=((V1>>8) & 0xFF);
            Pkt[4]=((V1>>16) & 0xFF);
            Pkt[5]=(V2 & 0xFF);
            Pkt[6]=((V2>>8) & 0xFF);
            Pkt[7]=((V2>>16) & 0xFF);
            SendI2C2Pkt();              // Transmit Pkt[].

        }
        else
        { // Single Motor request
            if(DataReq==0x01) V=V1; else V=V2;
            PktLen=6;
            Pkt[0]='V';
            Pkt[1]=0x03;
            Pkt[2]=(V & 0xFF);
            Pkt[3]=((V>>8) & 0xFF);
            Pkt[4]=((V>>16) & 0xFF);
            SendI2C2Pkt();              // Transmit Pkt[].
        }
    } // If I2C2
}
//-----------------------------------------------------------------------------
void DispADC(void)  // Get ADC (Ch:0 .. Ch:6)
{
    BYTE i;
    WORD AdcVal;                // ADC voltage in mV.

    DispSvc &= ~ADCreqMsk;      // Clear request flag

    if(CmdSource == API_SrcMsk)
    { // API Mode.  Transmit raw readings
        if(DataReq==0xFF)
        { // Send All Channnels
            PktLen=13;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='C';
            Pkt[3]=0x07;
            for (i=0; i<7;i++)  Pkt[i+4] = ADC0[i];
            Pkt[12]=chr_ETX;
            SendApiPkt();
        }
        else
        { // Single Channel
            PktLen=7;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='C';
            Pkt[3]=0x01;
            Pkt[4]=ADC0[DataReq];
            Pkt[6]=chr_ETX;
            SendApiPkt();
        }
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode.  Transmit readings in mV
        if(DataReq == 0xFF)
        { // Disp All Channels
            for (i=0; i<7; i++)
            {
                AdcVal=AdcConvert((WORD)ADC0[i]);
                printf("AN%1d: = %5d mV\r\n",i,AdcVal);
            }
            // Special case: vbatt (converted voltage from ADC0[6])
            printf("BAT: = %5d mV\r\n",AdcToVbatt(AdcVal));
        }
        else
        {
            AdcVal=AdcConvert((WORD)ADC0[DataReq]);
            printf("AN%1d: = %5d mV\r\n",DataReq,AdcVal);
        }
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        if(DataReq==0xFF)
        { // Disp All Channels.
            PktLen=10;
            Pkt[0]='C';
            Pkt[1]=0x07;
            for (i=0; i<7;i++)  Pkt[i+2] = ADC0[i];
            SendI2C2Pkt();              // Transmit Pkt[].
        }
        else
        { // Single Channel
            PktLen=4;
            Pkt[0]='C';
            Pkt[1]=0x01;
            Pkt[2]=ADC0[DataReq];
            SendI2C2Pkt();              // Transmit Pkt[].
        }
    } // If I2C2
}
//-----------------------------------------------------------------------------
void DispRC(void)
{
    BYTE i;
    WORD j;

    DispSvc &= ~RCreqMsk;       // Clear request flag

    if(CmdSource == API_SrcMsk)
    { // API Mode
        if(DataReq==0xFF)
        { // Send All Channnels
            PktLen=12;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='N';
            Pkt[3]=0x06;
            for (i=0; i<3;i++)
            {
                j=RC[i];
                Pkt[2*i+4] = j & 0xFF;
                Pkt[2*i+5] = j >> 8;
            }
            Pkt[11]=chr_ETX;
            SendApiPkt();
        }
        else
        { // Single Channel
            PktLen=8;
            j=RC[DataReq-1];
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='N';
            Pkt[3]=0x02;
            Pkt[4] = j & 0xFF;
            Pkt[5] = j >> 8;
            Pkt[7]=chr_ETX;
            SendApiPkt();
        }
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        if(DataReq == 0xFF)
        { // All Channels
            for(i=0; i<=2; i++)
                printf("Ch%1u: %5u uS (%03X)\r\n", i+1, RC[i],RC[i]);
        }
        else
        { // Single Channel
            i=DataReq-1;
            printf("Ch%1u: %5u uS (%03X)\r\n", DataReq,RC[i],RC[i]);
        }
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        if(DataReq==0xFF)
        { // All Channels.
            PktLen=9;
            Pkt[0]='N';
            Pkt[1]=0x06;
            for (i=0; i<3;i++)
            {
                j=RC[i];
                Pkt[2*i+2] = j & 0xFF;
                Pkt[2*i+3] = j >> 8;
            }
            SendI2C2Pkt();              // Transmit Pkt[].
        }
        else
        { // Single Channel
            PktLen=5;
            Pkt[0]='N';
            Pkt[1]=0x02;
            i=DataReq-1;
            Pkt[2]=(RC[i] & 0xFF);
            Pkt[3]=(RC[i] >> 8);
            SendI2C2Pkt();              // Transmit Pkt[].
        }
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispMEM(void)           // Single Memory Byte.
{
    DispSvc &= ~MEMBYTEreqMsk;  // Clear request flag
    if(CmdSource == API_SrcMsk)
    { // API Mode
        PktLen=7;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='R';
        Pkt[3]=0x01;
        Pkt[4]=xbyte;
        Pkt[6]=chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        printf("%02X\r\n", xbyte);
    }
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=4;
        Pkt[0]='R';
        Pkt[1]=0x01;
        Pkt[2]=xbyte;
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispIO(void)            // Single IOEXP register.
{
    DispSvc &= ~IOBYTEreqMsk;   // Clear request flag
    if(CmdSource == API_SrcMsk)
    { // API Mode
        PktLen=7;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='K';
        Pkt[3]=0x01;
        Pkt[4]=xbyte;
        Pkt[6]=chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        printf("%02X\r\n", xbyte);
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=4;
        Pkt[0]='K';
        Pkt[1]=0x01;
        Pkt[2]=xbyte;
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispSTAT(void)          // Status
{
    DispSvc &= ~STATreqMsk;     // Clear request flag

    if(CmdSource == API_SrcMsk)
    { // API Mode
        if(DataReq == 0xFF)
        { // Both motors
            PktLen=18;
            Pkt[0]=chr_STX;
            Pkt[1]=HOST_NID;
            Pkt[2]='U';
            Pkt[3]=0x0C;

            Pkt[4]=MTR1_MODE1;
            Pkt[5]=MTR1_MODE2;
            Pkt[6]=MTR1_MODE3;
            Pkt[7]=Power1;
            Pkt[8]=Mtr1_Flags1;
            Pkt[9]=Mtr1_Flags2;

            Pkt[10]=MTR2_MODE1;
            Pkt[11]=MTR2_MODE2;
            Pkt[12]=MTR2_MODE3;
            Pkt[13]=Power2;
            Pkt[14]=Mtr2_Flags1;
            Pkt[15]=Mtr2_Flags2;

            Pkt[17]=chr_ETX;
            SendApiPkt();
        }
        else
        { // Single motor
            if(DataReq == 0x01)
            { // Mtr1
                PktLen=12;
                Pkt[0]=chr_STX;
                Pkt[1]=HOST_NID;
                Pkt[2]='U';
                Pkt[3]=0x06;

                Pkt[4]=MTR1_MODE1;
                Pkt[5]=MTR1_MODE2;
                Pkt[6]=MTR1_MODE3;
                Pkt[7]=Power1;
                Pkt[8]=Mtr1_Flags1;
                Pkt[9]=Mtr1_Flags2;

                Pkt[11]=chr_ETX;
                SendApiPkt();
            }
            else
            { // Mtr2
                PktLen=12;
                Pkt[0]=chr_STX;
                Pkt[1]=HOST_NID;
                Pkt[2]='U';
                Pkt[3]=0x06;

                Pkt[4]=MTR2_MODE1;
                Pkt[5]=MTR2_MODE2;
                Pkt[6]=MTR2_MODE3;
                Pkt[7]=Power2;
                Pkt[8]=Mtr2_Flags1;
                Pkt[9]=Mtr2_Flags2;

                Pkt[11]=chr_ETX;
                SendApiPkt();
            }
        }
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        if( (DataReq==0x01) || (DataReq == 0xFF) )  MtrStatTE(0x01);
        if( (DataReq==0x02) || (DataReq == 0xFF) )  MtrStatTE(0x02);
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        if(DataReq == 0xFF)
        { // Both motors
            PktLen=15;
            Pkt[0]='U';
            Pkt[1]=12;
            Pkt[2]=MTR1_MODE1;
            Pkt[3]=MTR1_MODE2;
            Pkt[4]=MTR1_MODE3;
            Pkt[5]=Power1;
            Pkt[6]=Mtr1_Flags1;
            Pkt[7]=Mtr1_Flags2;

            Pkt[8]=MTR2_MODE1;
            Pkt[9]=MTR2_MODE2;
            Pkt[10]=MTR2_MODE3;
            Pkt[11]=Power2;
            Pkt[12]=Mtr2_Flags1;
            Pkt[13]=Mtr2_Flags2;

            SendI2C2Pkt();              // Transmit Pkt[].
        }
        else
        { // Single motor
            if(DataReq == 0x01)
            { // Mtr1
                PktLen=9;
                Pkt[0]='U';
                Pkt[1]=0x06;
                Pkt[2]=MTR1_MODE1;
                Pkt[3]=MTR1_MODE2;
                Pkt[4]=MTR1_MODE3;
                Pkt[5]=Power1;
                Pkt[6]=Mtr1_Flags1;
                Pkt[7]=Mtr1_Flags2;
                SendI2C2Pkt();              // Transmit Pkt[].
            }
            else
            { // Mtr2
                PktLen=9;
                Pkt[0]='U';
                Pkt[1]=0x06;
                Pkt[2]=MTR2_MODE1;
                Pkt[3]=MTR2_MODE2;
                Pkt[4]=MTR2_MODE3;
                Pkt[5]=Power2;
                Pkt[6]=Mtr2_Flags1;
                Pkt[7]=Mtr2_Flags2;
                SendI2C2Pkt();              // Transmit Pkt[].
            }
        } // If Single motor
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    MtrStatTE(BYTE mtr)
{ // Utility used to display motor status
    BYTE m1, m2, m3, s, flg2;

    if(mtr==1)
        { m1=MTR1_MODE1;m2=MTR1_MODE2;m3=MTR1_MODE3;s=Power1;flg2=Mtr1_Flags2; }
    else
        { m1=MTR2_MODE1;m2=MTR2_MODE2;m3=MTR2_MODE3;s=Power2;flg2=Mtr2_Flags2; }

    // MOTOR ID
    printf("MTR#  :%1X\r\n",mtr);

    // CONTROL MODE
    printf("MODE  :");
    if(m2 & PotcMsk) printf("POTC\r\n");
    else if (m2 & PotfMsk) printf("POTF\r\n");
    else if (m2 & RcNrmMsk) printf("RC\r\n");
    else if (m2 & RcMixMsk) printf("RCMIX\r\n");
    else if (m2 & RcServoMsk) printf("RCSVO\r\n");
    else if (m2 & PotServoMsk) printf("POTSVO\r\n");
    else if (m2 & PotMixMsk) printf("POTMIX\r\n");
    else printf("CMD\r\n");

    // PWM DUTY CYCLE & DIRECTION
    if (flg2 & DisableMsk) printf("PWM   :DISABLED\r\n");
    else if (s==0) printf("PWM   :OFF\r\n");
    else if(flg2 & _MtrD_mask) printf("PWM   :REV(%3u%%)\r\n",s);
    else printf("PWM   :FWD(%3u%%)\r\n",s);

    // TRIGGER MODE
    if(m1 & trigMsk) printf("TRIG  :ON\r\n");
    else printf("TRIG  :OFF\r\n");

    // PID ERR SUMMATION HOLDOFF
    if(m1 & sumhoMsk) printf("SUMHO :ON\r\n");
    else printf("SUMHO :OFF\r\n");

    // TARGET SPECIFICATION
    if(m1 & relMsk) printf("TGT   :REL\r\n");
    else printf("TGT   :ABS\r\n");

    // OVER CURRENT
    if (m3 & OcieMsk)
    {
        if (m3 & OcFastOffMsk) printf("OCMODE:FASTOFF\r\n");
        else printf("OCMODE:SLOWRUN\r\n");
    }
    else printf("OCMODE:NONE\r\n");

    printf("\r\n");
}
//-----------------------------------------------------------------------------
void DispPID()      // Motor PID runtime parameters
{
    WORD kp,ki,kd;
    BYTE vsp,vmin,vmax;

    //////////////////////////////////////////////////////////////////
    // Outputs motor specific PID parameters as well as a few that  //
    // apply to both motors.                                        //
    //////////////////////////////////////////////////////////////////
    DispSvc &= ~PIDreqMsk;      // Clear request flag
    if(DataReq==1)  {kp=KP1; ki=KI1; kd=KD1; vsp=VSP1; vmin=VMIN1; vmax=VMAX1;}
    else            {kp=KP2; ki=KI2; kd=KD2; vsp=VSP2; vmin=VMIN2; vmax=VMAX2;}
        //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode
        PktLen=19;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='P';
        Pkt[3]=0x0D;
        Pkt[4] = (kp & 0xFF);
        Pkt[5] = (kp >> 8);
        Pkt[6] = (ki & 0xFF);
        Pkt[7] = (ki >> 8);
        Pkt[8] = (kd & 0xFF);
        Pkt[9] = (kd >> 8);
        Pkt[10] = vsp;
        Pkt[11] = vmin;
        Pkt[12] = vmax;
        Pkt[13] = (MAXERR & 0xFF);
        Pkt[14] = (MAXERR >> 8);
        Pkt[15] = (MAXSUM & 0xFF);
        Pkt[16] = (MAXSUM >> 8);
        Pkt[18] = chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        printf("MTR# :%1X\r\n",DataReq);
        printf("KP   :%04X\r\n",kp);
        printf("KI   :%04X\r\n",ki);
        printf("KD   :%04X\r\n",kd);
        printf("VSP  :%02X\r\n",vsp);
        printf("VMIN :%02X\r\n",vmin);
        printf("VMAX :%02X\r\n",vmax);
        printf("MAXE :%04X\r\n",MAXERR);
        printf("MAXS :%04X\r\n",MAXSUM);
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=16;
        Pkt[0]='P';
        Pkt[1]=13;
        Pkt[2] = (kp & 0xFF);
        Pkt[3] = (kp >> 8);
        Pkt[4] = (ki & 0xFF);
        Pkt[5] = (ki >> 8);
        Pkt[6] = (kd & 0xFF);
        Pkt[7] = (kd >> 8);
        Pkt[8] = vsp;
        Pkt[9] = vmin;
        Pkt[10] = vmax;
        Pkt[11] = (MAXERR & 0xFF);
        Pkt[12] = (MAXERR >> 8);
        Pkt[13] = (MAXSUM & 0xFF);
        Pkt[14] = (MAXSUM >> 8);
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispHMS(void)           // HH:MM:SS (RTC Time)
{
    DispSvc &= ~HMSreqMsk;      // Clear request flag   
    //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode
        PktLen=9;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='D';
        Pkt[3]=0x03;
        Pkt[4] = HOURS;
        Pkt[5] = MINS;
        Pkt[6] = SECS;
        Pkt[8]=chr_ETX;
        SendApiPkt();
    }
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode
        printf("%02u:%02u:%02u\r\n", HOURS, MINS, SECS);
    }
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=6;
        Pkt[0]='D';
        Pkt[1]=0x03;
        Pkt[2] = HOURS;
        Pkt[3] = MINS;
        Pkt[4] = SECS;
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispExtEEBLOCK(void)    // Block of External EEPROM starting at 'pBYTE' 
{
    WORD    i,j;
    BYTE    hi,low;
    WORD    adrs;

    DispSvc &= ~EXTEEBLKreqMsk; // Clear request flag
    adrs=(WORD)pBYTE;
    //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode; Block Length variable (<= 0x80).
        PktLen=BlkLen+6;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='L';
        Pkt[3]=BlkLen;
        for (i=4; i<BlkLen+4; i++)
        {
            j=ReadExtEE_Byte(adrs); adrs++;
            Pkt[i]=j;
        }
        Pkt[BlkLen+5]=chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode; Block Length fixed (0x80);  Display as paired bytes.

        printf("External EEPROM\r\n");
        for (i=0; i<128; i+=8)
        {
            printf("%04X:",adrs);
            for (j=0; j<8; j+=2)
            {
                low=ReadExtEE_Byte(adrs); adrs++;
                hi=ReadExtEE_Byte(adrs);  adrs++;
                printf("%02X%02X ",low,hi);
            }
            printf("\r\n");
        }
        printf("\r\n");
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=BlkLen+3;
        Pkt[0]='L';
        Pkt[1]=BlkLen;
        for (i=2; i<BlkLen+2; i++)
        {
            j=ReadExtEE_Byte(adrs); adrs++;
            Pkt[i]=j;
        }
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispRAMBLOCK(void)      // 128 bytes of RAM starting at 'pBYTE' 
{
    WORD    i,j;
    BYTE    hi,low;

    DispSvc &= ~RAMBLKreqMsk;   // Clear request flag
    //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode (BlkLen: Block Length <= 0x80)
        PktLen=BlkLen+6;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='L';
        Pkt[3]=BlkLen;
        for (i=4; i<BlkLen+4; i++)
        {
            j=*pBYTE; pBYTE++;
            Pkt[i]=j;
        }
        Pkt[BlkLen+5]=chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode (Block Length assumed 0x80)
        printf("RAM\r\n");
        for (i=0; i<128; i+=8)
        {
            printf("%04X:",pBYTE);
            for (j=0; j<8; j+=2)
            {
                low=*pBYTE; pBYTE++; hi=*pBYTE; pBYTE++;
                printf("%02X%02X ",low,hi);
            }
            printf("\r\n");
        }
        printf("\r\n");
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=BlkLen+3;
        Pkt[0]='L';
        Pkt[1]=BlkLen;
        for (i=2; i<BlkLen+2; i++)
        {
            j=*pBYTE; pBYTE++;
            Pkt[i]=j;
        }
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    DispIntEEBLOCK(void)    // Block of Internal EEPROM starting at 'pBYTE' 
{
    WORD    i,j;
    BYTE    hi,low;
    WORD    adrs;

    DispSvc &= ~INTEEBLKreqMsk; // Clear request flag
    adrs=(WORD)pBYTE;
    //----------------------------
    if(CmdSource == API_SrcMsk)
    { // API Mode (BlkLen: Block Length <= 0x80)
        PktLen=BlkLen+6;
        Pkt[0]=chr_STX;
        Pkt[1]=HOST_NID;
        Pkt[2]='L';
        Pkt[3]=BlkLen;
        for (i=4; i<BlkLen+4; i++)
        {
            j=ReadIntEE_Byte(adrs); adrs++;
            Pkt[i]=j;
        }
        Pkt[BlkLen+5]=chr_ETX;
        SendApiPkt();
    } // If API
    //----------------------------
    else if(CmdSource == TE_SrcMsk)
    { // TE Mode (Block Length assumed 0x80)

        printf("Internal EEPROM\r\n");
        for (i=0; i<128; i+=8)
        {
            printf("%04X:",adrs);
            for (j=0; j<8; j+=2)
            {
                low=ReadIntEE_Byte(adrs); adrs++;
                hi=ReadIntEE_Byte(adrs);  adrs++;
                printf("%02X%02X ",low,hi);
            }
            printf("\r\n");
        }
        printf("\r\n");
    } // If TE
    //----------------------------
    else if(CmdSource == I2C2_SrcMsk)
    { // I2C2 Mode
        PktLen=BlkLen+3;
        Pkt[0]='L';
        Pkt[1]=BlkLen;
        for (i=2; i<BlkLen+2; i++)
        {
            j=ReadIntEE_Byte(adrs); adrs++;
            Pkt[i]=j;
        }
        SendI2C2Pkt();              // Transmit Pkt[].
    } // If I2C2
}
//-----------------------------------------------------------------------------
void    Tune1(void) // Mtr1 PID Tuning Aid: CmdQ Verbose output 
{
    BYTE i,index;

    ///////////////////////////////////////////////////////////////////////
    // Conditionally display PID Err for Mtr1.                           //
    //                                                                   //
    //  Primary usage: See "PID TUNING" Cmd.                             //
    //     Does nothing unless all conditions are met:                   //
    //       1) npid <= PidLimit.                                        //
    //       2) PID is active.                                           //
    //       3) Verbose mode active.                                     //
    //                                                                   //
    // For efficiency, API packets contain 8 samples of Err.             //
    ///////////////////////////////////////////////////////////////////////
    if( (Mtr1_Flags1 & pida_Msk) &&
        (npid1 < Pid1Limit) &&
        (MTR1_MODE3 & VerboseMsk))
    { // If something to do, ..
        //----------------------------
        if(CmdSource == API_SrcMsk)
        { // API Mode: Xmit in packets that contain 8 samples
            //
            // index: ptr to 3-byte Err field within DATA portion of pkt
            index = 4 + 3*(BYTE)(npid1 & 0x07);

            npid1++;    
            Pkt[index]=(Err1 & 0xFF);               // Low Byte
            Pkt[index+1]=((Err1 >> 8) & 0xFF);      // Mid Byte
            Pkt[index+2]=(Err1 >> 16);              // Hi Byte

            // Special case: Test for partial last packet
            if( ( npid1 == Pid1Limit ) && ( index != 25 ) )
            {
                for ( i = index; i <= 25; i+=3 )
                 { // Zero fill unused portion of DATA field
                    Pkt[i]=0; Pkt[i+1]=0; Pkt[i+2]=0;
                 }
                index = 25;
            }

            if( index == 25 )
            { // Pkt full.  Time to send
                Pkt[0]=chr_STX;
                Pkt[1]=HOST_NID;
                Pkt[2]='Q';         
                Pkt[3]=24;          // 8 Err samples (3 bytes per sample)
                Pkt[29]=chr_ETX;
                PktLen=30;
                SendApiPkt();
            }
        } // If API
        //----------------------------
        else if(CmdSource == TE_SrcMsk)
        { // TE Mode: Xmit line to TE
            npid1++;
            if(npid1==1) printf("STEP RESPONSE:1\r\n");
            printf("%3d:%+6Hd\r\n", npid1,Err1);
        } // If TE
        //----------------------------
        else if(CmdSource == I2C2_SrcMsk)
        { // I2C2 Mode: Xmit in packets that contain 8 samples.
            //
            // index: ptr to 3-byte Err field within DATA portion of pkt
            index = 2 + 3*(BYTE)(npid1 & 0x07);

            npid1++;    
            Pkt[index]=(Err1 & 0xFF);               // Low Byte
            Pkt[index+1]=((Err1 >> 8) & 0xFF);      // Mid Byte
            Pkt[index+2]=(Err1 >> 16);              // Hi Byte

            // Special case: Test for partial last packet
            if( ( npid1 == Pid1Limit ) && ( index != 23 ) )
            {
                for ( i = index; i <= 23; i+=3 )
                { // Zero fill unused portion of DATA field
                    Pkt[i]=0; Pkt[i+1]=0; Pkt[i+2]=0;
                }
                index = 23;
            }

            if( index == 23 )
            { // Pkt full.  Time to send
                Pkt[0]='Q';
                Pkt[1]=24;          // 8 Err samples (3 bytes per sample)
                PktLen=27;
                SendI2C2Pkt();      // Transmit Pkt[].
            }
        } // If I2C2
        //----------------------------
    } // something to do

    // Reset PidLimit after last output to inhibit in other cmds.
    if( npid1 == Pid1Limit ) Pid1Limit=0;
}
//-----------------------------------------------------------------------------
void    Tune2(void) // Mtr2 PID Tuning Aid: CmdQ Verbose output 
{
    BYTE i,index;

    ///////////////////////////////////////////////////////////////////////
    // Conditionally display PID Err for Mtr2.                           //
    //                                                                   //
    //  Primary usage: See "PID TUNING" Cmd.                             //
    //     Does nothing unless all conditions are met:                   //
    //       1) npid <= PidLimit.                                        //
    //       2) PID is active.                                           //
    //       3) Verbose mode active.                                     //
    //                                                                   //
    // For efficiency, API packets contain 8 samples of Err.             //
    ///////////////////////////////////////////////////////////////////////
    if( (Mtr2_Flags1 & pida_Msk) &&
        (npid2 < Pid2Limit) &&
        (MTR2_MODE3 & VerboseMsk))
    { // If something to do, ..
        //----------------------------
        if(CmdSource == API_SrcMsk)
        { // API Mode: Xmit in packets that contain 8 samples
            //
            // index: ptr to 3-byte Err field within DATA portion of pkt
            index = 4 + 3*(BYTE)(npid2 & 0x07);

            npid2++;    
            Pkt[index]=(Err2 & 0xFF);               // Low Byte
            Pkt[index+1]=((Err2 >> 8) & 0xFF);      // Mid Byte
            Pkt[index+2]=(Err2 >> 16);              // Hi Byte

            // Special case: Test for partial last packet
            if( ( npid2 == Pid2Limit ) && ( index != 25 ) )
            {
                for ( i = index; i <= 25; i+=3 )
                 { // Zero fill unused portion of DATA field
                    Pkt[i]=0; Pkt[i+1]=0; Pkt[i+2]=0;
                 }
                index = 25;
            }

            if( index == 25 )
            { // Pkt full.  Time to send
                Pkt[0]=chr_STX;
                Pkt[1]=HOST_NID;
                Pkt[2]='Q';         
                Pkt[3]=24;          // 8 Err samples (3 bytes per sample)
                Pkt[29]=chr_ETX;
                PktLen=30;
                SendApiPkt();
            }
        } // If API
        //----------------------------
        else if(CmdSource == TE_SrcMsk)
        { // TE Mode: Xmit line to TE
            npid2++;
            if(npid2==1) printf("STEP RESPONSE:2\r\n");
            printf("%3d:%+6Hd\r\n", npid2,Err2);
        } // If TE
        //----------------------------
        else if(CmdSource == I2C2_SrcMsk)
        { // I2C2 Mode: Xmit in packets that contain 8 samples.
            //
            // index: ptr to 3-byte Err field within DATA portion of pkt
            index = 2 + 3*(BYTE)(npid2 & 0x07);

            npid2++;    
            Pkt[index]=(Err2 & 0xFF);               // Low Byte
            Pkt[index+1]=((Err2 >> 8) & 0xFF);      // Mid Byte
            Pkt[index+2]=(Err2 >> 16);              // Hi Byte

            // Special case: Test for partial last packet
            if( ( npid2 == Pid2Limit ) && ( index != 23 ) )
            {
                for ( i = index; i <= 23; i+=3 )
                { // Zero fill unused portion of DATA field
                    Pkt[i]=0; Pkt[i+1]=0; Pkt[i+2]=0;
                }
                index = 23;
            }

            if( index == 23 )
            { // Pkt full.  Time to send
                Pkt[0]='Q';
                Pkt[1]=24;          // 8 Err samples (3 bytes per sample)
                PktLen=27;
                SendI2C2Pkt();      // Transmit Pkt[].
            }
        } // If I2C2
        //----------------------------
    } // something to do

    // Reset PidLimit after last output to inhibit in other cmds.
    if( npid2 == Pid2Limit ) Pid2Limit=0;
}
//-----------------------------------------------------------------------------






//-----------------------------------------------------------------------------
void    ResetPIC(void)          // Reset Dalf Board 
{
    DispSvc &= ~RESETreqMsk;        // Clear request flag
    DoReset();                  // Microcontroller reset
}
//-----------------------------------------------------------------------------
void    WinkLEDS(void)          // Briefly blink programmable LED's 
{
    _LED1_ON;
    _LED2_ON;
    _LED3_ON;
    SetDelay(&Delay1, 0, msec_200); // 200 msec delay
    while( !Timeout(&Delay1) );
    _LED1_OFF;
    _LED2_OFF;
    _LED3_OFF;
}
//-----------------------------------------------------------------------------
void    InitLED(void)           // Initialization for ServiceLED()
{
    ledshift = 0x80000000;
    ledcount = LED_PERIOD;
}
//-----------------------------------------------------------------------------
void    ServiceLED(void)        // Periodic LED service
{
///////////////////////////////////////////////////////////////////////////////
//      Four possible patterns on LED1 and LED2 indicating motor power:      //
//                                                                           //
//  OFF:        Power=0. No Power applied. ---------------- Off              //
//  FAST BLINK: Power>0, TGA active.  Power applied. ------- TGA Active      //
//  SLOW BLINK: Power>0, TGA inactive, Power applied, V>0. - Open loop move. //
//  ON:         Power>0, TGA inactive, Power applied, V=0. - Stalled.        //
//                                                                           //
//                 Three possible patterns on LED3                           //
//  OFF:          No Error.                                                  //
//  FAST BLINK:   Over Current (possibly transient condition)                //
//  SLOW BLINK:   R/C signal loss (possibly transient condition)             //
//  ON:           Low Batt (VBATT < VBWARN)                                  //
///////////////////////////////////////////////////////////////////////////////
    // Reset counter and do right shift (with wrap) on led scheduler.
    ledshift >>= 1; if(!ledshift) ledshift = 0x80000000;

    // Determine appropriate LED1 pattern
    if(!Power1)                             grn1pattern = LED_MTR_OFF;
    else if(Mtr1_Flags1 & tga_Msk)          grn1pattern = LED_MTR_TGA;
    else if(V1)                             grn1pattern = LED_MTR_OPENLP;
    else                                    grn1pattern = LED_MTR_STALL;

    // Determine appropriate LED2 pattern
    if(!Power2)                             grn2pattern = LED_MTR_OFF;
    else if(Mtr2_Flags1 & tga_Msk)          grn2pattern = LED_MTR_TGA;
    else if(V2)                             grn2pattern = LED_MTR_OPENLP;
    else                                    grn2pattern = LED_MTR_STALL;

    // Determine appropriate LED3 pattern
    if(LedErr==0)                           redpattern = LED_FULLOFF;
    else if(LedErr & (OC1msk + OC2msk))     redpattern = LED_OVERCURRENT;
    else if(LedErr & (SL1msk) + SL2msk)     redpattern = LED_SIGNAL_LOSS;
    else if(LedErr & VBATTmsk)              redpattern = LED_VBATT;


    // Adjust LED's as required.
    if(ledshift & grn1pattern) _LED1_ON; else _LED1_OFF;
    if(ledshift & grn2pattern) _LED2_ON; else _LED2_OFF;
    if(ledshift & redpattern)  _LED3_ON; else _LED3_OFF;
}
//-----------------------------------------------------------------------------
void    Motor1Current(void) // Voltage Window Transition - OverCurrent Sense
{
    //////////////////////////////////////////////////////////////////////
    //  This code is part of the response to a transition of the        //
    //  current sense voltage into or out of the the voltage window     //
    //  defined by the digital current limiting pots.                   //
    //                                                                  //
    //  The response depends on whether the transition is out of (over  //
    //  current) or into (current acceptable) the window as well as     //
    //  the setting of the "oc_fastoff" bit.                            //
    //                                                                  //
    //  If the "oc_fastoff" bit is set, and the transition is because   //
    //  of overcurrent, most of the response will have already been     //
    //  handled directly by the ISR.  Power to the motor will already   //
    //  have been removed and the motor control interface disabled.     //
    //  Recovery requires a board reset.                                //
    //                                                                  //
    //  Otherwise, if the "oc_fastoff" bit is clear, the interface will //
    // "recover" when the current again becomes acceptable.             //
    //                                                                  //
    //  The over current condition can be monitored by the state of     //
    //  Mtr1_Flags1.OverCurrent bit.                                    //
    //////////////////////////////////////////////////////////////////////
    if(ISR_Flags & OverC1Msk)       // Over current detected
    {
        // Flag over current
        Mtr1_Flags1 |= OverCurrent_Msk; // Flag over current condition
        LedErr |= OC1msk;               // Record also in LedErr

        //  If "oc_fastoff"=1, just fix some mtr control flags and exit.
        if(MTR1_MODE3 & OcFastOffMsk)
        {
            Mtr1FlagFix();
            return;
        }

        ///////////////////////////////////////////////////////////
        // Schedule a motor stop by issuing a "Stop Motor Cmd"   //
        // with a compatible CmdSource.  Leave operating mode    //
        // unchanged.                                            //
        ///////////////////////////////////////////////////////////
        if (MTR1_MODE2 & PotMsk) CmdSource = POT_SrcMsk;
        else if (MTR1_MODE2 & RcMsk) CmdSource = RC_SrcMsk;
        else CmdSource = TE_SrcMsk;
        SoftStop(0x01); 
    }
    else                            // Current now ok
    {
        Mtr1_Flags1 &= ~OverCurrent_Msk; // Reset over current condition
        LedErr &= ~OC1msk;               // Reset also in LedErr
    }
}
//-----------------------------------------------------------------------------
void    Motor2Current(void) // Voltage Window Transition - OverCurrent Sense
{
    //////////////////////////////////////////////////////////////////////
    //  This code is part of the response to a transition of the        //
    //  current sense voltage into or out of the the voltage window     //
    //  defined by the digital current limiting pots.                   //
    //                                                                  //
    //  The response depends on whether the transition is out of (over  //
    //  current) or into (current acceptable) the window as well as     //
    //  the setting of the "oc_fastoff" bit.                            //
    //                                                                  //
    //  If the "oc_fastoff" bit is set, and the transition is because   //
    //  of overcurrent, most of the response will have already been     //
    //  handled directly by the ISR.  Power to the motor will already   //
    //  have been removed and the motor control interface disabled.     //
    //  Recovery requires a board reset.                                //
    //                                                                  //
    //  Otherwise, if the "oc_fastoff" bit is clear, the interface will //
    // "recover" when the current again becomes acceptable.             //
    //                                                                  //
    //  The over current condition can be monitored by the state of     //
    //  Mtr2_Flags1.OverCurrent bit.                                    //
    //////////////////////////////////////////////////////////////////////
    if(ISR_Flags & OverC2Msk)       // Over current detected
    {
        // Flag over current
        Mtr2_Flags1 |= OverCurrent_Msk;     // Flag over current condition
        LedErr |= OC2msk;                   // Record also in LedErr

        //  If "oc_fastoff"=1, just fix some mtr control flags and exit.
        if(MTR2_MODE3 & OcFastOffMsk)
        {
            Mtr2FlagFix();
            return;
        }

        ///////////////////////////////////////////////////////////
        // Schedule a motor stop by issuing a "Stop Motor Cmd"   //
        // with a compatible CmdSource.  Leave operating mode    //
        // unchanged.                                            //
        ///////////////////////////////////////////////////////////
        if (MTR2_MODE2 & PotMsk) CmdSource = POT_SrcMsk;
        else if (MTR2_MODE2 & RcMsk) CmdSource = RC_SrcMsk;
        else CmdSource = TE_SrcMsk;
        SoftStop(0x02); 
    }
    else                            // Current now ok
    {
        Mtr2_Flags1 &= ~OverCurrent_Msk;    // Reset over current condition
        LedErr &= ~OC2msk;                  // Reset also in LedErr
    }
}
//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
void    GetApiPktCkSum(void)    // Compute and store Pkt CkSum Byte
{ // Entry: Pkt[], PktLen.  Exit: Pkt[PktLen-2]=CkSum
    BYTE cksum=0, i;
    Pkt[PktLen-2]=0;            // Pre-Fill CkSum field with zero.
    for (i=0; i<PktLen; i++) cksum+=Pkt[i];
    Pkt[PktLen-2] = ~cksum + 1;
}
//-----------------------------------------------------------------------------
void    GetI2C2PktCkSum(void)   // Compute and store Pkt CkSum Byte
{ // Entry: Pkt[], PktLen.  Exit: Pkt[PktLen-1]=CkSum
    BYTE cksum=0, i;
    Pkt[PktLen-1]=0;                        // Pre-Fill CkSum field with zero.
    for (i=0; i<PktLen; i++) cksum+=Pkt[i];
    Pkt[PktLen-1] = ~cksum + 1;
}
//-----------------------------------------------------------------------------
void    SendApiPkt(void)        // Transmit API Pkt to Host.
{ // Entry: Pkt[], PktLen.
    BYTE i;
    GetApiPktCkSum();
    for (i=0; i<PktLen; i++) printf("%c",Pkt[i]);
}
//-----------------------------------------------------------------------------
void    SendI2C2Pkt(void)       // Transmit I2C2 Pkt[] to Host
{
    GetI2C2PktCkSum();              // Pkt[PktLen-1] <-- CKSUM.
    WriteSSP2BUF(Pkt[0]);           // Start Transmission
}
//-----------------------------------------------------------------------------
void    WriteSSP2BUF(BYTE chr)  // Load I2C2 Transmit Buffer
{   //----------------------------------------------------------------
    // If the SSP2 ISR has requested data, we start transmit of the
    // first char (Pkt[0]) here.  Subsequent bytes to transmit, are loaded by 
    // the ISR starting at Pkt[1].  
    //
    // Else, we flag that transmission hasn't yet started so the 
    // ISR can auto-start transmission later from Pkt[0]. 
    //----------------------------------------------------------------
    if(I2C2_State==3)
    { // Data requested by SSP2_ISR.  Start xmit now.
        SSP2BUF=chr;                        // Load Xmit Buffer
        SSP2CON1bits.CKP=1;                 // Release SCLK (Start Xmit)
    }
    else
    { // Data has not yet been requested by SSP2_ISR.
        I2C2_PktStatus |= i2c2_AutoMsk;     // Flag data Pkt[] ready.  AutoStart
    }
}
//-----------------------------------------------------------------------------






//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++          Interrupt Service Actions (called by Main Loop)              +++
//+++                                                                       +++
//+++   These services are the place for interrupt driven actions that      +++
//+++   might introduce too much interrupt latency if included within the   +++
//+++   ISR handlers.                                                       +++
//+++                                                                       +++
//+++      ===========================================================      +++
//+++      The main loop calls SvcDispatch() with an index: 0<=i<=15        +++
//+++      which is used to access routines Svc0, Svc1, ... SvcF.  The      +++
//+++      ISR will have set the appropriate bit [0..F] in SERVICE_REQ.     +++
//+++      ===========================================================      +++
//+++                                                                       +++
//+++   Interrupts will be enabled during execution of these routines.      +++
//+++                                                                       +++
//+++   Some services are driven by timer interrupts which make regular     +++
//+++   periodic requests for service (eg; TMR0 and TMR1 services).         +++
//+++   Note that processing of the Svc's can be delayed by other tasks.    +++
//+++                                                                       +++
//+++                                                                       +++
//+++    SVC   INT       DESCRIPTION                           FREQ         +++
//+++   ----   -------   ----------------------------------    --------     +++
//+++   Svc0   TMR0      Mtr Command Processing                1 msec       +++
//+++   Svc1   INT0      Mtr2 Encoder (AB2INT)                 varies       +++
//+++   Svc2   INT1      Mtr1 Encoder (AB1INT)                 varies       +++
//+++   Svc3   TMR1      RTC                                   1 sec        +++
//+++   Svc4   TMR2      ADC State Machine (end of sequence)   35 msec (*)  +++
//+++   Svc5   INT2      Mtr2 Current Sense (I2INT)            varies       +++
//+++   Svc6   INT3      Mtr1 Current Sense (I1INT)            varies       +++
//+++   Svc7   ////      ///////                               //////       +++
//+++   Svc8   TX1       USART1 Transmit                       varies       +++
//+++   Svc9   RC1       USART1 Receive                        varies       +++
//+++   SvcA   CCP1      EX0 - R/C Channel 1                   varies       +++
//+++   SvcB   CCP2      EX1 - R/C Channel 2                   varies       +++
//+++   SvcC   CCP3      EX3 - R/C Channel 3                   varies       +++
//+++   SvcD   SSP2      I2C2 - Secondary I2C Bus (SLAVE)      varies       +++
//+++   SvcE   ////      ///////                               //////       +++
//+++   SvcF   ////      ///////                               //////       +++
//+++                                                                       +++
//+++     (*) - Timing affected by parameters in Parameter Block            +++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//-----------------------------------------------------------------------------
// Svc0 - TMR0 Service Request.                           Request every 1 msec.
//-----------------------------------------------------------------------------
void Svc0(void)         // TMR0: Heartbeat (1msec)
{
    ///////////////////////////////////////////////////////////////////////////
    //  Svc0 performs two types of tasks:                                    //
    //                                                                       //
    //  1) EXECUTE NEW COMMANDS                                              //
    //       Commands are received from serial (Svc9), I2C2, RC, and POTs.   //
    //       Serial and I2C2 cmds are executed shortly after receipt.  RC    //
    //       and POT cmds execute according to schedule maintained by        //
    //       the TIMESVC flags.  If enabled, serial cmd interface is checked //
    //       for timeout.                                                    //
    //                                                                       //
    //  2) MOTOR SERVICING                                                   //
    //       Open loop commands: Service consists of ramp to desired speed.  //
    //       Closed loop commands: Service consists of periodic calls to     //
    //       the Trajectory Generator and PID functions to directly alter    //
    //       the motor PWM duty cycle.                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Conditionally service on-board LED's
    ledcount--; if(!ledcount)
    { 
        ledcount=LED_PERIOD; 
        #ifdef DALF_ROVIM_T2D
        ROVIM_T2D_ServiceLED();
        #else //DALF_ROVIM_T2D
        ServiceLED();
        #endif //DALF_ROVIM_T2D
    }
    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_sysmonitorcount--;
    //For debug purposes, this task can be triggered by the user
    if((!ROVIM_T2D_sysmonitorcount) && (!ManualSysMonitoring))
    {
        ROVIM_T2D_sysmonitorcount=ROVIM_T2D_SYSTEM_MONITOR_PERIOD;
        ROVIM_T2D_MonitorSystem();
    }
    ROVIM_T2D_pwmrefreshcount--;
    if(!ROVIM_T2D_pwmrefreshcount)
    {
        ROVIM_T2D_pwmrefreshcount=ROVIM_T2D_PWM_REFRESH_PERIOD;
        ROVIM_T2D_ServicePWM();
    }
    #endif //DALF_ROVIM_T2D
    
#ifdef WATCHDOG_ENABLED
    watchdogcount--; if(!watchdogcount) {watchdogcount=WATCHDOG_PERIOD; KickWatchdog(); }
#endif
#ifdef DALF_TEST_ENABLED
    //TEST_Svc0();
#endif

    // Capture timed service requests.
    TIMESVC = TIMESVC_REQ;              // TIMESVC = Working copy
    TIMESVC_REQ ^= TIMESVC;             // Clear active Svc Req Flags

    //////////////////////////////////////////////////////////////////////
    //      P E N D I N G    C M D    S E R V I C E S                   //
    //                                                                  //
    //  If motor now stopped (Power1=0 or Power2=0), and pending        //
    //  cmd is awaiting motor stopped, do it now.                       //
    //////////////////////////////////////////////////////////////////////
    PendingCmd1();
    PendingCmd2();

    /////////////////////////////////////////////////
    //     R A M P I N G      S E R V I C E S      //
    /////////////////////////////////////////////////
    RampMotor1();           // Conditionally ramp mtr1
    RampMotor2();           // Conditionally ramp mtr2


    ////////////////////////////////////////
    //    V S P 1    S E R V I C E S      //
    ////////////////////////////////////////
    if(TIMESVC & Vsp1Msk)   // If time to sample Motor Velocity
    {
        #ifdef DALF_ROVIM_T2D
        ROVIM_T2D_UpdateVelocity1();  // Update Motor Velocity and Acceleration
        #else //DALF_ROVIM_T2D
        UpdateVelocity1();  // Update Motor Velocity
        #endif //DALF_ROVIM_T2D
        Trajectory1();      // Conditional mtr1 trajectory
        PID1();             // Conditional mtr1 PID Control Update
        Tune1();            // Conditional mtr1 PID tuning output
    }

    ////////////////////////////////////////
    //    V S P 2    S E R V I C E S      //
    ////////////////////////////////////////
    if(TIMESVC & Vsp2Msk)   // If time to sample Motor Velocity
    {
        UpdateVelocity2();  // Update Motor Velocity
        Trajectory2();      // Conditional mtr2 trajectory
        PID2();             // Conditional mtr2 PID Control Update
        Tune2();            // Conditional mtr2 PID tuning output
    }


    //////////////////////////////////////////////////////////////////
    //       N E W   M O T O R   C M D   S E R V I C E S            //
    //                                                              //
    // 1) SerialCmdDispatch() starts any RS232 or I2C2 cmds.        //
    //    ReturnData() returns data and/or status for serial cmds.  //
    //                                                              //
    // 2) CheckCmdTimeout() conditionally monitors Serial Cmd       //
    //    Interface and shuts down motors after a timeout.          //
    //                                                              //
    // 2) PotOrRc issues motor cmds depending on mode and TIMESVC.  //
    //    If Mode is RCMIX or POTMIX, PotOrRc(0x01) handles Mtr2    //
    //    also.  If OverCurrent -NOW-, PotOrRc() is skipped.        //
    //////////////////////////////////////////////////////////////////
    SerialCmdDispatch();    // Handle any new cmd from serial interfaces
    ReturnData();           // Returns status or data from commands
    if(TIMESVC & CmdspMsk)  CheckCmdTimeout();  // Monitor serial cmd interface
    if(!(Mtr1_Flags1 & OverCurrent_Msk))    PotOrRc(0x01);  // Mtr1 POT or RC
    if(!(Mtr2_Flags1 & OverCurrent_Msk))    PotOrRc(0x02);  // Mtr2 POT or RC
}
//-----------------------------------------------------------------------------
void Svc1(void)         // INT0: AB2INT (Not currently used)
{
}
//-----------------------------------------------------------------------------
void Svc2(void)         // INT1: AB1INT (Not currently used)
{
}
//-----------------------------------------------------------------------------
void Svc3(void)         // TMR1: RTC (every 1000 msec)
{
    ////////////////////////////////////////////////////////////////////////
    // Note: Good place for services that don't require timely execution  //
    ////////////////////////////////////////////////////////////////////////
    WORD x,vbatt;
                    // Computed VBATT voltage in mV.

    ////////////////////////////////////////////////////////////////////////
    //                       Monitor VBATT                                //
    //                                                                    //
    //  Hysteresis avoids LED blinking near the warning threshold.        //
    ////////////////////////////////////////////////////////////////////////
    x=AdcConvert(ADC0[6]);                  // V6 in millivolts
    vbatt=AdcToVbatt(x);                    // VBATT in millivolts
    if (vbatt < VBWARN)
    {
        LedErr |= VBATTmsk;                 // Signal Low VBATT
    }
    else if ( vbatt > (VBWARN + LED_HYSTERESIS) )
    {
        LedErr &= ~VBATTmsk;                // Signal VBATT ok
    }
}
//-----------------------------------------------------------------------------
void Svc4(void)         // TMR2: ADC State Machine 
{
}
//-----------------------------------------------------------------------------
void Svc5(void)         // INT2: I2INT - Mtr2 Current Sense
{
    /////////////////////////////////////////////////////////////////////
    //  The corresponding interrupt has occurred because the motor     //
    //  current sense voltage has transitioned into or out of voltage  //
    //  threshold window specified by the on board digital pots.       //
    /////////////////////////////////////////////////////////////////////
    Motor2Current();    // Mtr2 Over Current Transition
}
//-----------------------------------------------------------------------------
void Svc6(void)         // INT3: I1INT - Mtr1 Current Sense
{
    /////////////////////////////////////////////////////////////////////
    //  The corresponding interrupt has occurred because the motor     //
    //  current sense voltage has transitioned into or out of voltage  //
    //  threshold window specified by the on board digital pots.       //
    /////////////////////////////////////////////////////////////////////
    Motor1Current();    // Mtr1 Over Current Transition
}
//-----------------------------------------------------------------------------
void Svc7(void)         // UNUSED 
{
}
//-----------------------------------------------------------------------------
void Svc8(void)         // TX1 (UNUSED)
{
}
//-----------------------------------------------------------------------------
void Svc9(void)         // RC1 Service
{
    BYTE err;
    // The RX1 ISR has indicated that a unit of data (either a null terminated
    // (ASCIZ) command string in the case of a Terminal Emulator App, or a 
    // packetized command string in the case of a Smart App), is in the
    // circular buffer Rx1_Buff.  The code here parses the command string and
    // then if no errors causes it to be serviced later.
    //
    // TERMINAL EMULATOR CASE (See Owner's Manual for documentation)
    //    Eg:  The 11 char ASCIZ string  "L 01 01 3F"0x00 is parsed as:
    //
    //        CMD: "L"
    //        ARG[]: 0x01,0x01,0x3F
    //        ARGN: 3
    //
    // SMART APPLICATION CASE (See API Specification for documentation)
    //    Eg:  The xx char HEX string "02 01 4C 04 01 01 3F 20 49 03" is parsed as:
    //
    //        STX (0x02)
    //        NID (0x01)
    //          CMD: "L" (0x4C)
    //          N: ARGN=4 (0x04)
    //          ARG[]: 0x01,0x01,0x3F,0x20
    //          CKSUM: (0x49)
    //        ETX (0x03)
    //
    //        Note: API version of CMD_L has additional 4th argument (Length)    
    //----------------------------
    if ( SCFG == APIcfg )
    {// Smart Application Uses API
        err = API_CmdParse();
        if (!err) { serialcmd=TRUE; CmdSource=API_SrcMsk; }
    } // If API
    //----------------------------
    else if ( SCFG == TEcfg )
    {// Terminal Emulator Application uses ASCII string data
        err = TE_CmdParseExt();
        if (!err) { serialcmd=TRUE; CmdSource=TE_SrcMsk; }
        else if (err==eParseErr)     printf("Parse Err\r\nEnter 'H' for help\r\n");
    } // If TE
    //----------------------------
}
//-----------------------------------------------------------------------------
void SvcA(void)         // CCP1 (EX0 - R/C Channel 1)
{
    WORD pw;

    ////////////////////////////////////////////////////////////////////
    // Remark: The 16-bit PulseWidth is captured within the CCP ISR.  //
    // The ISR is briefly disabled here to avoid potential glitch in  //
    // obtaining the current value.                                   //
    ////////////////////////////////////////////////////////////////////
    PIE1bits.CCP1IE = 0;        // Disable CCP1: Pulse capture
    pw = PulseWidth1;           // pw <= 0x7FFF
    PIE1bits.CCP1IE = 1;        // Enable CCP1:  Pulse capture

    NewPulse |= NewRC1msk;      // Flag new pulse arrival
    RC[0] = PulseConvert(pw);
}
//-----------------------------------------------------------------------------
void SvcB(void)         // CCP2 (EX1 - R/C Channel 2)
{
    WORD pw;

    ////////////////////////////////////////////////////////////////////
    // Remark: The 16-bit PulseWidth is captured within the CCP ISR.  //
    // The ISR is briefly disabled here to avoid potential glitch in  //
    // obtaining the current value.                                   //
    ////////////////////////////////////////////////////////////////////
    PIE2bits.CCP2IE = 0;        // Disable CCP2: Pulse capture
    pw = PulseWidth2;           // pw <= 0x7FFF
    PIE2bits.CCP2IE = 1;        // Enable CCP2:  Pulse capture

    NewPulse |= NewRC2msk;      // Flag new pulse arrival
    RC[1] = PulseConvert(pw);
}
//-----------------------------------------------------------------------------
void SvcC(void)         // CCP3 (EX3 - R/C Channel 3)
{
    WORD pw;

    ////////////////////////////////////////////////////////////////////
    // Remark: The 16-bit PulseWidth is captured within the CCP ISR.  //
    // The ISR is briefly disabled here to avoid potential glitch in  //
    // obtaining the current value.                                   //
    ////////////////////////////////////////////////////////////////////
    PIE3bits.CCP3IE = 0;        // Disable CCP3: Pulse capture
    pw = PulseWidth3;           // pw <= 0x7FFF
    PIE3bits.CCP3IE = 1;        // Enable CCP3:  Pulse capture

    NewPulse |= NewRC3msk;      // Flag new pulse arrival
    RC[2] = PulseConvert(pw);
}
//-----------------------------------------------------------------------------
void SvcD(void)         // SSP2 (I2C2 SLAVE Serial Interface)
{
    BYTE err;

    // Application using I2C2 to issue commands
    err = I2C2_CmdParse();
    if (!err) { serialcmd=TRUE; CmdSource = I2C2_SrcMsk; }
}
//-----------------------------------------------------------------------------
void SvcE(void)         // UNUSED
{
}
//-----------------------------------------------------------------------------
void SvcF(void)         // UNUSED
{
}
//-----------------------------------------------------------------------------










//-----------------------------------------------------------------------------
void main (void)
{
    SystemInitExt();            // Basic System Initialization
//*************************************************************
//**  SystemInit() is required.  If you need to reconfigure  **
//**  resources, do it after SystemInit()                    **
//*************************************************************
    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_Init();       //ROVIM System Initialization

    /*The actions performed by this function do not need interrupts to work. However, print outputs 
    are done through interrupts. The function seems to work fine at this stage of execution, 
    so it will stay here*/
    ROVIM_T2D_Lockdown();  // Lock the brakes as soon as possible - safety first/
    #endif //DALF_ROVIM_T2D
    
#ifdef DALF_TEST_ENABLED
    TEST_TestInit();        //Testing Module Initialization
#endif

#ifdef LOG_ENABLED
    LOG_LogInit();          //Internal non-volatile event logger initialization
#endif

    InitLED();              // ServiceLED Initialization

    // Enable Interrupt System
    EnableInterrupts();

#ifdef WATCHDOG_ENABLED
    /*I don't think there's much problem in starting watchdog here instead of from power up,
    because until here the signals that are activated are goo (safetywise), not bad
    (such as acc, activate traction, etc.)*/
    STATUS_MSG("Starting watchdog.\r\n");
    DEBUG_MSG("Beware watchdog may actuate with debug traces enabled.\r\n");
    InitWatchdog();
#endif //WATCHDOG_ENABLED
    
    // Select custom putc()
    stdout = _H_USER;
    stderr = _H_USER;

    WinkLEDS();             // Wink LED's to indicate power on reset.

    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_Greeting();
    #endif //DALF_ROVIM_T2D

    #ifdef DALF_ROVIM_T2D
    ROVIM_T2D_Start();
    #endif //DALF_ROVIM_T2D
    
    //continuously monitor the changes we're doing, to avoid bigger troubles in the
    //future
#ifdef DALF_TEST_ENABLED
    //TEST_InDevelopmentTesting();
    //TEST_StartGPIODriverTest();
#endif
    
//  ***************************************************************************
//  *                         M A I N    L O O P                              *
//  *                                                                         *
//  *  MAIN LOOP - Process service requests from ISR's (int service routines) *
//  ***************************************************************************
    while(1)
    {
        SERVICE = SERVICE_REQ;              // SERVICE = Working copy
        SERVICE_REQ ^= SERVICE;             // Clear Svc Req Flags
        if( SERVICE )
        {
            if( SERVICE & 0x0001 ) Svc0();  // Mtr Cmd Processing
            if( SERVICE & 0x0008 ) Svc3();  // RTC
            if( SERVICE & 0x0020 ) Svc5();  // Mtr2 Current Sense
            if( SERVICE & 0x0040 ) Svc6();  // Mtr1 Current Sense
            if( SERVICE & 0x0200 ) Svc9();  // RC1 
            if( SERVICE & 0x0400 ) SvcA();  // EX0 R/C Channel 1
            if( SERVICE & 0x0800 ) SvcB();  // EX1 R/C Channel 2
            if( SERVICE & 0x1000 ) SvcC();  // EX3 R/C Channel 3
            if( SERVICE & 0x2000 ) SvcD();  // SSP2 (I2C2)

///////////////////////////////////////////////////////////////////////////////
//  Unused   - Bit set by ISR, but no required main loop service             //
//  Unmapped - Bit Doesn't get set by anything                               //
///////////////////////////////////////////////////////////////////////////////
        //  if( SERVICE & 0x0002 ) Svc1();  // Mtr2 Encoder    (UNUSED)
        //  if( SERVICE & 0x0004 ) Svc2();  // Mtr1 Encoder    (UNUSED)
        //  if( SERVICE & 0x0010 ) Svc4();  // TMR2: ADC State (UNUSED)
        //  if( SERVICE & 0x0080 ) Svc7();  // --------------- (UNMAPPED)
        //  if( SERVICE & 0x0100 ) Svc8();  // TX1             (UNUSED)
        //  if( SERVICE & 0x4000 ) SvcE();  // --------------- (UNMAPPED)
        //  if( SERVICE & 0x8000 ) SvcF();  // --------------- (UNMAPPED)
        }
        // If enabled, and nothing to do, enter low power (IDLE) mode.
        if( (SYSMODE & IdleMsk) && (!SERVICE_REQ) ) DoIdle();
    }   // while(1)
} // main()
//----------------------------------------------------------------------------
// High priority interrupt handler
// #pragma interrupt ISRHI
#pragma interrupt ISRHI nosave=section("MATH_DATA"),section(".tmpdata")
void ISRHI (void)
{
    // If INT2 Interrupt (I2INT: Mtr2 Current Sense)
    if ( (INTCON3bits.INT2IE == 1) && (INTCON3bits.INT2IF == 1) ) INT2_ISR();

    // If INT3 Interrupt (I1INT: Mtr1 Current Sense)
    if ( (INTCON3bits.INT3IE == 1) && (INTCON3bits.INT3IF == 1) ) INT3_ISR();

    // If TMR0 Interrupt (Heartbeat: 1msec)
    if ( (INTCONbits.TMR0IE == 1) && (INTCONbits.TMR0IF == 1) ) TMR0_ISR();

    // If TMR1 Interrupt (RTC: 1sec)
    if ( (PIE1bits.TMR1IE == 1) && (PIR1bits.TMR1IF == 1) ) TMR1_ISR();

    // If TMR2 Interrupt (ADC)
    if ( (PIE1bits.TMR2IE == 1) && (PIR1bits.TMR2IF == 1) ) TMR2_ISR();

    // If INT0 Interrupt (AB2INT: Mtr2 encoder)
    if ( (INTCONbits.INT0IE == 1) && (INTCONbits.INT0IF == 1) ) INT0_ISR();

    // If INT1 Interrupt (AB1INT: Mtr1 encoder)
    #ifdef DALF_ROVIM_T2D
    if ( (INTCON3bits.INT1IE == 1) && (INTCON3bits.INT1IF == 1) ) INT1_ISR_SINGLE_SOURCE();
    #else //DALF_ROVIM_T2D
    if ( (INTCON3bits.INT1IE == 1) && (INTCON3bits.INT1IF == 1) ) INT1_ISR();
    #endif //DALF_ROVIM_T2D

    // If TX1 Interrupt (Cmd Interface Xmit)
    if ( (PIE1bits.TX1IE == 1) && (PIR1bits.TX1IF == 1) ) TX1_ISR();

    // If RX1 Interrupt (Cmd Interface Rcv)
    if ( (PIE1bits.RC1IE == 1) && (PIR1bits.RC1IF == 1) ) RX1_ISR();

    // If CCP1 Interrupt (EX0 - R/C Channel 1 [Mtr1])
    if ( (PIE1bits.CCP1IE == 1) && (PIR1bits.CCP1IF == 1) ) CCP1_ISR();

    // If CCP2 Interrupt (EX1 - R/C Channel 2 [Mtr2])
    if ( (PIE2bits.CCP2IE == 1) && (PIR2bits.CCP2IF == 1) ) CCP2_ISR();

    // If CCP3 Interrupt (EX2 - R/C Channel 3 [Alt])
    if ( (PIE3bits.CCP3IE == 1) && (PIR3bits.CCP3IF == 1) ) CCP3_ISR();

    // If SSP2 Interrupt (SSP2 - Secondary I2C bus)
    if ( (PIE3bits.SSP2IE == 1) && (PIR3bits.SSP2IF == 1) ) SSP2_ISR();
}
//----------------------------------------------------------------------------
// Low priority interrupt handler
#pragma interruptlow ISRHI
void ISRLO (void)
{
}

#ifdef DALF_ROVIM_T2D
// Custom handler for INT1 Interrupt (AB1INT: Mtr1 encoder), for a single-source encoder
/* This ISR reads a single source encoder signal, and ignore quadrature signals and
direction calculations.*/
void INT1_ISR_SINGLE_SOURCE (void)
{
    volatile BYTE A=0;
    static BYTE prevA=0;
    
    INTCON3bits.INT1IF=0;
    /*Q: Should the counter be declared as volatile?
      A: I think it can work like this, because the variable may not be optimized in it's source
      object, and will therefore behave implicitly as volatile. But this is just an assumption.
      Anyhow, I've done some light testing, and it seems qualifying the variable as volatile doesn't
      change things.
      */
    /*change the polarity of the interrupt. Since we don't have a quadrature signal and we are
    getting a LOT of noisy interrupts (although the input signal doesn't appear to be noisy), 
    the readings get corrupted if we just increment the counter on each interrupt (way more 
    interrupts than encoder pulses). However, if we set the interrupt only on the rising edge, and
    compare the port value with the previous one, we only count the first interrupt, because after
    that the port value will allways equal the previous one. So we need to trigger the interrupt
    on both edges. To do that, we need to change the edge on each interrupt*/
    INTCON2bits.INTEDG1 = ~(INTCON2bits.INTEDG1);
    A=PORTE;
    A&=0x04; //We only want the RE2 bit, wich corresponds to the A1 input
    A=A>>2;
    if((A==1) && (prevA==0))
    {
        encode1++;
    }
    prevA=A;
}
#endif //DALF_ROVIM_T2D
//----------------------------------------------------------------------------
