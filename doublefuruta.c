
#include "F28x_Project.h"
#include "globals.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define buflen 50
#define DUTY_CYCLE_DEFAULT 0
#define SCI_SELECT 3

const int EPWM1_TIMER_TBPRD = 2000;
const float MAX_TORQUE = 0.75;
//
// Globals
//
Uint16 LoopCount;
double x[6] = {0.0};
State state;
int pwm_active = 1;
int motor_active = 0;
double torque;
//
// Function Prototypes
//
void InitEPwm1(void);
void InitEPwm11(void);
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);
void scib_echoback_init(void);
void scib_fifo_init(void);
void scib_xmit(int a);
void scib_msg(char *msg);

interrupt void xint1_isr(void);
interrupt void xint2_isr(void);
interrupt void xint3_isr(void);
//
// Main
//
void main(void)
{
    char recv[buflen];
    char *token;
    char delimiter[2] = ",";


//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.


//
   InitGpio();

//
// For this example, only init the pins for the SCI-A port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.

   GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);   // motor enable pin
   GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);

   GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);   // motor direction pin
   GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);

   GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);   // pwm enable pin
   GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);

   GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15); // scia pins
   GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
   GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);  // scib pins
   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);  // led pin
   GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);

   GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);  // led pin
   GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

   GPIO_WritePin(31, 1);    // turn off led for motor enable indication
   GPIO_WritePin(34, 0);    // turn on led for pwm_active indication
   GPIO_WritePin(2, 0);     // disable motor

   EALLOW;
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;
   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;

   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 5; //PWM11A
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0; //GPIO

   GpioCtrlRegs.GPADIR.bit.GPIO20 = GPIO_OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO21 = GPIO_OUTPUT;
   EDIS;

   GPIO_WritePin(21,0);

   CpuSysRegs.PCLKCR2.bit.EPWM1=1;
   InitEPwm1Gpio();

   CpuSysRegs.PCLKCR2.bit.EPWM11=1;

//
// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
   InitPieVectTable();

   InitEPwm1();
   InitEPwm11();

   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.XINT1_INT = &xint1_isr;
   PieVectTable.XINT2_INT = &xint2_isr;
   PieVectTable.XINT3_INT = &xint3_isr;
   EDIS;

   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
   PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5
   IER |= M_INT1;                              // Enable CPU INT1
   EINT;                                       // Enable Global Interrupts

   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;          // input
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 2;

   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO4 = 0;          // input
   GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;
   GpioCtrlRegs.GPAQSEL1.bit.GPIO4 = 2;

   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;          // input
   GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
   GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 2;

   GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;
   EDIS;

   GPIO_SetupXINT1Gpio(1);
   GPIO_SetupXINT2Gpio(4);
   GPIO_SetupXINT3Gpio(5);

   XintRegs.XINT1CR.bit.POLARITY = 0;
   XintRegs.XINT2CR.bit.POLARITY = 0;
   XintRegs.XINT3CR.bit.POLARITY = 0;

   XintRegs.XINT1CR.bit.ENABLE = 1;
   XintRegs.XINT2CR.bit.ENABLE = 1;
   XintRegs.XINT3CR.bit.ENABLE = 1;

   //
// Step 4. User specific code:
//
   LoopCount = 0;

   scia_fifo_init();       // Initialize the SCI FIFO
   scia_echoback_init();   // Initialize SCI for echoback
   scib_fifo_init();       // Initialize the SCI FIFO
   scib_echoback_init();   // Initialize SCI for echoback


   int i = 0;
   int j = 0;
   int k = 0;

   for(;;)
   {
       if(SCI_SELECT == 1)  // receives sensor data (X) from PC
       {
           while(SciaRegs.SCIFFRX.bit.RXFFST == 0) { } // wait for empty state
           while(SciaRegs.SCIFFRX.bit.RXFFST > 0)
           {
               recv[i] = SciaRegs.SCIRXBUF.all;

               if(recv[i] == '\n')
               {
                   scia_msg(recv);
                   token = strtok(recv,delimiter);
                   k = 0;
                   while(token != NULL)
                   {
                       x[k] = atof(token);
                       token = strtok(NULL,delimiter);
                       k++;
                   }

                   action();

                   for(j = 0; j <= buflen; j++)
                       recv[j] = '\0';
                   i = 0;
               }
               else i++;
           }
       }
       else if(SCI_SELECT == 2) //receives sensor data (X) from wireless receiver
       {
           while(ScibRegs.SCIFFRX.bit.RXFFST == 0) { } // wait for empty state
           while(ScibRegs.SCIFFRX.bit.RXFFST > 0)
           {
               recv[i] = ScibRegs.SCIRXBUF.all;

               if(recv[i] == '\n')
               {
                   scia_msg(recv);
                   token = strtok(recv,delimiter);
                   k = 0;
                   while(token != NULL)
                   {
                       x[k] = atof(token);
                       token = strtok(NULL,delimiter);
                       k++;
                   }

                   action();

                   for(j = 0; j <= buflen; j++)
                       recv[j] = '\0';
                   i = 0;
               }
               else i++;
           }
       }
       else if(SCI_SELECT == 3) //directly receives torque commands from PC
       {
           while(SciaRegs.SCIFFRX.bit.RXFFST == 0) { } // wait for empty state
           while(SciaRegs.SCIFFRX.bit.RXFFST > 0)
           {
               recv[i] = SciaRegs.SCIRXBUF.all;

               if(recv[i] == '\n')
               {
                   torque = atof(recv);

                   actuate(torque);

                   for(j = 0; j <= buflen; j++)
                       recv[j] = '\0';
                   i = 0;
               }
               else i++;
           }

       }

   }
}

interrupt void xint1_isr(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // toggle led state
    GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;   // toggle motor enable pin
    motor_active = !motor_active;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint2_isr(void)
{
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // toggle led state
    pwm_active = !pwm_active;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint3_isr(void)
{
    state = startup;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
//  scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
void scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //  Baud rate = LSPCLK/((BRR+1)*8)
    SciaRegs.SCIHBAUD.all = 0x0000;
    //SciaRegs.SCILBAUD.all = 0x0002;   //2000000

    ScibRegs.SCILBAUD.all = 0x0002;     //115200

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void scib_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //  Baud rate = LSPCLK/((BRR+1)*8)
    ScibRegs.SCIHBAUD.all = 0x0000;
    //ScibRegs.SCILBAUD.all = 0x0002;   //2000000

    ScibRegs.SCILBAUD.all = 0x0035; //115200

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all =a;
}

void scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF.all =a;
}

//
// scia_msg - Transmit message via SCIA
//
void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}

void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all = 0xE040;
    ScibRegs.SCIFFRX.all = 0x2044;
    ScibRegs.SCIFFCT.all = 0x0;
}


//
// End of file
//

void InitEPwm1()
{
   //
   // Setup TBCLK
   //
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   //
   // Setup shadow register load on ZERO
   //
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm1Regs.CMPA.bit.CMPA = DUTY_CYCLE_DEFAULT;     // Set compare A value

   //
   // Set actions
   //
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A,
                                                 // up count


}

void InitEPwm11()
{
   //
   // Setup TBCLK
   //
   EPwm11Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm11Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
   EPwm11Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm11Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm11Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm11Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm11Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   //
   // Setup shadow register load on ZERO
   //
   EPwm11Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm11Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm11Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm11Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm11Regs.CMPA.bit.CMPA = DUTY_CYCLE_DEFAULT;     // Set compare A value

   //
   // Set actions
   //
   EPwm11Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm11Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A,
                                                 // up count


}
