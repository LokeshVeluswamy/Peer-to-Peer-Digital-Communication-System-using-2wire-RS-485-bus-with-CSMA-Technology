// PROJECT CODE
// LOKESH VELUSWAMY
// MAV ID 1001561782

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:

// Blue  LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <hw_nvic.h>
#include <hw_types.h>

#define max_msgs 26               ///maximum msg size
#define max_data 100              //// maximum data array
#define  max 80                   /// maximum array of type, position, field

uint16_t src_add=10;              /// Source Address

//#define  repeatcount 3000
#define  max_packet_size 50       /////Maximum Packet Size
char str[max];                    /// String which takes input
char* str1;                       /// string to store the string got i.e getstring is stored
// step3 position, type and field
char position[max];               /// string of position
char type[max];                   ////string of type
uint32_t field=0;                 ///// variables used in step 3
uint8_t i=0;
uint16_t temp=0;
//
// Step 4 address, channel and number
uint32_t Add=0;                    /// Variable used to store Address
uint32_t channel=0;                /////// Variable used to store channel
uint8_t value[max_msgs]={};       ///// Array to store the Value i.e Data
char* strcmd;
//
// General Variables for Data processing
    uint8_t count=0;
    char c;
    uint8_t y=0;
    uint8_t l=0;
    uint8_t valid=0;
//
    char rec[max];

// Step 6   To Define the Array
    uint8_t xi=0;
    uint8_t cmdset=0x00;
    uint8_t cmdrgb=0x48;
    uint8_t dest_add[max_msgs];
    uint8_t seqid[max_msgs];         //    Table of Contents
    uint8_t cmd[max_msgs];           //
    uint8_t chan[max_msgs];          //
    uint8_t size[max_msgs];          //
    uint8_t checksum[max_msgs];      //
    uint8_t data[max_msgs][max_data];//
    bool valid1[max_msgs]={0};                      /////////// this valid1 is for the valid's in the table of entries
    bool ackreq;                                     ////////////if ack on then we control ack using this variable else next variable
    bool acknreq;
    bool retry_nack;

    uint8_t retrans_count[max_msgs];        /////  Variables to Define the Retransmit Timeout
    uint32_t retrans_timeout[max_msgs];     /////
    uint8_t max_time=5;
    uint8_t seq_id_count=0;

/////
/// step 7 and 8 variables  (Tx and Rx of data)

    // for transmitter
    bool in_progress=0;                  /////// Variables to deine Transmission and Reception
    uint8_t current_index=0;
    uint16_t current_phase=0;
    uint8_t fi=0;
    uint16_t bl=0;

    // for reciever
    uint8_t rx_phase=0;
    uint8_t rx_data[max_packet_size];  /// Reciever Packet Size
    uint8_t rx_size=0;
    uint8_t dp_v=0;

/////// variables to process the packet      //// Processing of Packet
    uint8_t check=0;
    uint8_t checksumcheck=0;
    char* str3;
    char strp[300]={0};

///// stepp 9 variables  (Carrier Sense)
    bool csreq;                                      ////////////if cs on then we control ack using this variable else next variable
    // bool csnreq;                                     ///////temporarily not using this as we make the above command true or false we can use that
   #define  cstimeoutcount 1000
    uint32_t cstimeout=cstimeoutcount;
    uint8_t  jrx_phase=0;                       ////variable for carrier sense for junk values
    uint8_t jrx_size=0;                         /////variable for carieer sense for junk values

///////   step 10 variables  (Leds)
      #define  l_timeout_count 1                /////variables to Process Timeouts of Leds
        uint32_t l_timeout=l_timeout_count;
        bool error_rx=false;
        bool error_tx=false;
        uint8_t greenled=0;
        uint8_t redled=0;
#define led_high 255                          ////2 levels of Led Values for Set Command
#define led_low 0
        uint8_t leds_high=1;                          ////2 levels of Led Values for Set Command
        uint8_t leds_low=0;

   ///// step 11 variables  (Ack)
   uint8_t a_array[50]={0};                  ////Ack Variables
   uint8_t ack_cmd=0x70;
   uint8_t ack_size=0x01;

   ////step 12 variables (Deadlock)        /////Variables to Process the Deadlock
#define deadlock_timeout 1000   //// when there is deadlock for more than a minute reset the phases
   uint8_t oldtx_phase=0;
   uint8_t oldrx_phase=0;
   uint16_t  deadlock_timeoutcount=deadlock_timeout;

   ///step13 variables (Poll)              //////Variables to Process the Poll
    uint8_t cmd_poll=0x78;
    uint8_t cmd_pollresp=0x79;
    uint8_t poll_add=0xff;

    ///step14 variables (get)              /////Variables to process the Get Command
    uint8_t cmd_get=0x20;
    uint8_t cmd_getresp=0x21;
    uint8_t red_led=0;
    uint8_t green_led=0;
    uint8_t blue_led=0;


    // step 15 variables (random)         ////Variables to process the random Command
    bool random_on=false;
    uint8_t ran[10]={5,1,6,2,6,4};
    uint8_t r=0;
    uint32_t repeatcount=0;
   #define  t0 500
   #define  t 100

    // step 16 variables (random)
    uint8_t rst_cmd=0x7f;
    bool reset_flag=false;
    uint8_t reset_timeout=0;

    ////Extra credit Uart                 /// Variables to Process the UART DATA of Extra Credit
    char u_string[max_data]={};
    uint8_t u_length=0;
    uint8_t u_count=0;
    char ut_str[max_data]={};
    uint8_t uart_cmd=0x50;

    //////pulse Extracredit step          ///// Variables to Process the Pulse Extra Credit
    uint16_t pulse_count=0;
    uint16_t pulse_time=0;
    uint8_t pulse_amp=0;
    bool pulse_flag=false;

    //////square Extracredit step        ///Variables to Process the Square Extra Credit Step
    uint8_t sq_on_amp=0;
    uint8_t sq_off_amp=0;
    uint16_t sq_on_time=0;
    uint16_t sq_off_time=0;
    uint16_t sq_cycles=0;
    bool square_flag=false;
    bool square_on=false;
    bool square_off=false;
 ////variables to timeout                //Variables to process the timeout
    uint16_t sq_time=0;
        uint16_t sq_on_timeout=0;
        uint16_t sq_off_timeout=0;
        uint16_t sq_cycles_timeout=0;

  /////uart control extra credit step
        uint16_t baud_rate=0;
        uint8_t u_control=0;

    /////sawtooth extracredit step       /// Variables to Process the Sawtooth Extra Credit Step
            uint16_t st_time=0;
            uint8_t st_amp1=0;
            uint8_t st_amp2=0;
                uint8_t delta=0;
                uint16_t dwell=0;
                uint16_t st_cycle=0;
                bool sawtooth_flag=false;
       /////sawtooth variables
                uint8_t sawtooth_amp1=0;
                uint8_t sawtooth_amp2=0;
                 uint16_t st_cycles_timeout=0;
                 uint16_t dwell_timeout=0;
                 uint8_t inst_amp=0;

   ////////////Triangle extracredit step       /// Variables to Process the Triangle Extra Credit Step
                 uint16_t tr_time=0;
                 uint8_t tr_amp1=0;
                 uint8_t tr_amp2=0;
                 uint8_t tr_delta1=0;
                 uint8_t tr_delta2=0;
                 uint16_t tr_dwell=0;
                 uint16_t tr_cycle=0;
                 bool triangle_flag=false;
                 bool triangle_flag_on=false;
                 bool triangle_flag_off=false;
                 /////triangle variables
                 uint8_t triangle_amp1=0;
                 uint8_t triangle_amp2=0;
                 uint16_t tr_cycles_timeout=0;
                 uint16_t tr_dwell_timeout=0;
                 uint8_t inst_amp1=0;
                 int fract1=0;
                 float fract=0;
                 //////////#define for Board Led's and PCB led's
////////
#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define  D_EN (*((volatile uint32_t *)(0x42000000+(0x400063FC-0x40000000)*32+6*4)))
#define BRED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define BGREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    ///clock to eeprom
    SYSCTL_RCGCEEPROM_R |= 0x01;          //////configuration of EEPROM
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    EEPROM_EESIZE_R |=0X00010000;        ////
    EEPROM_EEBLOCK_R|=0X00;
    EEPROM_EEOFFSET_R|=0X00;


    // Enable GPIO port A and F and c peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x0e;  // bit 1,2,3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R |= 0x0e; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0e;  // enable LEDs and pushbuttons
    GPIO_PORTF_AFSEL_R |=0x0e;  /////configure pins for alternate functions
    GPIO_PORTF_PCTL_R=0x00005550;  ///// cofigure pctl
    GPIO_PORTF_DIR_R |= 0x01;    ////// configuring Push Botton SW1
    GPIO_PORTF_DR2R_R |= 0x01;   //
    GPIO_PORTF_DEN_R |= 0x10;    //
    GPIO_PORTF_PUR_R |= 0x10;    //
    GPIO_PORTA_DIR_R |= 0xc0;  // bit 1,2,3 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R |= 0xc0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xc0;  // enable LEDs and pushbuttons

    // Configure UART0 pins
 //   SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;           //////configured for uart control on uart 2 pins not configured
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // configure gpio pins to operate UART1 to 38400 Baud rate for transmission and reception
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART0, leave other uarts in same status
          GPIO_PORTC_DEN_R |= 0x70;
          GPIO_PORTC_DIR_R |= 0x60;// default, added for clarity
          GPIO_PORTC_AFSEL_R |=0x30;                         // default, added for clarity
          GPIO_PORTC_PCTL_R |= 0x00220000;
          D_EN=0;

          // configure UART1 to 38400 Baud rate for transmission and reception
                  UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
                     UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
                     UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx38400), set floor(r)=65, where N=16
                     UART1_FBRD_R = 7;                              // round(fract(r)*64)=12
                     UART1_LCRH_R = 0x86| UART_LCRH_WLEN_8 | UART_LCRH_FEN;; // configure for 8N1 w/ 16-level FIFO
                     UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX and module

             ////CONFIG PWM
                                          SYSCTL_RCGC0_R = SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
                                           __asm(" NOP");                                   // wait 3 clocks
                                           __asm(" NOP");
                                           __asm(" NOP");
                                           SYSCTL_RCGCPWM_R = SYSCTL_RCGCPWM_R1;
                                           SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
                                           SYSCTL_SRPWM_R = 0;                              // leave reset state
                                          PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1
                                          PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 2
                                          PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                                                            // output 3 on PWM0, gen 1b, cmpb
                                           PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
                                                                                            // output 4 on PWM0, gen 2a, cmpa
                                           PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                                                            // output 5 on PWM0, gen 2b, cmpb
                                           PWM1_2_LOAD_R = 256;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
                                           PWM1_3_LOAD_R = 256;
                                           PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
                                                                                            // invert outputs for duty cycle increases with increasing compare values
                                           PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
                                           PWM1_3_CMPB_R = 0;                               // green off
                                           PWM1_3_CMPA_R = 0;                               // blue off

                                           PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 1
                                           PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 2
                                           PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                                                            // enable outputs

          // Configure Timer 1 as the time base for interrupts
                 SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
                 TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                 TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
                 TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
               //  TIMER1_TAILR_R = 0x2625a00;                     // set load value to 40e6 for 1 Hz interrupt rate
                 TIMER1_TAILR_R = 0x9c40;                          // set load value to 40e3 for 1kHz interrupt rate
                 TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                 NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                 TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


}

//// code to intitalize uart
 void initUart(void)
        {
      SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;
      uint32_t inte=(40000000 / (16 * baud_rate));           ///// yields integer part
      float fract=((((40000000 / (16 * baud_rate))-inte)*64)+0.5);      ///// yields fractional part
           fract1=fract;
                 UART2_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
                 UART2_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
                 UART2_IBRD_R =inte;                            // r = 40 MHz / (Nxbaud_rate), set floor(r)=65, where N=16
                 UART2_FBRD_R =fract1;                           // round(fract(r)*64 + 0.5)
                 if((u_control) & (0x80)) UART2_LCRH_R|=0x01;
                 if((u_control) & (0x40))
                 {
                     if((u_control) & (0x00)) UART2_LCRH_R|=0x86;
                     if((u_control) & (0x10)) UART2_LCRH_R|=0x82;
                     if((u_control) & (0x20)) UART2_LCRH_R|=0x06;
                     if((u_control) & (0x30)) UART2_LCRH_R|=0x02;
                 }
                 if((u_control) && (0x03))
                     UART2_CTL_R= 0x0301;
                 if((u_control) && (0x02))
                     UART2_CTL_R= 0x0101;
                 if((u_control) && (0x01))
                     UART2_CTL_R= 0x0201;
        }


//// CODE FOR PWN RGB
setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}

//////codes to set Led's Uniquely without Affecting Other Bits
setRedColor(uint16_t red)
{
    PWM1_2_CMPB_R = red;
}
setGreenColor(uint16_t green)
{
    PWM1_3_CMPB_R = green;
}
setBlueColor(uint16_t blue)
{
    PWM1_3_CMPA_R = blue;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* string)
{
    uint8_t i;
    for (i = 0; i < strlen(string); i++)
      putcUart0(string[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// step 6  // to store the data entered by the user in array table for transmission
void sendpacket(uint8_t address,uint8_t command,uint8_t channell,uint8_t sizee, uint8_t* datavalue)
{
    uint8_t tempsum=0;

    for (xi=0;xi<26;xi++)
    {
    if(valid1[xi]==true);                //
     else                                /// refresh the string after the contents are invalidated to erase junk
         {                              //
         uint8_t s=0;                   //
         dest_add[xi]=0;                //     Code to Refresh the Entire String Before Writing a New String
         seqid[xi]=0;                   //
         cmd[xi]=0;                     //
         chan[xi]=0;                    //
         size[xi]=0;                    //
         checksum[xi]=0;               //
         for(s=0;s<26;s++)             //
             data[xi][s]=0;           /// 2 diemensional data array is cleared
         break;
         }
    }
    dest_add[xi]=address;                                       ///
    seqid[xi]=seq_id_count++;                                   ///
    sprintf(strp,"\n\r Queuing Message %d \n\r ",seqid[xi]);    //////to represent the queing of message to the user
    putsUart0(strp);                                            ///
    if(ackreq==true)                                            ///
    {
       if ((command !=ack_cmd))                                 ///
         {
         cmd[xi]=command|0x80;                                   //
         }
        else cmd[xi]=command;                                    ///
    }
    else cmd[xi]=command;                                        ///               CODE TO WRITE THE USER COMMANDS INTO THE TABLE AND MAKE VALID TRUE.
    chan[xi]=channell;                                           //
    size[xi]=sizee;                                              //
if (sizee==0)
{
    tempsum=0;                                                   //
    goto next;                                                   ///
}
    for (i=0;i<(size[xi]);i++)               //// loop to write the data to 2 d array
    {
    data[xi][i]=*datavalue;                                      ///
    datavalue++;                                                 ///
    }
    for (i=0;i<(size[xi]);i++)                                  ////
    {
    tempsum+=data[xi][i];                                        ///
    }
next:    checksum[xi]=(~((dest_add[xi])+(src_add)+(seqid[xi])+(cmd[xi])+(chan[xi])+(size[xi])+tempsum));    /// program to calculate the checksum
    valid1[xi]=true;                                                                                      //////////
    if(seq_id_count==25) seq_id_count=0;           //////if seqence id is more than msgs clear it

}


//// program to process the data recieved
void process_packet(void)                                       ///////////  PROCESS PACKET TO PROCESS THE PACKET
{
    check=rx_data[5];                                           ///
    checksumcheck=0;                                            ///
    for(i=0;i<=(check+5);i++)                                   ///
   checksumcheck += rx_data[i];                                 ///
    checksumcheck =~checksumcheck;                              ///
    if(checksumcheck==rx_data[check+6])                         ///
    {
        /// program to process the ack
      if ((rx_data[3] & 0x80))                          /////////always at the first so that processing is first checked for ack
        {
            a_array[0]=rx_data[2];                                           //
            sendpacket(rx_data[1],ack_cmd,0x00,ack_size,&a_array[0]);        //
            if(rx_data[3]==0xff)                                             //
            {
                reset_flag=1;                                                 //
            }
        }

////////////////////////////////////////////////////////////////////////////////////////
     if(rx_data[3]==0x70)            /////////code to process the acknowledgement received and process accordingly
        {
            for(i=0;i<max_msgs;i++)
                   {
                               if(valid1[i])
                               {
                                   if (seqid[i]==rx_data[6])
                                   {
                                       sprintf(strp,"Ack Recieved destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d\r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
                                       putsUart0(strp);
                                       valid1[i]=0;
                                       retrans_timeout[i]=0;
                                       break;

                                   }

                               }
        }
        }

/////////////////////////////////////////////////////////////////////////////////////////
   ///////code for Processing Triangle
     if((rx_data[3]==0x05)||(rx_data[3]==0x85))
                 {
                     if(rx_data[4]==0x08)
                     {
                         sprintf(strp,"\n\r Processing Triangle \n\r");
                         putsUart0(strp);

                         tr_amp1=rx_data[6];
                         tr_amp2=rx_data[7];
                tr_delta1=rx_data[8];
                tr_delta2=rx_data[9];
                         tr_time=rx_data[10];
                         tr_time=tr_time<<8;
                         tr_time|=rx_data[11];
                 tr_dwell=(tr_time*10);             ///On time Duration
                         tr_time=rx_data[12];
                         tr_time=tr_time<<8;
                         tr_time|=rx_data[13];
                 tr_cycle=tr_time;             //Off time Duration

                //// To enable processing square
                 triangle_flag=true;
                 triangle_flag_on=true;
                 triangle_amp1=tr_amp1;            ////local variable for amp1
                 triangle_amp2=tr_amp2;            ////local variable for amp2
                 tr_cycles_timeout=tr_cycle;      ////local variable for timeout
                 tr_dwell_timeout=0;

                     }
                 }

    ////////////////////////////////////////////////////////////////////////////////////////
     ////code for processing extra credit with sawtooth
             if((rx_data[3]==0x04)||(rx_data[3]==0x84))
             {
                 if(rx_data[4]==0x07)
                 {
                     sprintf(strp,"\n\r Processing Sawtooth \n\r");
                     putsUart0(strp);

                     st_amp1=rx_data[6];
                     st_amp2=rx_data[7];
             delta=rx_data[8];
                     st_time=rx_data[9];
                     st_time=st_time<<8;
                     st_time|=rx_data[10];
             dwell=(st_time*10);             ///On time Duration in ms
                     st_time=rx_data[11];
                     st_time=st_time<<8;
                     st_time|=rx_data[12];
             st_cycle=st_time;              //Number of Cycles

            //// To enable processing square
             sawtooth_flag=true;
             sawtooth_amp1=st_amp1;            ////local variable for amp1
             sawtooth_amp2=st_amp2;            ////local variable for amp2
             st_cycles_timeout=st_cycle;      ////local variable for timeout
             dwell_timeout=0;

                 }
             }

   //////////////////////////////////////////////////////////////////////////////////////
     ///////code for processing extrra credit uart control
     if((rx_data[3]==0x51)||(rx_data[3]==0xD1))
     {
         if(rx_data[4]==0x0b)
         {
             sprintf(strp,"\n\r Processing Uart Control \n\r");
                         putsUart0(strp);
                         baud_rate=rx_data[6];
                         baud_rate=baud_rate<<8;
                         baud_rate|=rx_data[7];
                         u_control=rx_data[8];
                         initUart();
         }
     }

   //////////////////////////////////////////////////////////////////////////////////////
     ////code for processing extra credit with square
        if((rx_data[3]==0x03)||(rx_data[3]==0x83))
        {
            if(rx_data[4]==0x06)
            {
                sprintf(strp,"\n\r Processing Square \n\r");
                putsUart0(strp);

                sq_on_amp=rx_data[6];
                sq_off_amp=rx_data[7];
                sq_time=rx_data[8];
                sq_time=sq_time<<8;
                sq_time|=rx_data[9];
        sq_on_time=sq_time;             ///On time Duration
                sq_time=rx_data[10];
                sq_time=sq_time<<8;
                sq_time|=rx_data[11];
        sq_off_time=sq_time;           //Off time Duration
                sq_time=rx_data[12];
                sq_time=sq_time<<8;
                sq_time|=rx_data[13];
        sq_cycles=sq_time;              //Number of Cycles

        //// To enable processing square
        square_flag=true;
        square_on=true;
        sq_on_timeout=sq_on_time;
        sq_cycles_timeout=sq_cycles;


            }
        }

//////////////////////////////////////////////////////////////////////////////
     ////code for processing extra credit with pulse
     if((rx_data[3]==0x02)||(rx_data[3]==0x82))
     {
         if(rx_data[4]==0x05)
         {
             sprintf(strp,"Processing Pulse");
             putsUart0(strp);
             pulse_amp=rx_data[6];
            pulse_time=(rx_data[7]);
            pulse_time=pulse_time<<8;
            pulse_time|=rx_data[6];
             pulse_count=(pulse_time*10);          //////// DURATION IN 1O ms
             pulse_flag=true;
         }
     }

  ///////////////////////////////////////////////////////////////////////////////
     ///////code for set address
     if((rx_data[3]==0x7a)||(rx_data[3]==0xfa))
     {
         sprintf(strp,"\n\rProcessing Set Address\n\r");
         putsUart0(strp);
                 EEPROM_EEOFFSET_R=0;
                 EEPROM_EERDWR_R=rx_data[6];
                 EEPROM_EEOFFSET_R=0;
                 src_add=EEPROM_EERDWR_R;
     }

  ///////////////////////////////////////////////////////////////////////////////
     ////////// code for processing uart extra credit with ack
      if((rx_data[3]==uart_cmd)||(rx_data[3]==0xD0))
       {
          if(rx_data[4]==4)
          {
       memset(strp, 0, sizeof(strp));
       putsUart0("\n\r Recieved Data String\n\r");
        for(i=0;i<rx_data[5];i++)
            strp[i]=rx_data[5+i];
       putsUart0(strp);
          }
          }

   ///////////////////////////////////////////////////////////////////////////////////
    /////code for processing the command get with ack
      if((rx_data[3]==cmd_get)||(rx_data[3]==0xa0))
                 {
                    if((rx_data[4])==0x01)
                    {
                     a_array[0]=red_led;
                     sprintf(strp,"Processing Data Request");
                     putsUart0(strp);
                     sendpacket(rx_data[1],cmd_getresp,rx_data[4],0x01,&a_array[0]);
                    }
                    else if((rx_data[4])==0x02)
                     {
                      a_array[0]=green_led;
                      sprintf(strp,"Processing Data Request");
                      putsUart0(strp);
                      sendpacket(rx_data[1],cmd_getresp,rx_data[4],0x01,&a_array[0]);
                     }
                    else if((rx_data[4])==0x03)
                     {
                     a_array[0]=blue_led;
                     sprintf(strp,"Processing Data Request");
                     putsUart0(strp);
                     sendpacket(rx_data[1],cmd_getresp,rx_data[4],0x01,&a_array[0]);
                    }
                    else if((rx_data[4])==0x09)
                     {
                      a_array[0]=PUSH_BUTTON;
                      sprintf(strp,"Processing Data Request");
                      putsUart0(strp);
                      sendpacket(rx_data[1],cmd_getresp,rx_data[4],0x01,&a_array[0]);
                     }
         }
      ///// code to process get response with ack
     if((rx_data[3]==cmd_getresp)||(rx_data[3]==0xa1))
                        {
                     sprintf(strp,"Data Report  destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d\r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
                     putsUart0(strp);
                        }

  ///////////////////////////////////////////////////////////////////////////////////////////
     ///code to process poll with ack
     if((rx_data[3]==cmd_poll)||(rx_data[3]==0xf8))
         {
         a_array[0]=rx_data[2];
         sprintf(strp,"Processing Poll");
         putsUart0(strp);
         sendpacket(rx_data[1],cmd_pollresp,0x00,0x01,&a_array[0]);
         }
     ////code to process pollresponse with ack
     if((rx_data[3]==cmd_pollresp)||(rx_data[3]==0xf9))
                {
             sprintf(strp,"Recieved Response from  destadd = %d\n\r",  rx_data[1]);
               putsUart0(strp);
              }

////////////////////////////////////////////////////////////////////////////////////////////////////
     ///// program to reset  with ack but handled in an different( after processing ack we should reset so ack is transmitted and board is reset)
     if(rx_data[3]==rst_cmd)
         {
             HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
         }

////////////////////////////////////////////////////////////////////////////////////////////////////
     if (rx_data[5]==0) goto next1;                     /////code for processing leds when there is only 7 byte packet size

////////////////////////////////////////////////////////////////////////////////////////////////////
     //// code for processing set and leds
     if((rx_data[3]==0x00) || (rx_data[3]==0x80))       ////program for setting leds
        {
         if(rx_data[4]==1)
         {
             red_led=rx_data[6];
            if(rx_data[6]==0)
            {
                setRedColor(led_low);
                sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5],leds_low, rx_data[7]);
                         putsUart0(strp);
            }
            else if(rx_data[6]==1)
                {
                    setRedColor(led_high);
                    sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], leds_high, rx_data[7]);
                             putsUart0(strp);
                }
         }
         if(rx_data[4]==2)
         {
          green_led=rx_data[6];
          if(rx_data[6]==0)
          {
              setGreenColor(led_low);
              sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], leds_low, rx_data[7]);
                       putsUart0(strp);
          }
         else if(rx_data[6]==1)
          {
            setGreenColor(led_high);
            sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], leds_high, rx_data[7]);
               putsUart0(strp);
          }
         }
          if(rx_data[4]==3)
          {
           blue_led=rx_data[6];
          if(rx_data[6]==0)
          {
              setBlueColor(led_low);
              sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], leds_low, rx_data[7]);
                       putsUart0(strp);
          }
          else if(rx_data[6]==1)
          {
              setBlueColor(led_high);
              sprintf(strp,"destadd = %d ,srcadd = %d,seqid = %d , command = %d, channel = %d , size = %d , data = %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], leds_high, rx_data[7]);
                       putsUart0(strp);
          }
          }
        }

 //////////////////////////////////////////////////////////////////////////////////////////////
     ////code for processing the rgb commands
     if ((rx_data[3]==0x48) || (rx_data[3]==0xc8))
        {
         if(rx_data[4]==10)
         {
         red_led=rx_data[6];
         green_led=rx_data[7];
         blue_led=rx_data[8];
         setRgbColor((rx_data[6]),(rx_data[7]),(rx_data[8]));   ////rgb configured to pwm
         sprintf(strp,"destadd = %d ,srcadd = %d ,seqid = %d , command = %d, channel = %d , size = %d , data = %d %d %d , checksum = %d \r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7],rx_data[8],rx_data[9]);
         putsUart0(strp);
         }
        }
 next1: ;
    }
     else                       //////code to process the timeouts and checksum errors for board leds
            {
            error_rx=true;                            ///////when there is error in checksum or someother problem
           greenled=1;
           BGREEN_LED=1;
            }

}  ///////// END OF THE PROCESS PACKET

//////////////////////////////////////////////////////////////////////////////
// Interrupts service routine to transmit and receive data
void Timer1Isr()
{
    uint8_t i=0;
    uint8_t sc=0;  /// local variables

  ///////////////////////////////////////////////////////////////////////////
    /// code to remove deadlock
    if((oldtx_phase !=0) | (oldrx_phase !=0))
        {
        if(oldtx_phase !=0)
         {
            if(oldtx_phase==current_phase)
            {
               if(deadlock_timeoutcount==0)
               {
                   current_phase=0;
                   oldtx_phase=0;
                   deadlock_timeoutcount=deadlock_timeout;
               }
               else
                   deadlock_timeoutcount--;

           }
            else
                {
                oldtx_phase=0;
                deadlock_timeoutcount=deadlock_timeout;
                }
        }
          else if (oldrx_phase !=0)
              {
             if(oldrx_phase==rx_phase)
              {
                if(deadlock_timeoutcount==0)
                               {
                                   rx_phase=0;
                                   oldrx_phase=0;
                                   deadlock_timeoutcount=deadlock_timeout;
                               }
                               else
                                   deadlock_timeoutcount--;
            }
             else
                 {
                 oldrx_phase=0;
                 deadlock_timeoutcount=deadlock_timeout;
                 }
          }
         }

 //////////////////////////////////////////////////////////////////
    ////// code to add some delay to blink and also to make led solid when there is an error
   if ((greenled==1)|(redled==1))
       {
       if((error_tx==true) | (error_rx==true))
       {
               l_timeout=l_timeout_count;
       }
       else
       {
           if (l_timeout>0)
           {
               l_timeout--;
               goto doagain;
           }
           else if(l_timeout==0)
           {
               if ((redled))
               {
                   redled=0;
                   BRED_LED=0;
                   l_timeout=l_timeout_count;
               }
               else if((greenled))
               {
                   greenled=0;
                   BGREEN_LED=0;
                   l_timeout=l_timeout_count;
               }
               }
       }
       }

///////////////////////////////////////////////////////////////////////////////////
///to find the valid entry
if(!in_progress)
    {
 for(i=0;i<max_msgs;i++)
        {
            if((valid1[i] && retrans_timeout[i])==1)
            {
                (retrans_timeout[i])--;
                if ((retrans_timeout[i])==0)
                    {
                    fi=i;
                    retrans_count[current_index]++;
                    goto fwd;
                     }
                goto repeat;
            }
        }

        for(i=0;i<max_msgs;i++)
        {
                    if(valid1[i])
                    {
                        if((retrans_timeout[i])==0)
                        {
                        fi=i;
                        retrans_count[fi]=0;
                        goto fwd;
                        }
                    }
        }
       goto repeat;
fwd:   in_progress=true;
        current_index=fi;
        current_phase=0;
    }

/////////////////////////////////////////////////////////////////////////////////
// To transmit the valid entry and other operations
if(in_progress)
    {

    if (csreq==true)                          ///code for n persistant to write it by sensing and adding a delay
          {
        if ((jrx_phase!=0)|(rx_phase!=0))
        {
            if((UART1_FR_R & UART_FR_RXFE)==0)
            {
                goto repeat;
            }
        }

        else if ((jrx_phase==0)|(rx_phase==0))
              {
              if(cstimeout>0)
              {
              cstimeout--;
              goto repeat;
              }
              else if (cstimeout==0)
              {
                  if((UART1_FR_R & UART_FR_BUSY))
                  {
                      //RED_LED^=1;
                     goto again;
          }
              }
          else goto repeat;
      }
      }
again:sc=size[current_index];                      /////Transmission of Valid Contents in the Table
     if(current_phase<=(sc+6))
{
    D_EN=1;
   switch(current_phase)
   {
   case 0:
       if(UART1_FR_R & UART_FR_TXFE)
       {
        if((UART1_FR_R & UART_FR_BUSY)==0)
       {
               UART1_LCRH_R  &= 0xFB;
           UART1_DR_R=dest_add[current_index];
       current_phase++;
         error_tx=false;                        ///make the error false when recieving new data
         redled=0;                  // mirror of Green led for processing
         BRED_LED=0;          ///////////turn off the led because when there was error in previous reception led will be on all time so clear before recieving new one
       goto doagain;
       }
       else goto repeat;
       }
       else
           goto repeat;
   case 1:
       if((UART1_FR_R & UART_FR_BUSY)==0)
       {
          UART1_LCRH_R |=0x04;
              if((UART1_FR_R & UART_FR_TXFE))
          {
              UART1_DR_R=src_add;
              current_phase++;
              goto again;
          }
          }
         else
          break;
   case 2:
          if((UART1_FR_R & UART_FR_TXFF)==0)
          {
              UART1_DR_R=seqid[current_index];
          current_phase++;
          goto again;
          }
        else
          break;
   case 3:
           if((UART1_FR_R & UART_FR_TXFF)==0)
           {
               UART1_DR_R=cmd[current_index];
           current_phase++;
           goto again;
           }
         else
           break;
   case 4:
            if((UART1_FR_R & UART_FR_TXFF)==0)
            {
                UART1_DR_R=chan[current_index];
             current_phase++;
            goto again;
            }
          else
             break;
   case 5:
            if((UART1_FR_R & UART_FR_TXFF)==0)
            {
              UART1_DR_R=size[current_index];
            current_phase++;
            goto again;
            }
          else
            break;
   default:
            if((UART1_FR_R & UART_FR_TXFF)==0)
                    {
                if((current_phase>=6)&(current_phase<(6+sc)))
                {
                    UART1_DR_R=data[current_index][current_phase-6];
                    current_phase++;
                    if (sc==0) break;       ///////////to transmit only 6 byte
                    goto again;

                 }
               if(current_phase==(6+sc))
                {
                    UART1_DR_R=checksum[current_index];
                    current_phase++;
                    sprintf(strp,"\n\r Transmitting Message %d, Attempt %d \n\r",seqid[current_index],retrans_count[current_index]);    //////to represent the queing of message to the user
                     putsUart0(strp);
                     redled=1;                         /////mirror of green led
                     BRED_LED=1;
                    goto again;
                 }
                    }
                else
                    break;
   }
    }

else if (current_phase==(6+sc+1))                    ////////// Processing the data after Recieving the data
{
 if ((UART1_FR_R & UART_FR_BUSY)==0)
 {
     D_EN=0;
     in_progress=false;
     // program to ack to retransmit
     if(((cmd[current_index] & 0x80)==0) || (cmd[current_index]==0xf0))
      {
      //   if (simply==2000)
       //  {
       valid1[current_index]=false;
     //    simply=0;
     //   }
        // else simply++;
      }
      else
      {
          if((retrans_count[current_index])==max_time)
          {
              valid1[current_index]=false;
                   redled=1;                                                       /////mirror of green led
                         BRED_LED=1;                                              ////// set solid when there is error
                         error_tx=true;                                          ///////when the retries reaches maximum then make the led solid
          }
              else          /////////program performs the random function
              {
                  if(random_on==0)
                  {
                       repeatcount=(t0+((pow(2,retrans_count[current_index]))*t));
                       retrans_timeout[current_index]=repeatcount;
                  }
                  else if(random_on)
                      {
                      if (r<7)
                      {
                      repeatcount=(t0+((pow(2,ran[r])*t)));
                      retrans_timeout[current_index]=repeatcount;
                      r++;
                      }
                      else
                          r=0;
                      }
             // retrans_timeout[current_index]=repeatcount;
            }
     current_phase=0;
     if (csreq==true)
     cstimeout=cstimeoutcount;
}
    }
    }
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////// Recieve Process after Transmission of Data

 /// program to recieve from UART1
repeat:if(current_phase==0)
{
    // UART1_LCRH_R |=0x04;
    if((UART1_FR_R & UART_FR_RXFE)==0)
        {
        D_EN=0;

           temp = UART1_DR_R & 0x2FF;
          if(temp & 0x200)
           {
               dp_v=0;
               temp &= 0xFF;
               if((temp!=src_add)&&(temp!=0xff))               ////////program to recieve and update carrier sense for busy line
               {
                   jrx_phase++;
                   goto repeat;
               }
               else if((temp==src_add)||(temp==0xff))
               {
                  error_rx=false;  //////// turn off the error when recieving new data
                  greenled=0;
                  BGREEN_LED=0;   ///////////turn off the led because when there was error in previous reception led will be on all time so clear before recieving new one
                   rx_phase=0;
                   rx_data[rx_phase]=temp;
                   rx_phase++;
                   goto repeat;
               }
           }
          else if (jrx_phase!=0)                     //////program to wait till all the other junk data is processed and carrier sense is again initiated
          {
              if(jrx_phase<5)
              {
                  jrx_phase++;
                  goto repeat;
              }
              else if(jrx_phase==5)
              {
                 jrx_size=temp & 0xFF;
                 jrx_phase++;
                 goto repeat;
              }
              else if((jrx_phase>5) && (jrx_phase<=(jrx_size+6)))
              {
                  jrx_phase++;
                  goto repeat;
              }
          }
           else if (rx_phase!=0)
              {
               if(rx_phase<5)
               {
                      rx_data[rx_phase]=temp & 0xFF;
                      rx_phase++;
                      goto repeat;
                  }
               else if(rx_phase==5)
               {
                   rx_data[rx_phase]=temp & 0xFF;
                   rx_phase++;
                   rx_size=temp & 0xFF;
                   goto repeat;
               }
               else if((rx_phase>5) && (rx_phase<=(rx_size+6)))
               {
                   rx_data[rx_phase]=temp & 0xFF;
                   rx_phase++;
                             greenled=1;
                             BGREEN_LED=1;                          ///////////turn on the led when the data is fully recieved
                   goto repeat;
                }

              }

        }
   if (rx_phase==(rx_size+6+1))
                        {
                         process_packet();           //// function to process the recieved packet of data
                         rx_phase=0;
                         if (csreq==true)
                         cstimeout=cstimeoutcount;
                        }
   if (jrx_phase==(jrx_size+6+1))                        ///// code to clear the junkvariables of carrier sense and to clear it
   {
       jrx_phase=0;
      if (csreq==true)
       cstimeout=cstimeoutcount;
          }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //////////exception cases where which involves processing of data after the process data to add delay or to introduce some transmission
  if(rx_data[3]==0xff)
          {
          if(reset_flag)
                    {
                      if(reset_timeout==3)
                      {
                       HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                      }
                       else
                          {
                          reset_timeout++;
                          }
                    }
         }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////exceptional code for pulse
    if(pulse_flag)
    {
        if(pulse_count!=0)
        {
            setGreenColor(pulse_amp);
            pulse_count--;
        }
        else if(pulse_count==0)
        {
            pulse_flag=false;
            pulse_count=0;
            pulse_amp=0;
            setGreenColor(pulse_amp);
        }
    }

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////Exceptional code for sawtooth                    //////////// SAWTOOTH
       if(sawtooth_flag)
         {
           if(st_cycles_timeout!=0)
            {
             if(dwell_timeout==0)
              {
                if((sawtooth_amp1+delta)<=st_amp2)
                {
                sawtooth_amp1=(sawtooth_amp1+delta);
                inst_amp=sawtooth_amp1;
                setGreenColor(inst_amp);       //////First Amplitude
                dwell_timeout=dwell;
                }
             else if((sawtooth_amp1+delta)>st_amp2)
                 {
                     inst_amp=0;
                     setGreenColor(inst_amp);
                     st_cycles_timeout--;
                     sawtooth_amp1=st_amp1;
                     dwell_timeout=dwell;
                 }
             }
        else if(dwell_timeout !=0)
        {
            dwell_timeout--;
        }
            }
         else if (st_cycles_timeout==0)               ///////To Disable Processing Square
         {
                 sawtooth_flag=false;
                 sq_on_time=0;
                 sq_cycles=0;
         }
      }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///   ////Exceptional code for triangle                 \\\\\\\\\\\\\\\ TRIANGLE
             if(triangle_flag)
               {
                 if(tr_cycles_timeout!=0)
                  {
                  if(triangle_flag_on)
                    {
                   if(tr_dwell_timeout==0)
                    {
                      if((triangle_amp1+tr_delta1)<=tr_amp2)
                      {
                      triangle_amp1=(triangle_amp1+tr_delta1);
                      inst_amp1=triangle_amp1;
                      setGreenColor(inst_amp1);       //////First Amplitude
                      tr_dwell_timeout=tr_dwell;
                      }
                   else if((triangle_amp1+tr_delta1)>tr_amp2)
                         {
                           inst_amp1=tr_amp2;
                           setGreenColor(inst_amp1);
                           triangle_amp1=tr_amp1;
                           tr_dwell_timeout=0;
                           triangle_flag_on=false;
                           triangle_flag_off=true;

                         }
                       }
                   else if(tr_dwell_timeout !=0)
                               {
                                   tr_dwell_timeout--;
                               }
                    }
                  else if(triangle_flag_off)
                  {
                      if(tr_dwell_timeout==0)
                              {
                                           if((triangle_amp2-tr_delta2)>=tr_amp1)
                                           {
                                           triangle_amp2=(triangle_amp2-tr_delta2);
                                           inst_amp1=triangle_amp2;
                                           setGreenColor(inst_amp1);       //////First Amplitude
                                           tr_dwell_timeout=tr_dwell;
                                           }
                                        else if((triangle_amp2-tr_delta2)<tr_amp1)
                                              {
                                                inst_amp1=tr_amp1;
                                                setGreenColor(inst_amp1);
                                                triangle_amp2=tr_amp2;
                                                tr_dwell_timeout=0;
                                                triangle_flag_off=false;
                                                triangle_flag_on=true;
                                                tr_cycles_timeout--;
                                              }
                                            }
                                        else if(tr_dwell_timeout !=0)
                                                    {
                                                        tr_dwell_timeout--;
                                                    }

                 }
                  }
               else if (tr_cycles_timeout==0)
               {
                       triangle_flag=false;
                       tr_cycle=0;
               }
            }

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////Exceptional code for square                         /////////// SQUARE
    if(square_flag)
      {
          if(sq_cycles_timeout!=0)
         {
          if(square_on)
          {
              if(sq_on_timeout!=0)
              {
                  setGreenColor(sq_on_amp);       //////First Amplitude
                  sq_on_timeout--;
              }
              else if(sq_on_timeout==0)
              {

                  square_on=false;
                  square_off=true;
                  sq_off_timeout=sq_off_time;
              }
          }
          else if(square_off)
          {
              if(sq_off_timeout!=0)
                           {
                               setGreenColor(sq_off_amp);          //////Second Amplitude
                               sq_off_timeout--;
                           }
                           else if(sq_off_timeout==0)
                           {
                               setGreenColor(sq_off_amp);
                               sq_cycles_timeout--;
                               if(sq_cycles_timeout!=0)
                               {
                               square_off=false;
                               square_on=true;
                               sq_on_timeout=sq_on_time;
                               }
                           }
          }
      }
    else if (sq_cycles_timeout==0)               ///////To Disable Processing Square
      {
          setGreenColor(sq_off_amp);
              square_flag=false;
              square_on=false;
              square_off=false;
              sq_off_time=0;
              sq_on_time=0;
              sq_cycles=0;
      }
   }

} //////////////////////////////////////////END OF RECEPTION OF DATA AND PROCESSING OF DATA

doagain:if ((UART1_FR_R & UART_FR_BUSY)==0)       /////////Data Enable is made zero after Busy check to prevent Junk Data Entry
           {
            D_EN=0;
           }
if((current_phase!=0)|(rx_phase!=0))
{
    if(current_phase!=0)
         oldtx_phase=current_phase;
    else
        oldrx_phase=rx_phase;
}
 TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////Wait MicroSecond to Introduce Delay
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// step 4 implementataion to find out address Channel and Number
int iscommand(char* strcmd, uint8_t minargs )      ///////// To compare the Entered String with Predefined Commands
{
  if((strcmp(&str[position[0]],strcmd)==0) && ((field-1) >= minargs))       /////change to (field-1) >= minargs) if to take more than minagrs as arguments
  {
      return 1;
  }
  else return 0;
}

//////////////////////////////////
void getu_string()       /////////////// Entire New String Processing For Uart Data Transmission
{
   uint8_t i=0;
   for(i=0;i<26;i++)
   {
       ut_str[i]=u_string[position[3]+i];
      if (ut_str[i]=='\0') break;
   }
}

///////////////////////////////////
char* getstring(uint8_t x)
{
    return (&str[position[x]]);
}

///////////////////////////////////       Array To Integer Conversion
int16_t getnumber(uint8_t x)
{
    return atoi(&str[position[x]]);
}

/////////////////////////////////////////////////
// step 5 to transmit and recieve data
void putcUart1(uint16_t cm)      ///// To Transmit character commands through UART1 extn of putsuart1
{
  D_EN=1;
   UART1_LCRH_R|=0x00;
while(UART1_FR_R & UART_FR_TXFF);
UART1_DR_R=cm;
}

/////////////////////////////////////////////////////////
void putsUart1(char* string)    ////// To Transmit string commands through UART1
{
    uint8_t i;
    D_EN=1;
    UART1_LCRH_R|=0x82;
    putcUart1(string[0]);
    UART1_LCRH_R^=0x02;
    for (i = 1; i < strlen(string); i++)
    {
        putcUart1(string[i]);
    }
}

//////////////////////////////////////////////////////////////
uint8_t getcUart1()           ////////// To Recieve commands through UART1
{
  D_EN=0;
 // uint16_t temp=0;
  uint16_t i=0;
  UART1_LCRH_R|=0x04;
    while (UART1_FR_R & UART_FR_RXFE);
    temp=UART1_DR_R & 0x2FF;
    if((temp && 512)==1)
    {
        rec[0]=(temp & 0xFF);
        for( i=1;i<10;i++)
        {
            rec[i]=UART1_DR_R & 0xFF;
        }
    }
    return 0;
}

////////////////////////////////////////////////////////////// Continuous Loop To Receive Data From User
//// steps 2 , 3 , 4 implementation
void dataproc(void)
{
    ///////////////////////////////////////////////////// Read the EEPROM INITIALLY TO GET THE ADDRESS FROM EEPROM
    EEPROM_EEOFFSET_R=0;
            if (EEPROM_EERDWR_R != 0XFFFFFFFF)
            {
                EEPROM_EEOFFSET_R=0;
                src_add=EEPROM_EERDWR_R;
            }
///////////////////////////////////////////////////////////////////
    BGREEN_LED=1;
    waitMicrosecond(500000);
    BGREEN_LED=0;
    putsUart0("\r\nReady\r\n");
    sprintf(strp,"\n\r Address is = %d \n\r",src_add);    //////to represent the queing of message to the user
    putsUart0(strp);
    for(;;)
    {
        count=0;
        // step 3 variables to be cleared when starting a new string
        memset(u_string,0,l);
        memset(ut_str,0,l);
        memset(str,0,l);
        memset(position,0,l);
        memset(type,0,l);
        uint8_t x=0;
        field=0;
        // step 4 variables to be cleared when starting a new string
        valid=0;
        while(1)
        {
 rxinput:      c = getcUart0();
               if (c <= 32)
               {
                if(c==8)
                {

                                 if(count==0) goto rxinput;
                                    else
                                    {
                                        count=count-1;
                                        u_count=count;                 ///////to solve the back space problem in n byte word data transfer
                                        u_string[u_count]='\0';                //////to solve the back space problem n byte word data transfer
                                        goto rxinput;
                                    }

                }
                else if (c==32)
                {
                    str[count+1]=32;
              //      u_count=count;
                }
                    else if (c==13)
                {
                    str[count++]='\0';
                //    u_count=count;
                    l=strlen(str);
                    for (y=0; y <= l ;y++)
                                               {
                                                   while (((str[y]>0) && (str[y]<48))|| ((str[y]>57) && (str[y]<65))||((str[y]>90) && (str[y]<97))||((str[y]>123) && (str[y]<127)))
                                                           {
                                                       str[y]=0;
                                                           }



                                                   if ((str[y]>=48) && (str[y]<=57))
                                                   {
                                                       position[x]=y;
                                                       type[x]='n';
                                                       x=x+1;
                                                       field=x;
                                                   while(((str[y]>=48) && (str[y]<=57)) && ((str[y+1]>=48) && (str[y+1]<=57))) y=y+1;
                                                   }

                                                   if ((str[y]>=65) && (str[y]<=90))
                                                   {
                                                       position[x]=y;
                                                       type[x]='a';
                                                       x=x+1;
                                                       field=x;
                                                      while(((str[y]>=65) && (str[y]<=90)) && ((str[y+1]>=65) && (str[y+1]<=90)))y=y+1;
                                                   }


                                                       if ((str[y]>=97) && (str[y]<=122))
                                                       {
                                                           position[x]=y;
                                                           type[x]='a';
                                                            x=x+1;
                                                           field=x;
                                                       while(((str[y]>=97) && (str[y]<=122)) && ((str[y+1]>=97) && (str[y+1]<=122))) y=y+1;
                                                       }


                }

         ////////processing Triangle
                    if ((iscommand("triangle",8)))
                                                       {
                                                        uint16_t time=0;
                                                        uint16_t time1=0;
                                                        putsUart0("\n\r Triangle is Enabled \n\r");
                                                        Add = getnumber(1);
                                                        channel=getnumber(2);
                                                value[0]= getnumber(3);    //data amp1
                                                value[1]=getnumber(4);     ////data amp2
                                                value[2]=getnumber(5);
                                                value[3]=getnumber(6);
                                                       time= getnumber(7);        //duration  for on cycle
                                                       time1=time;
                                                value[4]=((time>>8) & 0x00ff);
                                                value[5]=(time1 & 0x00ff);
                                                        time= getnumber(8);        //duration for off cycle
                                                        time1=time;
                                                value[6]=((time>>8) & 0x00ff);
                                                value[7]=(time1 & 0x00ff);
                                                        sendpacket(Add,0x05,channel,0x08,&value[0]);
                                                        valid=1;
                                                     }

                    //// extra credit sawtooth
                                      if ((iscommand("sawtooth",7)))
                                         {
                                          uint16_t time=0;
                                          uint16_t time1=0;
                                          putsUart0("\n\r sawtooth is Enabled \n\r");
                                          Add = getnumber(1);
                                          channel=getnumber(2);
                                  value[0]= getnumber(3);    //data amp1
                                  value[1]=getnumber(4);     ////data amp2
                                  value[2]=getnumber(5);
                                         time= getnumber(6);        //duration  for on cycle
                                         time1=time;
                                  value[3]=((time>>8) & 0x00ff);
                                  value[4]=(time1 & 0x00ff);
                                          time= getnumber(7);        //duration for off cycle
                                          time1=time;
                                  value[5]=((time>>8) & 0x00ff);
                                  value[6]=(time1 & 0x00ff);
                                          sendpacket(Add,0x04,channel,0x07,&value[0]);
                                          valid=1;
                                       }

               /*     if ((iscommand("uartcontrol",4)))
                             {
                                  uint16_t time=0;
                                  uint16_t time1=0;
                                 putsUart0("\n\r Uart Control is Enabled \n\r");
                                  str1=getstring(0);
                                  Add = getnumber(1);
                                  channel = getnumber(2);
                                 time= getnumber(3);        //duration  for on cycle
                                 time1=time;
                                 value[0]=((time>>8) & 0x00ff);
                                 value[1]=(time1 & 0x00ff);
                                 value[2]=getnumber(4);
                                  sendpacket(Add,0x51,channel,0x03,&value[0]);
                                  valid=1;
                             }   */


                    //// extra credit square
                    if ((iscommand("square",7)))
                       {
                        uint16_t time=0;
                        uint16_t time1=0;
                        putsUart0("\n\r square is Enabled \n\r");
                        Add = getnumber(1);
                        channel=getnumber(2);
                value[0]= getnumber(3);    //data amp1
                value[1]=getnumber(4);     ////data amp2
                        time= getnumber(5);        //duration  for on cycle
                        time1=time;
                value[2]=((time>>8) & 0x00ff);
                value[3]=(time1 & 0x00ff);
                        time= getnumber(6);        //duration for off cycle
                        time1=time;
                value[4]=((time>>8) & 0x00ff);
                value[5]=(time1 & 0x00ff);
                        time= getnumber(7);        //Number of Cycles
                        time1=time;
                value[6]=((time>>8) & 0x00ff);
                value[7]=(time1 & 0x00ff);    ///duration
                        sendpacket(Add,0x03,channel,0x08,&value[0]);
                        valid=1;
                     }
//////////////////////////////////// PULSE
                    if ((iscommand("pulse",4)))
                    {
                        uint16_t time=0;
                        uint16_t time1=0;
                        putsUart0("\n\r Pulse is Enabled \n\r");
                         Add = getnumber(1);
                         channel=getnumber(2);
                         value[0]= getnumber(3);    //data
                         time= getnumber(4);        //duration
                         time1=time;
                         value[1]=((time>>8) & 0x00ff);
                         value[2]=(time1 & 0x00ff);    ///duration
                         sendpacket(Add,0x02,channel,0x03,&value[0]);
                        valid=1;
                   }
///////////////////////////////////////// UART
                    if ((iscommand("uart",3)))
                    {
                        putsUart0("\n\r Uart Data Address is Enabled \n\r");
                         Add = getnumber(1);
                         channel=getnumber(2);
                         getu_string();
                         u_length=strlen(ut_str);
                         sendpacket(Add,uart_cmd,channel,u_length,(uint8_t*) &ut_str[0]);
                        valid=1;
                   }
 ///////////////    SET ADDRESS  Command
          if ((iscommand("sa",2)))
          {
              putsUart0("\n\r Set Address is Enabled \n\r");
               str1=getstring(0);
               Add = getnumber(1);
               value[0]=getnumber(2);
               sendpacket(Add,0x7A,0x00,1,&value[0]);
              valid=1;

          }
 //////////////////// RESET COMMAND
           if ((iscommand("reset",1)))
           {
               putsUart0("\n\r Reset is Enabled \n\r");
                str1=getstring(0);
                Add = getnumber(1);
                sendpacket(Add,rst_cmd,0x00,0x00,0x00);
                valid=1;
           }

////////////////////// SET COMMAND
        if ((iscommand("set",3)))
        {
            if((type[1]=='n')&(type[2]=='n')&(type[3]=='n')) /// take only if the other three parameters are numbers not arguments
              {
                putsUart0("\n\r Set is Enabled \n\r");
               str1=getstring(0);
               Add = getnumber(1);
               channel = getnumber(2);
              value[0]= getnumber(3);
             sendpacket(Add,cmdset,channel,1,&value[0]);
             valid=1;
             }
        }
///////////////// RGB COMMAND
        if ((iscommand("rgb",5)))
             {
                 if((type[1]=='n')&(type[2]=='n')&(type[3]=='n')&(type[4]=='n')&(type[5]=='n')) /// take only if the other three parameters are numbers not arguments
                   {
                     putsUart0("\n\r RGB is Enabled \n\r");
                    str1=getstring(0);
                    Add = getnumber(1);
                    channel = getnumber(2);
                   value[0]= getnumber(3);
                   value[1]= getnumber(4);
                   value[2]= getnumber(5);
                  sendpacket(Add,cmdrgb,channel,3,&value[0]);
                  valid=1;                                           ////////////valid to represent that theres no error in the command
                   }

             }
////////////////////// GET COMMAND
                if ((iscommand("get",2)))
                {
               putsUart0("\n\r Get is Enabled \n\r");
                 str1=getstring(0);
                 Add = getnumber(1);
                 channel = getnumber(2);
                 sendpacket(Add,cmd_get,channel,0x00,0x00);
                 valid=1;
                }
////////////////////// CARRIER SENSE COMMAND
               if ((iscommand("cs",1)))
               {
                   str1=getstring(0);
                   str3=getstring(1);
                               valid=1;
                               if(strcmp(str3,"on")==0)
                               {
                               csreq=true;
                               putsUart0("\n\r Carrier Sense is Enabled \n\r");
                               //retry_nack=true;
                               }
                               else if (strcmp(str3,"off")==0)
                               {
                                   csreq=false;
                                   putsUart0("\n\r Carrier Sense is Disabled \n\r");
                                  /// retry_nack=false;
                               }
               }
//////////////////////////// RANDOM COMMAND
                    if ((iscommand("random",1)))

                    {
                        str1=getstring(0);
                        str3=getstring(1);
                        valid=1;
                      if(strcmp(str3,"on")==0)
                        {
                        random_on=true;
                        putsUart0("\n\r Random is Enabled \n\r");
                        //retry_nack=true;
                        }
                       else if (strcmp(str3,"off")==0)
                       {
                          random_on=false;
                          putsUart0("\n\r Random is Disabled \n\r");
                        }
                 }
////////////////////////////// POLL COMMAND
                   if ((iscommand("poll",0)))
                   {
                  putsUart0("\n\r Poll is Enabled \n\r");
                  sendpacket(poll_add,cmd_poll,0x00,0x00,0x00);
                  valid=1;
                    }
//////////////////////////////// ACK COMMAND
               if((iscommand("ack",1)))
               {


               str1=getstring(0);
               str3=getstring(1);
               valid=1;
               if(strcmp(str3,"on")==0)
               {
               ackreq=true;
               putsUart0("\n\r Ack is Enabled \n\r");
               //retry_nack=true;
               }
               else if (strcmp(str3,"off")==0)
               {
                   ackreq=false;
                   putsUart0("\n\r Ack is Disabled \n\r");
                  /// retry_nack=false;
               }

              }
     break;
        }

else
  {
  putsUart0("\r\nInvalid Input\r\n");   /// unwanted character print invalid
   break;
 }
 }
                if (count>max)
               {
                   str[count++]='\0';
                   putsUart0("\r\n Wordlength Reached\r\n");
                   break;
               }

               else
               {
                   if ((c>=65)&&(c<=90)) c=c+32;             //////////code to make the input insensitive
                   str[count++]=c;
                   u_count=count;                           /////// TEMPORARY STRING TO DATA TO PROCESS THE UART COMMAND
                   u_string[u_count-1]=c;
               }

}      //// end of the continuous while loop to recieve the data


        if (valid==1);                   //////// IF NOT VALID THEN  ERROR IS DEFINED
          else
         {
         putsUart0("\n\rError\r\n");
        }

    }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)                              ////////////// MAIN FUNCTION
{
    // Initialize hardware
    initHw();
    dataproc();   /// the function to store and process the user command


    while(1);
}




/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
// TM4C123GH6PM STARTUP CCS

////////////////////////////////////////////////////////////////


//*****************************************************************************
//
// Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include <stdint.h>

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
extern void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);
extern void Timer1Isr(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
// To be added by user

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_TOP),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    IntDefaultHandler,                      // The SysTick handler
    IntDefaultHandler,                      // GPIO Port A
    IntDefaultHandler,                      // GPIO Port B
    IntDefaultHandler,                      // GPIO Port C
    IntDefaultHandler,                      // GPIO Port D
    IntDefaultHandler,                      // GPIO Port E
    IntDefaultHandler,                      // UART0 Rx and Tx
    IntDefaultHandler,                      // UART1 Rx and Tx
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
    IntDefaultHandler,                      // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
    IntDefaultHandler,                      // Watchdog timer
    IntDefaultHandler,                      // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    Timer1Isr,                              // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    IntDefaultHandler,                      // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    IntDefaultHandler,                      // Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    IntDefaultHandler,                      // GPIO Port F
    IntDefaultHandler,                      // GPIO Port G
    IntDefaultHandler,                      // GPIO Port H
    IntDefaultHandler,                      // UART2 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
    IntDefaultHandler,                      // Timer 3 subtimer A
    IntDefaultHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    IntDefaultHandler,                      // Quadrature Encoder 1
    IntDefaultHandler,                      // CAN0
    IntDefaultHandler,                      // CAN1
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Hibernate
    IntDefaultHandler,                      // USB0
    IntDefaultHandler,                      // PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port J
    IntDefaultHandler,                      // GPIO Port K
    IntDefaultHandler,                      // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
    IntDefaultHandler,                      // UART3 Rx and Tx
    IntDefaultHandler,                      // UART4 Rx and Tx
    IntDefaultHandler,                      // UART5 Rx and Tx
    IntDefaultHandler,                      // UART6 Rx and Tx
    IntDefaultHandler,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    IntDefaultHandler,                      // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
    IntDefaultHandler,                      // Wide Timer 0 subtimer A
    IntDefaultHandler,                      // Wide Timer 0 subtimer B
    IntDefaultHandler,                      // Wide Timer 1 subtimer A
    IntDefaultHandler,                      // Wide Timer 1 subtimer B
    IntDefaultHandler,                      // Wide Timer 2 subtimer A
    IntDefaultHandler,                      // Wide Timer 2 subtimer B
    IntDefaultHandler,                      // Wide Timer 3 subtimer A
    IntDefaultHandler,                      // Wide Timer 3 subtimer B
    IntDefaultHandler,                      // Wide Timer 4 subtimer A
    IntDefaultHandler,                      // Wide Timer 4 subtimer B
    IntDefaultHandler,                      // Wide Timer 5 subtimer A
    IntDefaultHandler,                      // Wide Timer 5 subtimer B
    IntDefaultHandler,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
    IntDefaultHandler,                      // GPIO Port M
    IntDefaultHandler,                      // GPIO Port N
    IntDefaultHandler,                      // Quadrature Encoder 2
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}
