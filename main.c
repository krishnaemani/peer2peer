#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <hw_nvic.h>
#include <hw_types.h>
#include "tm4c123gh6pm.h"

#define RED_LED      PWM1_2_CMPB_R
#define GREEN_LED    PWM1_3_CMPB_R
#define BLUE_LED     PWM1_3_CMPA_R
#define DEN          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define TXRED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))   // red color led
#define RXGREEN_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))  // green color led

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
#define MAX_CHARS 80
#define MAX_ADDRESS 25
#define MAX_DATA 25

uint8_t valid_i, current_phase, count, field_n;
uint8_t value[MAX_DATA];
uint8_t destadd_tab[MAX_ADDRESS];
uint8_t seqid_tab[MAX_ADDRESS];
uint8_t channel_tab[MAX_ADDRESS];
uint8_t command_tab[MAX_ADDRESS];
uint8_t size_tab[MAX_ADDRESS];
uint8_t checksum_tab[MAX_ADDRESS];
uint8_t data_tab[MAX_DATA][MAX_DATA];
        uint8_t size = 0, command = 0, temp_i = 0, rx_phase = 0;
    uint8_t rx_checksum[MAX_ADDRESS];
    uint8_t rx_srctab[MAX_ADDRESS];
    uint8_t rx_sizetab[MAX_ADDRESS];
    uint8_t rx_seqidtab[MAX_ADDRESS];
    uint8_t rx_index;
    uint8_t rx_datatab[MAX_DATA];
    uint8_t rx_commandtab[MAX_ADDRESS];
    uint8_t rx_channeltab[MAX_ADDRESS];
    uint8_t received_tab[MAX_ADDRESS];
    uint8_t starttx = 0;

uint16_t rx_data, val_temp, rx_data;

int str_pos[50], str_tpos[50];
int add, channel, valid, i = 0;

char str[MAX_CHARS + 1];
char str2[MAX_CHARS + 1];
char str_type[50] = {0};
char str_after_delimit[MAX_CHARS + 1];

bool inprogress = false;
bool cs_enable, ack_enable, random_enable, startrx = false;
    bool valid_tab[MAX_ADDRESS];
    bool ackreq_tab[MAX_ADDRESS];
//RETRANSMIT
    uint8_t retranscount[MAX_ADDRESS];
    bool retrans_enable[MAX_ADDRESS];
    uint16_t retranstimeout[MAX_ADDRESS];
//for random
    uint16_t random_tab[MAX_ADDRESS] = {3,7,5,11,15,9,2,13,1,13,4,10,12,1,4,3,8,1,6,7,14,6,7,12,22};
    uint16_t random,b,T0 = 100,T = 500;
// for deadlock
    uint8_t oldrx_phase = 0, oldtx_phase = 0, deadrx_timeout, deadtx_timeout, countdead_rx, countdead_tx;
//pulse
    uint16_t pulse_timeout;
    bool pulse_enable;
//square
    uint16_t sqcycle,sqhigh_timeout,sqlow_timeout,temp_sqhtime,temp_sqltime;
    uint8_t sq_high,sq_low;
    bool square_enable;
    uint8_t srcadd_tab;
    bool sa_enable;
// sprintf
    char kali[100];
    uint8_t dec;

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
    // Enable GPIO port B, D, E, and F peripherals
    SYSCTL_RCGC1_R |= 0x00000002;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;  // enable LEDs and pushbuttons
    GPIO_PORTF_AFSEL_R = 0x0E; // auxillary function
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;
    // solder board
    GPIO_PORTA_DIR_R = 0xC0;
    GPIO_PORTA_DR2R_R = 0xC0;
    GPIO_PORTA_DEN_R = 0xC0;
    // configure UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    //Configure RGB
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    SYSCTL_SRPWM_R = 0;
    PWM1_2_CTL_R = 0;
    PWM1_3_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_2_LOAD_R = 1024;
    PWM1_3_LOAD_R = 1024;
    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
    PWM1_2_CMPB_R = 0;
    PWM1_3_CMPA_R = 0;
    PWM1_3_CMPB_R = 0;
    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN |PWM_ENABLE_PWM6EN |PWM_ENABLE_PWM7EN;
    // configure baudrate 115200 and 8 bit length
      UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
      UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
      UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
      UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
      UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
      UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
      //configuration for PC6 DE of the IC
      GPIO_PORTC_DIR_R |= 0x60;  // make bit 1 an outputs for DE
      GPIO_PORTC_DR2R_R |= 0x60; // set drive strength to 2mA (not needed since default configuration -- for clarity)
      GPIO_PORTC_DEN_R |= 0x70;  //  for data transmission used in the program for receveing data
      GPIO_PORTC_AMSEL_R &= ~0x30; // disabe the analog on PC5
      // configure UART1
      SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1 ;         // turn-on UART1, leave other uarts in same status
      GPIO_PORTC_DEN_R |= 0X30;                           // default, added for clarity
      GPIO_PORTC_AFSEL_R |= 0X30;                         // default, added for clarity
      GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
      // configure baudrate 38400 and 8 bit length
        UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx38400kHz), set floor(r)=65, where N=16
        UART1_FBRD_R = 7;                               // round(fract(r)*64)=7
        UART1_LCRH_R = 0X000000F6;//UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN; // configure for 9N1 w/ 16-level FIFO
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
        // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x9C40;                      // set load value to 40e3 for 1 kHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
// getting a string for string command comparision
char *getstring(uint8_t i_field)
{
return &str_after_delimit[str_tpos[i_field]];
}
//getting a number for the address, channel and value
uint16_t get_number(uint8_t i_field)
{
    uint16_t tem1;
   char *temp1_str[10] = {0};
   *temp1_str = &str_after_delimit[str_tpos[i_field]];
   tem1 = atoi (*temp1_str);
   return tem1;
}
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
// send packet// step 6 declaring a send packet function for storing the values and making a table
void sendpacket(uint8_t destadd, uint8_t commandtotab, uint8_t channeltotab, uint8_t sizetotab, uint8_t datatotab[MAX_DATA])
{
    checksum_tab[i] = 0;
    destadd_tab[i] = destadd;
    channel_tab[i] = channeltotab;    //temptab;
    size_tab[i] = sizetotab;
    for(temp_i = 0; temp_i<size_tab[i]; temp_i++)
    {
        data_tab[i][temp_i] = datatotab[temp_i];
        checksum_tab[i] = checksum_tab[i] + data_tab[i][temp_i];
    }
    if(ack_enable)
    {
        ackreq_tab[i] = 1;
        command_tab[i] = commandtotab | 0X80;
        valid_tab[i] = true;
    }
    else if(!ack_enable)
    {
        ackreq_tab[i] = 0;
        valid_tab[i] = true;
        command_tab[i] = commandtotab;
    }
    else
    {
        valid_tab[i] = false;
    }
    seqid_tab[i] = seqid_tab[i] + 1;
    checksum_tab[i] = ~(destadd_tab[i] + srcadd_tab + seqid_tab[i] + channel_tab[i] + command_tab[i] + size_tab[i] + checksum_tab[i]);
i++;
if(i == 25)
    i = 0;
}
// process data()
void  processtxdata()
{
    if(!(UART1_FR_R & UART_FR_BUSY) && (current_phase == 0))
        {
        UART1_LCRH_R = 0xF2;
        DEN = 1;
        UART1_DR_R = destadd_tab[valid_i];
        current_phase = 1;
        TXRED_LED = 1;
        oldtx_phase = current_phase - 1;
        }
    if(!(UART1_FR_R & UART_FR_BUSY) && (current_phase == 1))
    {
        UART1_LCRH_R = 0xF6;
        UART1_DR_R = srcadd_tab;
        current_phase = 2;
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 2))
    {
        UART1_DR_R = seqid_tab[valid_i];
        current_phase = 3;
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 3))
    {
        UART1_DR_R = command_tab[valid_i];
        current_phase = 4;
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 4))
    {
        UART1_DR_R = channel_tab[valid_i];
        current_phase = 5;
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 5))
    {
        UART1_DR_R = size_tab[valid_i];
        current_phase = 6;
        if(size_tab[valid_i] == 0)
        {
            current_phase = 7;
        }
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 6))
    {
        for(size = 0; size < size_tab[valid_i]; size++)
        {
            if(!(UART1_FR_R & UART_FR_TXFF))
            {
                UART1_DR_R = data_tab[valid_i][size];
            }
        }
        current_phase = 7;
    }
    if(!(UART1_FR_R & UART_FR_TXFF) && (current_phase == 7))
    {
        UART1_DR_R = checksum_tab[valid_i];
        current_phase = 8;
    }
    if(!(UART1_FR_R & UART_FR_BUSY) && (current_phase == 8))
    {
        sprintf(kali,"queuing : %d", dec);
        dec++;
        if(dec == 25)
            dec = 0;
        putsUart0(kali);
        DEN = 0;
        current_phase =0;
        TXRED_LED = 0;
        inprogress = 0;
        if(!ackreq_tab[valid_i])
        {
            valid_tab[valid_i] = 0;
            valid_i = 0;
            retranscount[valid_i] = 0;
        }
        else
        {
            if(random_enable)
                random = random_tab[valid_i + b];
            else
                random = 1;
            retranstimeout[valid_i] = (500 + random*(2^b)*100);
            b++;
            if(b == 25)
                b=0;
            retranscount[valid_i]++;
            putsUart0("Transmitting\r\n");
            if(retranscount[valid_i] >= 5)
            {
                retranscount[valid_i] = 0;
                valid_tab[valid_i] = 0;
                current_phase = 0;
                TXRED_LED = 1;
                ackreq_tab[valid_i] = 0;
                putsUart0("Error sending message");
            }
        }
    }
}
//process data
void processdata()
{
    uint8_t k[10];
    if(rx_checksum[rx_index] != received_tab[6 + rx_sizetab[rx_index]])
    {
        RXGREEN_LED = 1;
    }
    else
    {
    if(received_tab[3] == 0x00 || 0x80)
    {
        if(received_tab[4] == 1)
            {
            GREEN_LED = 0;
            BLUE_LED = 0;
            RED_LED = rx_datatab[0];
            k[0] = rx_datatab[0];
            if(received_tab[6] == 0)
                {
                RED_LED = 0;
                k[0] = 0;
                }
            }
    if(received_tab[4] == 2)
    {
        RED_LED = 0;
        GREEN_LED = rx_datatab[0];
        BLUE_LED = 0;
        k[0] = rx_datatab[0];
        if(received_tab[6] == 0)
        {
            GREEN_LED = 0;
            k[0] = 0;
        }
    }
    if(received_tab[4] == 3)
    {
        BLUE_LED = rx_datatab[0];
        RED_LED = 0;
        GREEN_LED = 0;
        k[0] = rx_datatab[0];
        if(received_tab[6] == 0)
        {
            BLUE_LED = 0;
            k[0] = 0;
        }
    }
    }
    rx_phase = 0;
    if(rx_commandtab[rx_index] == 0x79) // poll received
        {
        char textdisp [25];
        putsUart0("r\n");
        sprintf(textdisp,"Poll received from %d",rx_datatab[0]);
        putsUart0(textdisp);
        }
    if((rx_commandtab[rx_index] & 0x80) == 0x80) // ack sent
        {
        k[0] = rx_seqidtab[rx_index];
        sendpacket(rx_srctab[rx_index],0x70,00,01,&k[0]);   //sendpacket( dest, command, channel,size, data)
        }
    if(rx_commandtab[rx_index] == 0x70)         // ack received
        {
        for(temp_i = 0; temp_i < 25; temp_i++)
        {
            if(ackreq_tab[temp_i] == 1)
            {
                if(seqid_tab[temp_i] == rx_datatab[temp_i])
                {
                    ackreq_tab[temp_i] = 0;
                    retranscount[temp_i] = 5;
                    valid_tab[temp_i] = 0;
                    putsUart0("Acknowledged\r\n");
                }
            }
        }
        }
    if(rx_commandtab[rx_index] == 0x78)         //poll request
        {
        k[0] = srcadd_tab;
        sendpacket(rx_srctab[rx_index],0x79,0,1,&k[0]);
        }
    if(rx_commandtab[rx_index] == 0x20)         //data request(get command)
        {
        if(rx_channeltab[rx_index] == 1)
            sendpacket(rx_srctab[rx_index],0x21,0,1,&k[0]);  //sendpacket(add, command, channel, size, value);
        if(rx_channeltab[rx_index] == 2)
            sendpacket(rx_srctab[rx_index],0x21,0,1,&k[0]);
        if(rx_channeltab[rx_index] == 3)
            sendpacket(rx_srctab[rx_index],0x21,0,1,&k[0]);
        putsUart0("Data report successful\r\n");
        }
    if(rx_commandtab[rx_index] == 0x21)         // data report
        {
        char textdisp2 [25];
        putsUart0("r\n");
        sprintf(textdisp2,"Data received %d",rx_datatab[0]);
        putsUart0(textdisp2);
        }
    if((rx_commandtab[rx_index] == 0x7f))       //reset command
        {
        HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }
    if(rx_commandtab[rx_index] == 0x48)
    {
        if(received_tab[4] == 4)
        {
            RED_LED = rx_datatab[0];
            GREEN_LED = rx_datatab[1];
            BLUE_LED = rx_datatab[2];
        }
    }
    if(rx_commandtab[rx_index] == 0x7A)
    {
        srcadd_tab = rx_datatab[0];
        sa_enable = 1;
    }
    if(rx_commandtab[rx_index] == 0x02)
    {
        if(received_tab[4] == 5)
        {
            pulse_enable = true;
            RED_LED = rx_datatab[0];
            GREEN_LED = rx_datatab[0];
            BLUE_LED = rx_datatab[0];
            pulse_timeout = ((rx_datatab[1] << 8) | rx_datatab[2]);
        }
    }
    if(rx_commandtab[rx_index] == 0x03)
    {
        if(received_tab[4] == 6)
        {
            square_enable = true;
            sq_high = rx_datatab[0];
            sq_low = rx_datatab[1];
            sqhigh_timeout = ((rx_datatab[2] << 8) | rx_datatab[3]);
            sqlow_timeout = ((rx_datatab[4] << 8) | rx_datatab[5]);
            sqcycle = ((rx_datatab[6] << 8) | rx_datatab[7]);
            temp_sqhtime = sqhigh_timeout;
        }
    }
    }
}
//process receiver data
void processrxdata()
{
    while(!(UART1_FR_R & UART_FR_RXFE))
    {
        bool parity;
        rx_data = (UART1_DR_R & 0x2FF);
        parity = (rx_data & 0x200);
        if (parity)
        {
            startrx = true;
            rx_phase = 0;
            received_tab[rx_phase] = rx_data & 0xFF;
            RXGREEN_LED = 1;
            oldrx_phase = rx_phase - 1;
            if(received_tab[rx_phase] == (srcadd_tab) || (received_tab[rx_phase] == 0xff) || 8)
            {
                rx_checksum[rx_index] = received_tab[0];
                rx_phase++;
            }
        }
        else if (rx_phase != 0)
        {
            received_tab[rx_phase] = rx_data & 0xFF;
            rx_phase++;
            if(rx_phase == (7 + received_tab[5]))
            {
                rx_phase = 1;
                if(rx_phase == 1)
                {
                    rx_srctab[rx_index] = received_tab[rx_phase];
                    rx_checksum[rx_index] = rx_checksum[rx_index] + rx_srctab[rx_index];
                    rx_phase++;
                }
                if(rx_phase == 2)
                {
                    rx_seqidtab[rx_index] = received_tab[rx_phase];
                    rx_checksum[rx_index] = rx_checksum[rx_index] + rx_seqidtab[rx_index];
                    rx_phase++;
                }
                if(rx_phase == 3)
                {
                    rx_commandtab[rx_index] = received_tab[rx_phase];
                    rx_checksum[rx_index] = rx_checksum[rx_index] + rx_commandtab[rx_index];
                    rx_phase++;
                }
                if(rx_phase == 4)
                {
                    rx_channeltab[rx_index] = received_tab[rx_phase];
                    rx_checksum[rx_index] = rx_checksum[rx_index] + rx_channeltab[rx_index];
                    rx_phase++;
                }
                if(rx_phase == 5)
                {
                    rx_sizetab[rx_index] = received_tab[rx_phase];
                    rx_checksum[rx_index] = rx_checksum[rx_index] + rx_sizetab[rx_index];
                    rx_phase++;
                }
                if(rx_phase == 6)
                {
                    for(temp_i = 0; temp_i < rx_sizetab[rx_index]; temp_i++)
                        {
                        rx_datatab[temp_i] = received_tab[temp_i + 6];
                        rx_checksum[rx_index] = rx_checksum[rx_index] + rx_datatab[temp_i];
                        rx_phase++;
                        }
                }
                if(rx_phase == (6 + received_tab[5]))
                {
                    RXGREEN_LED = 0;
                    //startrx = false;
                    //rx_checksum[rx_index] -= received_tab[rx_phase];
                    rx_checksum[rx_index] = ~(rx_checksum[rx_index]);
                    processdata();
                    rx_phase = 0;
                }
            }
        }
    }
}
// deadlock process subroutine
void processdeadlock()
{
    if(oldrx_phase != rx_phase)
        {
            deadrx_timeout++;
            if(deadrx_timeout > 25000)
                rx_phase = 0;
        }
    else if(oldrx_phase == rx_phase)
        {
            countdead_rx++;
            if(countdead_rx > 25000)
            {
                rx_phase = 0;
                rx_data = 0;
            }
        }
    else if(oldtx_phase != current_phase)
        {
            deadtx_timeout++;
            if(deadtx_timeout > 25000)
                current_phase = 0;
        }
    else if (oldtx_phase == current_phase)
        {
            countdead_tx++;
            if(countdead_tx > 25000)
            {
                current_phase = 0;
                valid_tab[valid_i] = false;
                inprogress = false;
            }
        }
        oldrx_phase++;
        oldtx_phase++;
}
// Frequency counter service publishing latest frequency measurements every second
void Timer1Isr()
{
    if(!inprogress)
    {
        starttx++;
        for(temp_i = 0; temp_i < 25; temp_i++)
        {
            if((valid_tab[temp_i]) && (retranstimeout[temp_i] == 0))
            {
                if(cs_enable && starttx == 47)
                {
                    starttx = 0;
                    if(!startrx)
                    {
                        inprogress = true;
                        valid_i = temp_i;
                        current_phase = 0;
                        break;
                    }
                }
                else
                {
                    inprogress = true;
                    valid_i = temp_i;
                    current_phase = 0;
                    break;
                }
            }
        }
    }
    if(inprogress)// && (retranstimeout[temp_i] == 0))
    {
            //processing the transmission data
            processtxdata();
    }
    for(temp_i = 0; temp_i < 25; temp_i++)
    {
        if(retranstimeout[temp_i] > 0)
            retranstimeout[temp_i]--;
    }
    //receiver section
    processrxdata();
    //deadlock
    processdeadlock();
    //pulse
    if(pulse_enable)
    {
        pulse_timeout--;
        if(pulse_timeout == 0)
        {
            RED_LED = 0;
            GREEN_LED = 0;
            BLUE_LED = 0;
            pulse_enable = false;
        }
    }
    //square
    if(square_enable)
    {
        if(sqcycle != 0)
        {
            if(temp_sqhtime != 0)
                {
                    GREEN_LED = sq_high;
                    temp_sqhtime--;
                    if(temp_sqhtime == 0)
                        temp_sqltime = sqlow_timeout;
                }
            else if(temp_sqltime != 0)
                {
                    GREEN_LED = sq_low;
                    temp_sqltime--;
                    if(temp_sqltime == 0)
                    {
                        temp_sqhtime = sqhigh_timeout;
                        sqcycle--;
                    }
                }
        }
        else GREEN_LED = 0;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    // clear interrupt flag
}

//is command function: step 4
bool iscommand (char str_cmd[10], uint16_t min_args)
{
    if(strcmp("set", str_cmd) == 0)
    {
        if(min_args == 3)            //putsUart0("error\r\n\n\n");
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("reset", str_cmd) == 0)
    {
        if(min_args == 1)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("cs", str_cmd) == 0)
    {
        if(min_args == 1)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("random", str_cmd) == 0)
    {
        if(min_args == 1)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("get", str_cmd) == 0)
    {
        if(min_args == 2)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("sa", str_cmd) == 0)
    {
        if(min_args == 2)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("ack", str_cmd) == 0)
    {
        if(min_args == 1)
        {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
        }
    }
    if(strcmp("poll", str_cmd) == 0)
    {
        if(field_n > min_args)
        {
            return true;
        }
        else return false;
    }
    if(strcmp("rgb", str_cmd) == 0)
    {
        if(min_args == 5)
        {
            if(field_n > min_args)
            {
                return true;
            }
            else return false;
        }
    }
    if(strcmp("pulse", str_cmd) == 0)
    {
        if(min_args == 4)
        {
            if(field_n > min_args)
            {
                return true;
            }
            else return false;
        }
    }
    if(strcmp("square", str_cmd) == 0)
    {
        if(min_args == 7)
        {
            if(field_n > min_args)
            {
                return true;
            }
            else return false;
        }
    }
    else return false;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main()
{
    // Initialize hardware
    initHw();
    srcadd_tab = 8;
    GREEN_LED = 44;
    waitMicrosecond(10000);
    GREEN_LED = 0;
    putsUart0("Ready\r\n");
    putsUart0("Enter string\r\n");

    while(1)
    {
        // initialize for each time the data should be considered without previous values

        putsUart0("\r\n");
        add = 0;
        channel = 0;
        for(temp_i = 0; temp_i<MAX_DATA; temp_i++)
        {
        value[temp_i] = 0;
        }
        valid = 0;
        field_n = 0;
        int l = 81;
        for(l = 0; l<= 80; l++)
        {
            str[l] = 0;
            str_after_delimit[l] = 0;
            str2[l] = 0;
            str_pos[l] = 0;
            str_tpos[l] = 0;
            str_type[l] = 0;
        }
        uint8_t i,j,m = 0,n = 0,k;
        uint32_t x = 0;
        char* str3[MAX_CHARS + 1] = {0};
        char z;
        //char str_after_delimit[MAX_CHARS + 1];
        count = 0;
        while(1)
        {
            char c = getcUart0();
            if(c == '\b')
              {
                if(count > 0)
                {
                    count--;
                }
              }
            if(c == '\r')
            {
                str[count++] = '\0';
                break;
            }
            if(c >= ' ')
            {
                str[count++] = c;
            }
            if(count > MAX_CHARS)
            {
                str[count++] = '\0';
                break;
            }
        }
        for(j = 0 ; j <= strlen(str); j++)       // for loop; to check the string and categorizing the string
        {
            z = str[j];
            str[j] = tolower(z);
            str2[j] = str[j];
            if(str[j] >= 48 && str[j] <= 57)
            {
                str_type[j] = 'n';
            }
            if((str[j] >= 65 && str[j] <= 90) | (str[j] >= 97 && str[j] <= 122))
            {
                str_type[j] = 'a';
            }
            if((str[j] <= 47) ||(str[j] >= 58 && str[j] <= 64) || (str[j] >= 91 && str[j] <= 96) || (str[j] >= 123 && str[j] <= 127)) //str[j] >= 0 &&
            {
                str_type[j] = ' ';
            }
        }
//string compare for field  and position
    for(x = 0; x <= strlen(str_type);x++)
    {
        switch (str_type[x])
        {
        case ' ': str_after_delimit[x] = '\0';
        break;
        case 'a':  str_after_delimit[x] = str[x];
        if(str_type[x] != str_type[x+1])
        {
            field_n = field_n + 1;
            if(x != 0)
            {
                if (str_type[x] == str_type[x-1])
                {
                    str_pos[x] = 0;
                    //break;
                }
                else if(str_pos[x-1] == 0)
                {
                    str_pos[x] = x;
                }
            }
            if(x == 0)
            {
                str_pos[x] = x;
            }
        }
        if(str_type[x] == str_type[x+1])
        {
            field_n = field_n;
            if(x != 0)
            {
                if(str_type[x-1] == str_type[x])//(str_pos[x-1] != (x-1))
                {
                    str_pos[x] = 0;
                }
                else if(str_pos[x-1] == 0)
                {
                    str_pos[x] = x;
                }
            }
            if(x == 0)
            {
                str_pos[x] = x;
            }
        } break;
        case 'n': str_after_delimit[x] = str[x];
        if(str_type[x] != str_type[x+1])
        {
            field_n = field_n + 1;
            if(x != 0)
            {
                if (str_type[x] == str_type[x-1])
                {
                    str_pos[x] = 0;
                    break;
                }
                else if(str_pos[x-1] == 0)
                {
                    str_pos[x] = x;
                }
            }
            if(x == 0)
            {
                str_pos[x] = x;
            }
        }
        if(str_type[x] == str_type[x+1])
        {
            field_n = field_n;
            if(x != 0)
            {
                if(str_type[x] == str_type[x-1])
                {
                    str_pos[x] = 0;
                }
                else if(str_pos[x-1] == 0)
                {
                    str_pos[x] = x;
                }
            }
            if(x == 0)
            {
                str_pos[x] = x;
            }
        } break;
        default: break;
        }
    }
    for(m=0;m<strlen(str_type);m++) // minimizing the string by removing the zeros
    {
        if(str_pos[m] != 0)
        {
            str_tpos[n] = str_pos[m];
            n++;
        }
        else if (m == 0)
        {
            if(str_type[m] != ' ')
            {
                str_tpos[n] = 0;
                n++;
            }
        }
    }
// check error if the command is correct;
    *str3 = getstring(0);
  if((strcmp(*str3, "cs")==0) || (strcmp(*str3, "random")==0) || (strcmp(*str3, "ack")==0))
        {
            for(k=1;k<field_n;k++)
            {
                if(str_type[str_tpos[i]] == 'n')
                {
                    putsUart0("error argument1\r\n");
                }
            }
        }
  else
      {
          for(i=1;k<field_n;k++)
          {
              if(str_type[str_tpos[i]] == 'a')
              {
                  putsUart0("err\r\n");
              }
          }
      }
//is command verification of the string and the minimum arguments
  if (iscommand(*str3,(field_n - 1)))
  {
      // taking arguments for set
      if(strcmp(*str3, "set") == 0)
      {
          add = get_number(1);
          channel = get_number(2);
          value[0] = get_number(3);
          valid = 1;
          size = 1;
          sendpacket(add, command, channel, size, &value[0]);
      }
//taking arguments for reset
      else if(strcmp(*str3, "reset") == 0)
      {
          add = get_number(1);
          valid  = 1;
          size = 0;
          command = 0X7F;
          channel = 0;
          value[0] = 0;
          sendpacket(add, command, channel, size, &value[0]);//sendpacket(uint8_t destadd, uint8_t command, uint8_t channel = 0, uint8_t size, uint8_t data = 0)
      }
//taking arguments for carrier sense
      else if(strcmp(*str3, "cs") == 0)
      {
          if(strcmp(getstring(1), "on") == 0)
          {
              cs_enable = true;
              valid = 1;
          }
          else if(strcmp(getstring(1), "off") == 0)
          {
              cs_enable = false;
              valid = 1;
          }
          else valid = 0;
      }
//taking arguments for random
      else if(strcmp(*str3, "random") == 0)
      {
          if (strcmp(getstring(1), "on") == 0)
          {
              random_enable = true;
              valid =1;
          }
          else if (strcmp(getstring(1), "off") == 0)
          {
              random_enable = false;
              valid = 1;
          }
          else valid = 0;
      }
//taking arguments for get
      else if(strcmp(*str3, "get") == 0)
      {
          add = get_number(1);
          channel = get_number(2);
          valid = 1;
          value[0] = 0;
          command = 0X20;
          sendpacket(add, command, channel, 0, &value[0]);//sendpacket(uint8_t destadd, uint8_t command, uint8_t channel, uint8_t size, uint8_t data)
      }
//taking arguments for set address
      else if(strcmp(*str3, "sa") == 0)
      {
          add = get_number(1);
          value[0] = get_number(2);
          valid  = 1;
          size = 1;
          command = 0X7A;
          channel = 0;
          sendpacket(add, command, channel, size, &value[0]);//sendpacket(uint8_t destadd, uint8_t command, uint8_t channel, uint8_t size, uint8_t data)
      }
//taking arguments for acknowledgment
      else if(strcmp(*str3, "ack") == 0)
      {
          if (strcmp(getstring(1), "on") == 0)
          {
              ack_enable = true;
              valid = 1;
          }
          else if (strcmp(getstring(1), "off") == 0)
          {
              ack_enable = false;
              valid = 1;
          }
          else valid = 0;
      }
// taking arguments for poll
      else if(strcmp(*str3,"poll") == 0)
      {
          command = 0x78;
          channel = 0;
          size = 0;
          value[0] = 0;
          valid = 1;
          sendpacket(0xff,command, channel, size, &value[0]);
      }
//taking arguments for RGB
      else if(strcmp(*str3,"rgb") == 0)
      {
          command = 0x48;
          add = get_number(1);
          channel = get_number(2);
          size = 3;
          valid = 1;
          value[0] = get_number(3);
          value[1] = get_number(4);
          value[2] = get_number(5);
          sendpacket(add, command, channel, size, &value[0]);
      }
//taking arguments for pulse
      else if(strcmp(*str3,"pulse") == 0)
      {
          command = 0x02;
          add = get_number(1);
          channel = get_number(2);
          size = 3;
          valid = 1;
          value[0] = get_number(3);
          val_temp = get_number(4);
          value[1] = (val_temp / 255);
          value[2] = (val_temp % 255);
          sendpacket(add,command,channel,size,&value[0]);
      }
// taking arguments for square
      else if(strcmp(*str3,"square") == 0)
      {
          command = 0x03;
          add = get_number(1);
          channel = get_number(2);
          size = 8;
          valid = 1;
          value[0] = get_number(3);
          value[1] = get_number(4);
          val_temp = get_number(5);
          value[2] = (val_temp / 255);
          value[3] = (val_temp % 255);
          val_temp = get_number(6);
          value[4] = (val_temp / 255);
          value[5] = (val_temp % 255);
          val_temp = get_number(7);
          value[6] = (val_temp / 255);
          value[7] = (val_temp % 255);
          sendpacket(add,command,channel,size,&value[0]);
      }
  }
  if (!valid)
  {
      putsUart0("String error\r\n");
  }
//  putsUart0(*str3);
}
}


