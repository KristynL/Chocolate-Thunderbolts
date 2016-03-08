// phasor.c: Impliments a Phasor Voltmeter
// Uses the 10-bit ADC on the f38x microcontroller and the
// multiplexer to measure input Vpeaks.  
// This program measures the peak voltages applied to pins P2.0 to P2.2 
// and the zero cross waves applied to P0.1 and P0.0
// Prints results to the hyperterminal
// ~C51~ 

#include <stdio.h>
#include <stdlib.h>
#include <c8051f38x.h>
#include <string.h>

// ANSI colors
#define	COLOR_BLACK		0
#define	COLOR_RED		  1
#define	COLOR_GREEN		2
#define	COLOR_YELLOW	3
#define	COLOR_BLUE		4
#define	COLOR_MAGENTA	5
#define	COLOR_CYAN		6
#define	COLOR_WHITE		7

// Some ANSI escape sequences
#define CURSOR_ON "\x1b[?25h"
#define CURSOR_OFF "\x1b[?25l"
#define CLEAR_SCREEN "\x1b[2J"
#define GOTO_YX "\x1B[%d;%dH"
#define CLR_TO_END_LINE "\x1B[K"

/* Black foreground, white background */
#define BKF_WTB "\x1B[0;30;47m"
#define FORE_BACK "\x1B[0;3%d;4%dm"
#define FONT_SELECT "\x1B[%dm"

#define MHZ 1000000L
#define SYSCLK (24*MHZ)
#define BAUDRATE 115200L


#define REF_ZERO P0_1    // pin to input ref zero cross signal
#define TEST_ZERO P0_0   // pin to input test zero cross signal

unsigned char overflow_count;



char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == (12*MHZ))
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == (24*MHZ))
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == (48*MHZ))
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12MHZ, 24MHZ, or 48MHZ
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency
	
	// Configure P2.0 to P2.3 as analog inputs
	P2MDIN &= 0b_1111_0000; // P2.0 to P2.3
	P2SKIP |= 0b_0000_1111; // Skip Crossbar decoding for these pins

	// Init ADC multiplexer to read the voltage between P2.0 and ground.
	// These values will be changed when measuring to get the voltages from
	// other pins.
	// IMPORTANT: check section 6.5 in datasheet.  The constants for
	// each pin are available in "c8051f38x.h" both for the 32 and 48
	// pin packages.
	AMX0P = LQFP32_MUX_P2_0; // Select positive input from P2.0
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
	#if (SYSCLK/BAUDRATE/2L/256L < 1)
		TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
		CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
		CKCON |=  0x08;
	#elif (SYSCLK/BAUDRATE/2L/256L < 4)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
		CKCON |=  0x01;
	#elif (SYSCLK/BAUDRATE/2L/256L < 12)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
	#else
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
		CKCON |=  0x02;
	#endif
	
	TL1 = TH1;     // Init timer 1
	TMOD &= 0x0f;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |= 0x20;                       
	TR1 = 1;       // Start timer1
	SCON = 0x52;
	
	return 0;
}

void PORT_Init (void)
{
	// P0MDOUT |= 0x10; // Enable UART TX as push-pull output	
	P0MDOUT|=0x10; // Enable Uart TX as push-pull output
	P1MDOUT|=0b_1111_1111;
	// P2MDOUT|=0b_1111_1111; // being used for ADC
	P3MDOUT|=0b_0000_0011;
	XBR0=0b_0000_0001; // Enable UART on P0.4(TX) and P0.5(RX)                    
	XBR1=0b_0101_0000; // Enable crossbar.  Enable T0 input.
	XBR2=0b_0000_0000;
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

void TIMER0_Init(void)
{
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	TR0=0; // Stop Timer/Counter 0
}



#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  2

void main (void)
{
	float v;
	unsigned char j;
  float time;
  float period;
  float halfperiod;
  float phase;
    						
	PORT_Init();     // Initialize Port I/O
	TIMER0_Init();
	
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	printf( FORE_BACK , COLOR_BLACK, COLOR_WHITE );
	printf( CLEAR_SCREEN );
	printf ("Phasor Voltmeter\n"
	        "Apply zero cross to P0_1, P0_0; Vpeak to P2_0, P2_2\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);
	printf( FORE_BACK, COLOR_BLACK, COLOR_WHITE );
	printf( "ÉÍÍÍÍÍÍÍÍÍÍÍÍËÍÍÍÍÍÍÍÍÍÍÍÍ»\n" );
  	printf( "º Vrms Ref   º            º\n" );
  	printf( "ÌÍÍÍÍÍÍÍÍÍÍÍÍÎÍÍÍÍÍÍÍÍÍÍÍÍ¹\n" );
  	printf( "º Vrms Test  º            º\n" );
  	printf( "ÌÍÍÍÍÍÍÍÍÍÍÍÍÎÍÍÍÍÍÍÍÍÍÍÍÍ¹\n" );
  	printf( "º Phase      º            º\n" );
  	printf( "ÌÍÍÍÍÍÍÍÍÍÍÍÍÎÍÍÍÍÍÍÍÍÍÍÍÍ¹\n" );
  	printf( "º Frequency  º            º\n" );
  	printf( "ÈÍÍÍÍÍÍÍÍÍÍÍÍÊÍÍÍÍÍÍÍÍÍÍÍÍ¼\n" );
    

  //1. measure half period
    TL0=0; 
		TH0=0;
		TF0=0;
		overflow_count=0;
		
		while(REF_ZERO!=0); // Wait for the signal to be zero
		while(REF_ZERO!=1); // Wait for the signal to be one
		TR0=1; // Start the timer
		while(REF_ZERO!=0) // Wait for the signal to be zero
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
		while(REF_ZERO!=1) // Wait for the signal to be one
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
		TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period!
		halfperiod=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);
  
  //2 wait for zero cross, quarter period//measure peak voltage of ref & test signal
  while(REF_ZERO!=1);
  while(REF_ZERO!=0);
  waitms(halfperiod/2);
  // measure peak voltage of ref & test signal
	// Start the ADC in order to select the first channel.
	// Since we don't know how the input multiplexer was set up,
	// this initial conversion needs to be discarded.
	AD0BUSY=1;
	while (AD0BUSY); // Wait for conversion to complete

	while(1)
	{
		
		printf("\x1B[6;1H"); // ANSI escape sequence: move to row 6, column 1

		for(j=0; j<NUM_INS; j++)
		{
			  AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
			  // Select next channel while ADC0 is busy
		  	switch(j)
			  {
				    case 0:
					    AMX0P=LQFP32_MUX_P2_0;    // input ref voltage
				    break;
				    case 1:
					    AMX0P=LQFP32_MUX_P2_2;    //input test voltage
				    break;
			  }
			
			  while (AD0BUSY); // Wait for conversion to complete
		  	v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
			  // Display measured values
		  	switch(j)
		  	{
			  	case 0:
			  	    printf( GOTO_YX , 7, 19);
    			    printf( FORE_BACK , COLOR_GREEN, COLOR_WHITE );
				      printf("%5.3f  ",  v/1.41421356); // ref RMS voltage
				  break;
				  case 1:
				      printf( GOTO_YX , 9, 19);
    		    	printf( FORE_BACK , COLOR_GREEN, COLOR_WHITE );
				      printf("%5.3f", v/1.41421356); // test RMS voltage
			  	break;
			  }
		}
        
    //6. measure phase by measuring change in zero time 
    TL0=0; 
		TH0=0;
		TF0=0;
		overflow_count=0;
    while(TEST_ZERO != 0);
    while(TEST_ZERO != 1);
    TR0 = 1; // start timer 0
        
    if(REF_ZERO == 1) // case where ref is at 1
    {
        while(REF_ZERO == 1)
        {
            if(TF0 == 1)
            {
                overflow_count++;
                TF0 = 0;
            }
        }
    }
        
    else // case where ref is at 0
    {
        while(REF_ZERO == 0)
        {
            if(TF0 == 1)
             {
                  overflow_count++;
                  TF0 = 0;
              }
        }
    }
        
    TR0 = 0; // stop the counter
    time = (overflow_count * 65536.0 + TH0 * 256.0 + TL0) * (12.0 / SYSCLK); 
        
		period = 2*halfperiod;
    if(time > (period / 4.0)) // negative phase
          phase = (-((period / 2.0) - time) / period ) * 360.0;

    else
          phase = (((period / 2.0) - time) / period ) * 360.0;
            
        
  
  printf( GOTO_YX , 11, 17);
  printf( FORE_BACK , COLOR_GREEN, COLOR_WHITE );
  printf("%5.3f ",phase);
  printf( GOTO_YX , 13, 18);
  printf( FORE_BACK , COLOR_GREEN, COLOR_WHITE );
  if(1/period < 1000 )
  printf("period=%5.6f frequency=%5.6f", period, 1/period);
  printf("\x1B[K");
  waitms(1000);  // Wait 100ms before next round of measurements.
	}  
}	



