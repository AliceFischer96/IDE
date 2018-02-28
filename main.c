/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Spring 2016
*
* Filename: main_A2D_template.c
*/
 
#include "uart.h"
#include "MK64F12.h"
#include "stdio.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK 20485760u /* Default System clock value */

void initPDB(void);
void initGPIO(void);
void initFTM(void);
void initInterrupts(void);
void Button_Init(void);
void LED_Init(void);

void initPDB(void){
	//Enable clock for PDB module
	SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;
	
	// Set continuous mode, prescaler of 128, multiplication factor of 20,
	// software triggering, and PDB enabled
	PDB0_SC |= PDB_SC_CONT_MASK;					
	
	PDB0_SC |= PDB_SC_PRESCALER(7);
	
	PDB0_SC |= (PDB_SC_MULT(2));
	
	PDB0_SC |= PDB_SC_TRGSEL(15);
	
	PDB0_SC |= PDB_SC_PDBEN_MASK;
	
	
	//Set the mod field to get a 1 second period.
	//There is a division by 2 to make the LED blinking period 1 second.
	//This translates to two mod counts in one second (one for on, one for off)
  PDB0_MOD = DEFAULT_SYSTEM_CLOCK / 2560;
	
	//Configure the Interrupt Delay register.
	PDB0_IDLY = 10;
	
	//Enable the interrupt mask.
	PDB0_SC |= PDB_SC_PDBIE_MASK;
	
	//Enable LDOK to have PDB0_SC register changes loaded. 
	PDB0_SC |= PDB_SC_LDOK_MASK;
	
	return;
}

void initFTM(void){
	//Enable clock for FTM module (use FTM0)
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	
	//turn off FTM Mode to  write protection;
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	
	//divide the input clock down by 128,  
	FTM0_SC |= FTM_SC_PS_MASK;
	
	//reset the counter to zero
	FTM0_CNT &= ~(FTM_CNT_COUNT_MASK);
	
	//Set the overflow rate
	//(Sysclock/128)- clock after prescaler
	//(Sysclock/128)/1000- slow down by a factor of 1000 to go from
	//Mhz to Khz, then 1/KHz = msec
	//Every 1msec, the FTM counter will set the overflow flag (TOF) and 
	FTM0->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;
	
	//Select the System Clock 
	FTM0_SC &= ~(FTM_SC_CLKS_MASK);
	FTM0_SC |= 0x08;
	
	//Enable the interrupt mask. Timer overflow Interrupt enable
	FTM0_SC |= FTM_SC_TOIE_MASK;
	
	return;
}

void initGPIO(void){
	//initialize push buttons and LEDs
	Button_Init();
	LED_Init();
	
	//initialize clocks for each different port used.

	
	//Configure Port Control Register for Inputs with pull enable and pull up resistor

	// Configure mux for Outputs
	
	
	// Switch the GPIO pins to output mode (Red and Blue LEDs)
	
	
	// Turn off the LEDs

	// Set the push buttons as an input
	
	
	// interrupt configuration for SW3(Rising Edge) and SW2 (Either)
	
	
	return;
}

void initInterrupts(void){
	/*Can find these in MK64F12.h*/
	// Enable NVIC for portA,portC, PDB0,FTM0
	NVIC_EnableIRQ(PDB0_IRQn);
	NVIC_EnableIRQ(FTM0_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	//NVIC_EnableIRQ(ADC1_IRQn);
	
	return;
}

void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	//SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	 
	// Configure the Signal Multiplexer for GPIO
  PORTB_PCR22 = PORT_PCR_MUX(1);
	//PORTE_PCR26 = PORT_PCR_MUX(1);
	PORTB_PCR21 = PORT_PCR_MUX(1);
	
	// Switch the GPIO pins to output mode
	GPIOB_PDDR = (1 << 21) | (1 << 22);
	//GPIOC_PDDR = (1 << 6);
	//GPIOE_PDDR = (1 << 26);

	// Turn off the LEDs
	GPIOB_PDOR |= (1 << 21);
	GPIOB_PDOR |= (1 << 22);
	//GPIOE_PDOR |= (1 << 26);
   
}

void Button_Init(void){
	// Enable clock for Port C PTC6 button
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// Configure the Mux for the button
	PORTC_PCR6 = PORT_PCR_MUX(1);
	PORTA_PCR4 = PORT_PCR_MUX(1);

	// Set the push button as an input
	GPIOC_PDOR |= (1 << 6);
	GPIOA_PDOR |= (1 << 4);
	
	PORTA_PCR4 |= PORT_PCR_IRQC(9);
	PORTC_PCR6 |= PORT_PCR_IRQC(11);
}

 
void PDB_INIT(void) {
    //Enable PDB Clock
    SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;
    //PDB0_CNT = 0x0000;
    PDB0_MOD = 50000; // 50,000,000 / 50,000 = 1000

    PDB0_SC = PDB_SC_PDBEN_MASK | PDB_SC_CONT_MASK | PDB_SC_TRGSEL(0xf)
                                    | PDB_SC_LDOK_MASK;
    PDB0_CH1C1 = PDB_C1_EN(0x01) | PDB_C1_TOS(0x01);
}
 
void ADC1_INIT(void) {
    unsigned int calib;
 
    // Turn on ADC1
    //(Insert your code here.) - Done
		SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
		//SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
		//PORTE_PCR0 = PORT_PCR_MUX(3);
  
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
  
    PORTB_PCR2 = PORT_PCR_MUX(0);
    PORTB_PCR3 = PORT_PCR_MUX(0);
    PORTB_PCR10 = PORT_PCR_MUX(0);
    PORTB_PCR11 = PORT_PCR_MUX(0);
    PORTC_PCR11 = PORT_PCR_MUX(0);
    PORTC_PCR10 = PORT_PCR_MUX(0);
    
	
    // Configure CFG Registers
    // Configure ADC to divide 50 MHz down to 6.25 MHz AD Clock, 16-bit single ended
    //(Insert your code here.) - Done
		ADC1_CFG1 |= ADC_CFG1_ADIV(3);
		ADC1_CFG1 |= ADC_CFG1_MODE(3);
	
 
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC1_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC1_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC1_CLP0;
    calib += ADC1_CLP1;
    calib += ADC1_CLP2;
    calib += ADC1_CLP3;
    calib += ADC1_CLP4;
    calib += ADC1_CLPS;
    calib = calib >> 1;
    calib |= 0x8000;
    ADC1_PG = calib;
 
    // Configure SC registers.
    // Select hardware trigger.
    // (Insert your code here.) - Done
		ADC1_SC2 |= ADC_SC2_ADTRG_MASK;
 
    // Configure SC1A register.
    // Select ADC Channel and enable interrupts. Use ADC1 channel DADP3  in single ended mode.
    // (Insert your code here.) - Done
		ADC1_SC1A |= ADC_SC1_ADCH(3);
		ADC1_SC1A |= ADC_SC1_AIEN_MASK;
 
    // Enable NVIC interrupt
		//(Insert your code here.) - Done
    NVIC_EnableIRQ(ADC1_IRQn);
}
 
// ADC1 Conversion Complete ISR
void ADC1_IRQHandler(void) {
    // Read the result (upper 12-bits). This also clears the Conversion complete flag.
    unsigned short i = ADC1_RA >> 4;

    //Set DAC output value (12bit)
    //(Insert your code here.) - Done
		DAC0_DAT0L = i;
		DAC0_DAT0H = (i >> 8);
}

void DAC0_INIT(void) {
    //enable DAC clock
    SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;
    DAC0_C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
    DAC0_C1 = 0;
}


 
int main(void) {
    int i; char str[100];
   
    // Enable UART Pins
    //(Insert your code here.)
   
    // Initialize UART
    // (Insert your code here.) - Done
		uart_init();
  
    initPDB();
    initGPIO();
    initFTM();
    uart_init();
    initInterrupts();
               
    DAC0_INIT();
    ADC1_INIT();
    PDB_INIT();
 
    // Start the PDB (ADC Conversions)
    PDB0_SC |= PDB_SC_SWTRIG_MASK;
 
    for(;;) {
		sprintf(str,"\r\n Decimal: %d Hexadecimal: %x \n\r",ADC1_RA,ADC1_RA);
		put(str);
		for( i=0; i < 5000000; ++i ){
                       
        }
    }
 
    return 0;
}
 








