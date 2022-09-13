// Kevin Cheung
// 400170725
// cheunk20
// Assigned Bus Speed - 24MHz
// Distance Status - PF4
// Displacement Status - PL3

#include <math.h>  
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "Systick.h"
#include "PLL.h"
#include "uart.h"
#include "onboardLEDs.h"

// Initializing Port E for the motor
void PortE_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		  // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	  // allow time for clock to stabilize
	GPIO_PORTE_DEN_R= 0b00001111;
	GPIO_PORTE_DIR_R |= 0b00001111;                           // make PE0 output  
	GPIO_PORTE_DATA_R = 0b00000000;                             // setting state to zero to drive the row, one to disable 
	return;
}

// Initializing Port L3 for external LED output
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                 //activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};        //allow time for clock to stabilize 
	GPIO_PORTL_DIR_R = 0b00001001;       								    
  GPIO_PORTL_DEN_R = 0b00001001;
	GPIO_PORTL_DATA_R = 0b00000000;  
	return;
}

// Default address (0101 0010) ToF sensor
uint16_t	dev=0x52; 
// Array to store the outputted ToF values
int tofValues[8] = {0,0,0,0,0,0,0,0};
// Variable to track the status of the ToF sensor
int status=0; 

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

// Initialize ToF debugger variables
uint8_t byteData, sensorState=0; //myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

// Function which sets up the ToF sensor
void ToF_Init(void){
	
	// Basic I2C Read Functions
  status = VL53L1_RdByte(dev, 0x010F, &byteData);
  status = VL53L1_RdByte(dev, 0x0110, &byteData);
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }

	FlashAllLEDs();
	
	// Clearing interrupt used for the next interrupt
	status = VL53L1X_ClearInterrupt(dev);

  // Initializing the sensor of the ToF
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	// Initializing the range function of the ToF
  status = VL53L1X_StartRanging(dev);
	Status_Check("StartRanging", status);
	
}

// Function which gets a distance measurement from the ToF chip and returns it
int getTOFDistance(void) {
	
	 // Waiting for ToF to be ready for sensing
	 while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
	
		// Getting the data from the ToF sensor
      dataReady = 0; 
	    status = VL53L1X_GetRangeStatus(dev, &RangeStatus); 
	    status = VL53L1X_GetDistance(dev, &Distance); 
      FlashLED4(1); 

			// Clear the interrupt to enable next interrupt
	    status = VL53L1X_ClearInterrupt(dev); 
			return Distance;
		
}

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

// Function that Initializes I2C communication to ToF sensor chip
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
	
	// Check if system ready
  while((SYSCTL_PRGPIO_R&0x0002) == 0){}; 

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;              // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
		
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE; // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011; // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//Reset the VL53L1X using XSHUT and pin PG0
void PortG_Init(void){
	
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};
    GPIO_PORTG_DIR_R &= 0x00; // PG0 to input (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01; // Disable Alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01; // Enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01; // Disable analog functionality on PG0
    return;
}

// Active-low shutdown input pin - board pulls it up to VDD to enable the sensor by default
// Driving the pin puts it into hardware standby - input not level-shifted
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01; // Set PG0 to out
    GPIO_PORTG_DATA_R &= 0b11111110; // PGO=0
    FlashAllLEDs(); // Flash LEDS
    SysTick_Wait10ms(10); 
    GPIO_PORTG_DIR_R &= ~0x01; // Set PG0 to Input (HIZ)
    
}

// Turn the stepper motor
void motorTurn(int speed){
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00001100;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00000110;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00000011;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00001001;
}

// Enables interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disables interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Waits for interrupt
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
// GPIO Port J = Vector 67
// Bit in interrupt register = 51

// Enable button J1 as an interrupt
void PortJ1_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize

  GPIO_PORTJ_DIR_R &= ~0x02;    // (c) make PJ1 in 

  GPIO_PORTJ_DEN_R |= 0x02;     //     enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; //  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;	//   disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;			//	enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x02;     // (d) PJ1 is edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02;    //     PJ1 is not both edges 
  GPIO_PORTJ_IEV_R &= ~0x02;    //     PJ1 falling edge event 
  GPIO_PORTJ_ICR_R = 0x02;      // (e) clear flag1
  GPIO_PORTJ_IM_R |= 0x02;      // (f) arm interrupt on PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // (g) priority 5
  NVIC_EN1_R |= 0x00080000;              // (h) enable interrupt 67 in NVIC
  EnableInt();           				// lets go
}

// Var which stores the current state of the program
int buttonState = 0;

// The Interrupt Handler when PJ1 button is clicked
void GPIOJ_IRQHandler(void){
  GPIO_PORTJ_ICR_R = 0x02; // Acknowledge flag4
	// Toggles the state into a new state
	if(buttonState == 1){
		buttonState = 0;
	}else{
		buttonState = 1;
	}	
}

// main function
int main(void){
	
	// Initialize functions
  PLL_Init();
	SysTick_Init();
	
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	ToF_Init();
	
	PortE_Init();
	PortL_Init();
	PortJ1_Init();
	
	// X displacement variable
	int x = 0;
	
	// Infinite loop for continous data
	while(1){	
		loop:	
		// Turn on the external LED			
		GPIO_PORTL_DATA_R = 0b00001000; 
		// Check if the button state is active
		if(buttonState == 1){
			// Variable counter for first loop
			int j = 0;
			while(j<=7){
				// Check if button state is still active
				if(buttonState == 1){
				// Variable counter for second loop
				int i = 1;
				// Turn off the external LED
				GPIO_PORTL_DATA_R = 0b00000000; 
					while(i<=64){
						
						// Turn the stepper motor
						motorTurn(1);
						i++;
					
						// Blink the LED every 45 degrees
						if(i == 64){
							GPIO_PORTF_DATA_R = 0b000010000;
							SysTick_Wait10ms(10);
							GPIO_PORTF_DATA_R = 0b000000000;
						}
					}
					
					// Store TOF Data in an array 
					tofValues[j] = getTOFDistance();
			
				}else{
					// Reset loop if state changed
					goto loop;
				}
				j++; 
			}
			// Reset state after complete cycle
			buttonState = 0; 		
		}else{		
			// The motor has run and ToF data has been collected
			// Double check array to make sure data exists in the array
			if(tofValues[0] != 0){				
				int degree = 0;				
				// Loop to parse through array
				for(int i = 0;i <=7;i++){
					// If there is any information inside the array at current index
					if(tofValues[i] != 0){
					// Calculate and send the xyz coordinates from the ToF sensor to python script
					sprintf(printf_buffer,"%d %d %d\r\n", x, (int)round(tofValues[i]*cos(degree*3.14159265/180)) ,(int)round(tofValues[i]*sin(degree*3.14159265/180)));
					UART_printf(printf_buffer);
					}

					// Reset array and increase degree
					tofValues[i] = 0; 
					degree +=45;
				}
				
				// Manually increase the x by 200mm
				x+=200; 
				sprintf(printf_buffer,"End\r\n"); 
				UART_printf(printf_buffer);
			}
			// Turn on the external LED
			GPIO_PORTL_DATA_R = 0b00001000; 
		}
	}
}



