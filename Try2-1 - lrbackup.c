// -O0
// 7372800Hz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;
unsigned char PortBRestore = 0;

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void back (void)            //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0x0A);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0x02);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0x00);
}

void soft_stop (void)       //soft stop(stops slowly)
{
  motion_set(0x0F);
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 timer1_init();
 sei(); //Enables the global interrupts
}

void velocity (unsigned char left_motor, unsigned char right_motor) 
{
OCR1AL = left_motor;
OCR1BL = right_motor;
}

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();
 motion_pin_config();
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}


//Main Function
int main(void)
{
	//...............DECLARED VARIABLES................//
	int kp = 2;
	int ki = 0;
	int kd = 4;
	int PID = 0, e=0, e_old=0;
	int e_i=0;
	int e_d=0;
	//unsigned int output=0;	

	//...............PRESET OR BIAS VARIABLES................//
	//int P=2; //to eliminate any zero bias value present in the error
	int S=90; //set velocity
	int lm=0,rm=0;

 init_devices();

 lcd_set_4bit();
 lcd_init();


while(1)
{
	l=ADC_Conversion(3);
	c=ADC_Conversion(4);
	r=ADC_Conversion(5);
	//lcd_print(1, 1, l, 3); //l is left reading
	//lcd_print(1, 5, c, 3); //c is centre reading
	//lcd_print(1, 9, r, 3); //r is right reading
	//_delay_ms(300);
	////////////////////////ERROR FUNCTION///////////////////////
	e = l-r;
	
/* 	if(e>=0){lcd_print(2, 1, e, 3);}
	else if(e<0){lcd_print(2, 1, (-1*e), 3);} */
	
	//////////////////////////PID/////////////////////////////////////
	e_i += e;
	e_d = e - e_old;
 	PID = (kp*e + ki*e_i + kd*e_d);
	e_old = e;	
	

	/////////////////////////DECIDING THE DIRECTION OF THE BOT////////
	if(c>20){forward();}
	if(PID>-70 && PID<70){lm=S-PID;rm=S+PID;}
	else if(PID>70 && PID<130){lm=0;rm=S+PID;}
	else if(PID>-70 && PID<-130){lm=S-PID;rm=0;}
	else if(PID>130){left();lm=S+PID;rm=S+PID;}
	else if(PID<-130){right();lm=S-PID;rm=S-PID;}
	//////////////////////////LIMIT CHECKS FOR THE BOTS///////////////
	if(lm>250){lm=250;}
	if(rm>250){rm=250;}
	if(lm<0){lm=0;}
	if(rm<0){rm=0;}
	//////////////////////////MISCELLANEOUS CHECKS////////////////////
	if(PID<20 && PID>-20){
		forward();
		velocity(90,90);
	}
	//////OVERCOMING THE THREE BLACK//////////////
	else if(l>100 && c>100 && r>100){ // Black Black Black
		forward();
		velocity(90,90);
		_delay_ms(100);}
	//////OVERCOMING THE Y JUNCTIONS///////////////
	else if(l<100 && c>130 && r>130){ // White  Black Black
		soft_left();
		velocity(90,90);
		_delay_ms(100);
	}
	else if(l>130 && c<130 && r>130){ // Black White Black
		soft_left();
		velocity(90,90);
		_delay_ms(100);		
	}	
	else {velocity(lm,rm);}
	
	
/*
while(1)
{
	forward();            //both wheels forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	back();               //both wheels backward						
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	left();               //Left wheel backward, Right wheel forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	right();              //Left wheel forward, Right wheel backward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_left();          //Left wheel stationary, Right wheel forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_right();         //Left wheel forward, Right wheel is stationary
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_left_2();        //Left wheel backward, right wheel stationary
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_right_2();       //Left wheel stationary, Right wheel backward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);
}
*/
}
}
