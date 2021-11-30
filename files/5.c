// -O0
// 7372800Hz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <math.h>

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;
unsigned char PortBRestore = 0, ref=255;
float error=0,total_error=0,diff_error=0,kp=0,ki=0,kd=0,PID_cal=0,setpoint=0,position=0,prev_error=0;
	


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
 init_devices();

 lcd_set_4bit();
 lcd_init();
 /////////////////////////////////////////////////////////////////////////////////////
 kp=.5;
 ki=0.0;
 kd=0;
///////////////////////////////////////////////////////////////////////////////////////
while(1)
{	

//char error_print[10];
//sprintf(error_print,"%f",error);

	
	l=ADC_Conversion(3);
	c=ADC_Conversion(4);
	r=ADC_Conversion(5);
	lcd_print(1, 1, l, 3);
	lcd_print(1, 5, c, 3);
	lcd_print(1, 9, r, 3);
	/*
	lcd_print(2,1,error_print[1],4);
	lcd_print(2,2,error_print[2],4);
	lcd_print(2,3,error_print[3],4);
	lcd_print(2,4,error_print[4],4);*/
	if (l<20)
	l=0;
	if (c<20)
	c=0;
	if (r<20)
	r=0;
	float sum_sensor= (l+c+r);

	if( sum_sensor ==0)
		{
		hard_stop();
		sum_sensor=0.00001;
			//break;
		}
			position= ((l*0)+(c*1)+(r*2))/sum_sensor;//weighted mean
		if( sum_sensor ==0)
		{
		hard_stop();
		position=1;
		//break;
		}

	setpoint=1;
	error= setpoint-position;
	//_delay_ms(300);
	
	total_error+=error;
	diff_error=prev_error-error;
	PID_cal= (unsigned int) abs (kp*error+ki*total_error+kd*diff_error)/20;
//	if (PID_cal> 255)
//	PID_cal=255;
	prev_error=error;
	
	lcd_print(2, 1, 0, 3);
	
	//PID_cal= 
	if (error>0.05) //rotate left
	{
		left();
		velocity(PID_cal,PID_cal);
	
	}
	else if (error<-0.05)
	{	
		right(); 
		velocity(PID_cal,PID_cal);
		//	_delay_ms(300); 
	}





	else if (error<-0.005)
	{	
		soft_right();//right(); 
		velocity(PID_cal,PID_cal);
		//	_delay_ms(300); 
	}


	else if (error>0.005) //rotate left
	{
		soft_left();//left();
		velocity(PID_cal,PID_cal);
	
	}




	else
	{	forward();
		//velocity(PID_cal,PID_cal);
		lcd_print(2, 1, 1, 3);
		PID_cal=ref;
		velocity(PID_cal,PID_cal);
		}

		lcd_print(2, 5, PID_cal, 3);
	//	_delay_ms(300); 

/*	if ( abs(error)<.005 ) //to avoid 3 blacks on sesnor
	{
		forward();
	//	_delay_ms(100); 
	//	velocity(PID_cal,PID_cal);
		velocity(255,255);
	}
	
*/

/*	if (l<20 && c<20 && r<20 ) //to avoid 3 white on sesnor
	{

		forward();
		velocity(ref,ref);
		_delay_ms(200); */
			if (l>80 &&  c>80 &&  r>80 ) //to avoid 3 blacks on sesnor
	{
		forward();
		velocity(ref,ref);
		_delay_ms(1000); 
		//velocity(PID_cal,PID_cal);
		
	}


}


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
