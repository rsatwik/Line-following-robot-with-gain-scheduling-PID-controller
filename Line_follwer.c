#include <avr/io.h>
#include <avr/delay.h>
#include <util/delay.h>

#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC
#define Kp	1.2
#define Ki	0
#define Kd	0.5

#define sbit(reg,bit)	reg |= (1<<bit)
#define cbit(reg,bit)	reg &= ~(1<<bit)

void init_ports();
void lcd_reset_4bit();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_home();
void lcd_cursor(char, char);
void lcd_print(char, char, unsigned int, int);
void lcd_string(char*);
void lcd_port_config();
void adc_init();
void adc_pin_config();
unsigned char ADC_Conversion(unsigned char);
void print_sensor(char,char,unsigned char);
void motion_pin_config();
void port_config();
void timer1_init();
void velocity(unsigned char,unsigned char);
void motion_set(unsigned char);

unsigned char ADC_Value,flag=0,flag1=0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char left_motor_control, right_motor_control,last_left=0,last_center=0,last_right=0;

float turn_error, error, last_error=0, control_var, derivative, integral=0;

unsigned int i; 
unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
} 

void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;
 Direction &= 0x0F; 		// removing upper nibble as it is not needed
 PortBRestore = PORTB; 		// reading the PORTB's original status
 PortBRestore &= 0xF0; 		// setting lower direction nibble to 0
 PortBRestore |= Direction; // adding lower nibble for direction command and
 							// restoring the PORTB status
 PORTB = PortBRestore; 		// setting the command to the port
} 

void forward (void) //both wheels forward
{
 motion_set(0x06);
}

void back (void) //both wheels backward
{
 motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
 motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
 motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void hard_stop (void) //hard stop(stop suddenly)
{
 motion_set(0x00);
}

 void soft_stop (void) //soft stop(stop slowly)
{
 motion_set(0x0F);
}

void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function to Reset LCD
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 in the upper nibble to initialize LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin
}

//Function to Initialize LCD
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28); //4-bit mode and 5x8 dot character font
	lcd_wr_command(0x01); //Clear LCD display
	lcd_wr_command(0x06); //Auto increment cursor position
	lcd_wr_command(0x0E); //Turn on LCD and cursor
	lcd_wr_command(0x80); //Set cursor position
}


//Function to initialize ports & LCD & ADC
void port_init()
{
 motion_pin_config();
 adc_pin_config();
 //lcd_port_config();
 adc_init();
 //lcd_init();
 //lcd_set_4bit();
 timer1_init();
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

void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}
	 
//Function to write command on LCD
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

//Function to write data on LCD
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}


void lcd_home()
{
	lcd_wr_command(0x80);
}


//Function to Print String on LCD
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

//Position the LCD cursor at "row", "column"

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

//Function to print any input value up to the desired digit on LCD
void lcd_print (char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
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

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor; 
 OCR1BH = 0x00;
 OCR1BL = right_motor;
}

int main()
{
	port_init();

	while(1)
	{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(5);	//Getting data of Right WL Sensor
 
//		print_sensor(2,1,3);		//Prints value of White Line Sensor Left
//		print_sensor(2,5,4);		//Prints value of White Line Sensor Center
//		print_sensor(2,9,5);		//Prints value of White Line Sensor Right

		turn_error = Right_white_line - Left_white_line;
		derivative = turn_error - last_error;
		integral += turn_error;
		control_var = Kp * turn_error + Ki * integral + Kd * derivative;
		if(Center_white_line > 40)
		{
			forward();
		}
		if(control_var<60 && control_var>-60)
		{
			left_motor_control = (90 + control_var);
			right_motor_control = (90 - control_var);
		}
		else if (control_var>60 && control_var<120)
		{
			right_motor_control = 0;
			left_motor_control = (90 + control_var);
		}
		else if (control_var>120)
		{
			right();
			right_motor_control = (90 + control_var);
			left_motor_control = (90 + control_var);
		}
		else if (control_var<-60 && control_var>-120)
		{
			left_motor_control = 0;
			right_motor_control = (90 - control_var);
		}
		else if (control_var<-120)
		{
			left();
			left_motor_control = (90 - control_var);
			right_motor_control = (90 - control_var);
		}
		last_error = turn_error;

		if(left_motor_control > 250)
		left_motor_control = 250;
		if(right_motor_control > 250)
		right_motor_control = 250;
		if(left_motor_control < 0)
		left_motor_control = 0;
		if(right_motor_control < 0)
		right_motor_control = 0;
//		lcd_print(1,1,left_motor_control,4);
//		lcd_print(1,5,right_motor_control,4);
//		lcd_print(1,9,control_var,4);
//		lcd_print(1,13,turn_error,4);
	
		
		if(Left_white_line < 20 && Right_white_line < 20 && Center_white_line < 20)
		{
			forward();
			velocity(200,200);
			_delay_ms(100);
		}
		else if(Center_white_line > 40 && control_var < 20 && control_var > -20)
		{
			forward();
			velocity(250,250);
		}
		else
			velocity(left_motor_control,right_motor_control);
//		for(unsigned int i=0;i<1000;i++);
//		last_left = Left_white_line;
//		last_right = Right_white_line;
	}
}	
