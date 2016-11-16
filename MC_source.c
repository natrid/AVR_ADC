//Include needed libraries
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


//Definitions
#define F_CPU 16000000UL // CPU frequency for delay function
#define BAUDRATE 9600UL  //UART link speed
#define UBRVALUE (F_CPU/(16UL*BAUDRATE)-1) //prescaler for uart

void InitUART(void);
void InitADC(void);
void AdjustLevel(uint8_t voltage);
void InitProgram(void);
void SendString(char* StringPtr, uint8_t endline);
void ReadADC(void);
void EnablePWM(void);
void DisablePWM(void);
void AdjustOutput(uint8_t voltage);

//Global variables
volatile int dutyCycle = 195;
volatile double taso = 5;
volatile int tila = 1;
volatile uint16_t adc_value;
volatile uint16_t keskiarvo;

//Output voltages as X / 255 * 5V
uint8_t jannite1 = 51;
uint8_t jannite2 = 102;
uint8_t jannite3 = 153;
uint8_t jannite4 = 204;
uint8_t jannite5 = 255;

//Adjust parameter
int16_t error = 0;


//Receive an 8-bit data package through UART
unsigned char UARTReceive (void)
{
	while(!(UCSR0A) & (1<<RXC0));                   // wait while data is being received
	return UDR0;                                   // return 8-bit data
}

//Transmit an 8-bit data package through UART
void UARTTransmit (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}

//Function sends string through using UARTTransmit function
void SendString(char* StringPtr, uint8_t endline)
{
	while(*StringPtr != 0x00)
	{
		UARTTransmit(*StringPtr);
		StringPtr++;
	}

	//Change line if endline parameter is 1
	if(endline == 1 )
	{
		UARTTransmit('\r');
		UARTTransmit('\n');
	}
}

//Function reads ADC value from channel 1
void ReadADC(void)
{
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1 << ADIF))); // Wait for conversion to finish

	adc_value = ADCH;
}


//Function adjusts output voltage closer to wanted "voltage"
void AdjustOutput(uint8_t voltage)
{
	switch (voltage)
	{
		case 1:
		error = (jannite1-adc_value);
		if(adc_value < jannite1)
		{
			dutyCycle++;
		}
		else if(adc_value > jannite1+1)
		{
			dutyCycle = dutyCycle -2;
		}	
		break;

		case 2:
		if(adc_value < jannite2)
		{
			dutyCycle++;
		}
		else if(adc_value > jannite2+2)
		{
			dutyCycle = dutyCycle -1;
		}
		break;
		
		case 3:
		if(adc_value < jannite3-3)
		{
			dutyCycle++;
		}
		else if(adc_value > jannite3+3)
		{
			dutyCycle = dutyCycle -1;
		}
		break;
		
		case 4:
		if(adc_value < jannite4-1)
		{
			dutyCycle++;
		}
		else if(adc_value > jannite4+2)
		{
			dutyCycle = dutyCycle -1;
		}
		break;
		
		case 5:
		if(adc_value < jannite5)
		{
			dutyCycle++;
		}
		else if(adc_value > jannite5+2)
		{
			dutyCycle = dutyCycle -1;
		}
		break;
		
		default:
		break;
	}

	if (dutyCycle > 255) dutyCycle = 255;
	if (dutyCycle < 0) dutyCycle = 0;
	OCR0A = dutyCycle;
}


int main(void)
{
	char str[15];
	char str1[15];
	char str2[15];
	uint8_t DataByte;
	InitUART();
	InitADC();
	InitProgram();
	EnablePWM();

	while(1)
	{		
		//USART RX pollaus ja ulostulon säätö
		if((UCSR0A & (1<<RXC0)) != 0)
		{
			DataByte = UARTReceive();
			taso = DataByte;
			AdjustLevel(taso - '0');
			UARTTransmit(DataByte);
		}
				
		//Prosessin tahditus
		_delay_ms(100);
		
		//Jos kontrolleri on ajo tilassa
		if(tila)
		{
			//Lähetetään USART:lla ohjaustiedot
			strcpy(str1,"PWM: ");
			sprintf(str, "%d", dutyCycle);
			SendString(str1,0);
			SendString(str,0);
			SendString(" ADC arvo on: ",0);
			//dtostrf((double)adc_value,8,8,str2); //toinen tapa muodostaa string
			ReadADC();
			sprintf(str2, "%d", adc_value);
			SendString(str2, 1);
			
			//Laske ohjaus, mikä tahdittuu mainin mukaan
			//Nopeampi suoritus saavutetaan nopeuttamalla mainia...		
			AdjustOutput(taso - '0');
		}
	}
}

//Initialize UART
void InitUART(void)
{
	//prescaler value
	UBRR0 = UBRVALUE;

	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);     // enable receiver and transmitter
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format

}

//Initialize Analog to Digital conversion
void InitADC(void)
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //Prescaler at 128 for 125KHz
	ADMUX |= (1<<REFS0);
	ADMUX &= ~((1<<REFS1));
	ADMUX |= (1<<ADLAR);
	ADCSRA |= (1<<ADATE);                //Signal source free-running
	ADCSRA |= (1<<ADEN);                //Power up the ADC
	ADCSRA |= (1<<ADSC);                //Start converting

}

//Function for adjusting output close to the wanted parameter voltage
void AdjustLevel(uint8_t voltage)
{
	EnablePWM();
	switch (voltage)
	{
		case 0:
		PORTB |= (1<<PB5);
		_delay_ms(1000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 0;
		DisablePWM();
		break;

		case 1:
		PORTB |= (1<<PB5);
		_delay_ms(1000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 51;
		break;

		case 2:
		PORTB |= (1<<PB5);
		_delay_ms(2000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 102;
		break;
		
		case 3:
		PORTB |= (1<<PB5);
		_delay_ms(3000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 153;
		break;
		
		case 4:
		PORTB |= (1<<PB5);
		_delay_ms(4000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 204;
		break;
		
		case 5:
		PORTB |= (1<<PB5);
		_delay_ms(5000);
		PORTB &= ~(1<<PB5);
		dutyCycle = 255;
		break;
		
		default:
		break;
	}
	OCR0A = dutyCycle;
}

//Code for initializing the program
void InitProgram(void)
{
	//LED lights up for 5s on pin SCK/PB5
	DDRB = 0b00010000;
	PORTB |= (1<<PB5);
	_delay_ms(5000);
	PORTB &= ~(1<<PB5);


	//Interrupt for INT0
	PORTD |= (1 << PD2);
	EIMSK |= (1 << INT0); // INT0 enabled
	EICRA = (1<<ISC01); // falling edge generates the interrupt

	sei(); //set global interrupt flag
	
}

//Function enables PWM on port PD6
void EnablePWM(void)
{
	//PWM port PD6
	DDRD = (1 << PORTD6);
	PORTD |= (1<<PD6);

	//(1 << COM0A1) Clear OC0A on Compare Match, set OC0A at BOTTOM,
	//(non-inverting mode)
	//FAST PWM
	TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
	TIMSK0 = (1 << TOIE0); //TOIE0: Timer/Counter0 Overflow Interrupt Enable

	//Set dutyCycle
	OCR0A = dutyCycle;

	
	//Disable prescaler
	TCCR0B |= _BV(CS00);
}

//Function disbles PWM
void DisablePWM(void)
{
	//pwm rekisterit nolliksi
	DDRD &= ~(1<<PORTD6);
	PORTD &= ~(1<<PD6);
	TCCR0A = 0;
	TIMSK0 = 0;
	TCCR0B = 0;
	OCR0A = 0;

}


//Interrupt vector for INT0 / PD2 port
//Button is installed here
ISR(INT0_vect)
{

	if(tila == 0)
	{
		EnablePWM();
		tila = 1;
		PORTB |= (1<<PB5);
		_delay_ms(100);
		PORTB &= ~(1<<PB5);
	}
	else
	{
		DisablePWM();
		tila = 0;
		PORTB |= (1<<PB5);
		_delay_ms(100);
		PORTB &= ~(1<<PB5);
		
	}
	

}

