//#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SD21_I2C_ADDR	(0xC2 >> 1)
#define SD21_I2C_RADDR	((SD21_I2C_ADDR << 1) | 0x01)
#define SD21_I2C_WADDR	((SD21_I2C_ADDR << 1) | 0x00)
#define SD21_I2C_ADDR2	(0xC4 >> 1)

/* cf. Datasheet p.275 */
#define START		0x08
#define MT_SLA_ACK	0x18
#define MT_DATA_ACK	0x28

#define TWI_FREQ	400000UL
#define SERVO_NB	12
#define CALIB_CONSIGNE	1500

#define CONFIG_HW_REV3
//#define CONFIG_USART
#define CONFIG_I2C

//volatile uint16_t servo_consigne_us = 2200;
volatile uint16_t servo_consigne_us = CALIB_CONSIGNE; /* mid-pos by default */

#define RESO_US		25
volatile uint16_t consignes[SERVO_NB];

volatile uint8_t buffer_address;
//volatile uint8_t txbuffer[0xFF];
volatile uint8_t rxbuffer[0xFF];
volatile uint8_t *txbuffer = rxbuffer;

uint8_t calib_mode = 0;


#ifdef CONFIG_HW_REV3
static inline void vservo_output_enable()
{
	/* PD2 as output, set to 1 */
	DDRD |= (1 << 2);
	PORTD |= (1 << 2);
}
static inline void vservo_output_disable()
{
	/* PD2 as output, set to 0 */
	DDRD |= (1 << 2);
	PORTD &= ~(1 << 2);
}
#elif CONFIG_HW_REV2
// VServo /EN signal : pin D2 as Open-Drain output. Initial state +Vcc
static inline void vservo_output_enable()
{
	/* PD2 as output, set to 0 */
	PORTD &= ~(1 << 2);
	DDRD |= (1 << 2);
}

static inline void vservo_output_disable()
{
	/* PD2 as input, no pull-up */
	PORTD &= ~(1 << 2);
	DDRD &= ~(1 << 2);
}
#else
#define vservo_output_enable()
#define vservo_output_disable()
#endif

static inline uint8_t calib_button_pressed()
{
	return !(PINC & (1 << 3));
}

static inline uint8_t i2caddr_use_alternative()
{
	return !(PINC & (1 << 2));
}

//#define CONFIG_HAS_LED
#if defined(CONFIG_HAS_LED)
static inline void led_on()
{
//	PORTB |= 1 << 5;
	PORTD |= 1 << 0;
}

static inline void led_off()
{
//	PORTB &= ~(1 << 5);
	PORTD &= ~(1 << 0);
}
#else
#define led_on()
#define led_off()
#endif

#if defined(CONFIG_HAS_CALIB_BTN)
static inline char calib_btn_getvalue()
{
	return PORTC & (1 << 3);
}
#else
#define calib_btn_getvalue() (0)
#endif

void I2C_init(uint8_t address){
#ifdef CONFIG_I2C
	// load address into TWI address register
	TWAR = (address << 1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
#endif
}

//void I2C_stop(void){
//	// clear acknowledge and enable bits
//	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
//}

#ifdef CONFIG_I2C
ISR(TWI_vect){

	// temporary stores the received data
	uint8_t data;

//	PORTB |= 1 << 5;
	led_on();

	// own address has been acknowledged
	if( (TWSR & 0xF8) == TW_SR_SLA_ACK ){  
		buffer_address = 0xFF;
		// clear TWI interrupt flag, prepare to receive next byte and acknowledge
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
	}
	else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ){ // data has been received in slave receiver mode

		// save the received byte inside data 
		data = TWDR;

		// check wether an address has already been transmitted or not
		if(buffer_address == 0xFF){

			buffer_address = data; 

			// clear TWI interrupt flag, prepare to receive next byte and acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{ // if a databyte has already been received

			// store the data at the current address
			rxbuffer[buffer_address] = data;

			// if there is still enough space inside the buffer
			if(buffer_address < 0xFF){
				uint8_t servo_id = buffer_address / 3;
// ##############
				if (!calib_mode) {
					// buffer_address == servo number
//					servo_consigne_us = 1000;
					// rxbuffer[0] = LSB
					// rxbuffer[1] = MSB
					//servo_id = 1;
					servo_consigne_us = ((uint16_t)rxbuffer[servo_id*3+2]) << 8 | ((uint16_t)rxbuffer[servo_id*3+1] & 0xff);

					if (servo_id < SERVO_NB)
						consignes[servo_id] = servo_consigne_us / RESO_US;
//					for (unsigned char i = 0; i < SERVO_NB; i++)
//						consignes[i] = servo_consigne_us / RESO_US;
				}
// ##############
				// clear TWI interrupt flag, prepare to receive next byte and acknowledge
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
			}
			else{
				// Don't acknowledge
				TWCR &= ~(1<<TWEA); 
				// clear TWI interrupt flag, prepare to receive last byte.
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN); 
			}

			// increment the buffer address
			buffer_address++;
		}
	}
	else if( (TWSR & 0xF8) == TW_ST_DATA_ACK ){ // device has been addressed to be a transmitter

		// copy data from TWDR to the temporary memory
		data = TWDR;

		// if no buffer read address has been sent yet
		if( buffer_address == 0xFF ){
			buffer_address = data;
		}

		// copy the specified buffer address into the TWDR register for transmission
		TWDR = txbuffer[buffer_address];
		// increment buffer read address
		buffer_address++;

		// if there is another buffer address that can be sent
		if(buffer_address < 0xFF){
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{
			// Don't acknowledge
			TWCR &= ~(1<<TWEA); 
			// clear TWI interrupt flag, prepare to receive last byte.
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN); 
		}

	}
	else{
		// if none of the above apply prepare TWI to be addressed again
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	} 

//	PORTB &= ~(1 << 5);
	led_off();
}
#endif

#ifdef CONFIG_I2C
static void send_i2c(uint8_t SLA_W, uint8_t DATA)
{
	/* Send Start condition */
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	/* Wait for transmit */
	while (!(TWCR & (1<<TWINT)))
		;

	/* Check status */
	if ((TWSR & 0xF8) != START)
		;//ERROR();

	/* Send SLA_W */
	TWDR = SLA_W;
	TWCR = (1<<TWINT) | (1<<TWEN);

	/* Wait for transmit, and ACK/NACK has been received */
	while (!(TWCR & (1<<TWINT)))
		;

	if ((TWSR & 0xF8) != MT_SLA_ACK)
		goto send_stop;

	/* Send DATA */
	TWDR = DATA;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	/* Wait for transmit */
	while (!(TWCR & (1<<TWINT)))
		;

	if ((TWSR & 0xF8) != MT_DATA_ACK)
		goto send_stop;

send_stop:
	/* Send Stop condition */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}
#else
#define send_i2c(...)
#endif

#if defined(CONFIG_USART)
void usart_send_char(char c)
{
	if (c == '\n')
		usart_send_char('\r');

	while (!(UCSR0A & (1<<UDRE0)))
		;

	UDR0 = c;
}

void usart_printf(char *s)
{
	while(s && *s) {
		usart_send_char(*s);
		s++;
	}
}
#else
#define usart_send_char(c)
#define usart_printf(s)
#endif /* defined(CONFIG_USART) */

#define CONFIG_HAS_TIMER
#if defined(CONFIG_HAS_TIMER)
ISR(TIMER1_OVF_vect) {
#if 0
	static int i = 0;
	DDRD |= 1 << 1;

	if (!i) {
		i = 1;
		PORTD |= 1 << 1;
	} else {
		i = 0;
		PORTD &= ~(1 << 1);
	}
#else
		for (unsigned i = 0; i < 2500 / RESO_US; i++) {

//			for (j = 0; j < SERVO_NB; j++) {
//				if (i < consignes[j])
//					PORTB |= 1 << 5;
//				else
//					PORTB &= ~(1 << 5);
//			}
//			_delay_us(8);
#if defined(CONFIG_HW_REV2)
			/* Servo #01 = PORTC0 */
			if (i < consignes[0])
				PORTC |= 1 << 0;
			else
				PORTC &= ~(1 << 0);

			/* Servo #02 = PORTC1 */
			if (i < consignes[1])
				PORTC |= 1 << 1;
			else
				PORTC &= ~(1 << 1);
#elif defined(CONFIG_HW_REV3)
			/* Servos 01 & 02 were swap due to pcb routing constraints */
			/* Servo #02 = PORTC0 */
			if (i < consignes[1])
				PORTC |= 1 << 0;
			else
				PORTC &= ~(1 << 0);

			/* Servo #01 = PORTC1 */
			if (i < consignes[0])
				PORTC |= 1 << 1;
			else
				PORTC &= ~(1 << 1);
#endif

			/* Servo #03 = PORTB4 */
			if (i < consignes[2])
				PORTB |= 1 << 4;
			else
				PORTB &= ~(1 << 4);

			/* Servo #04 = PORTB3 */
			if (i < consignes[3])
				PORTB |= 1 << 3;
			else
				PORTB &= ~(1 << 3);

			/* Servo #05 = PORTB2 */
			if (i < consignes[4])
				PORTB |= 1 << 2;
			else
				PORTB &= ~(1 << 2);

			/* Servo #06 = PORTB1 */
			if (i < consignes[5])
				PORTB |= 1 << 1;
			else
				PORTB &= ~(1 << 1);

			/* Servo #07 = PORTB0 */
			if (i < consignes[6])
				PORTB |= 1 << 0;
			else
				PORTB &= ~(1 << 0);

			/* Servo #08 = PORTD7 */
			if (i < consignes[7])
				PORTD |= 1 << 7;
			else
				PORTD &= ~(1 << 7);

			/* Servo #09 = PORTD6 */
			if (i < consignes[8])
				PORTD |= 1 << 6;
			else
				PORTD &= ~(1 << 6);

			/* Servo #10 = PORTD5 */
			if (i < consignes[9])
				PORTD |= 1 << 5;
			else
				PORTD &= ~(1 << 5);

			/* Servo #11 = PORTD4 */
			if (i < consignes[10])
				PORTD |= 1 << 4;
			else
				PORTD &= ~(1 << 4);

			/* Servo #12 = PORTD3 */
			if (i < consignes[11])
				PORTD |= 1 << 3;
			else
				PORTD &= ~(1 << 3);
//			if (i < consignes[11])
//				PORTB |= 1 << 5;
//			else
//				PORTB &= ~(1 << 5);

			_delay_us(16); // RESO_US - 9
		}
//		sei();

		PORTC &= ~(1 << 0);
		PORTC &= ~(1 << 1);
		PORTB &= ~(1 << 4);
		PORTB &= ~(1 << 3);
		PORTB &= ~(1 << 2);
		PORTB &= ~(1 << 1);
		PORTB &= ~(1 << 0);
		PORTD &= ~(1 << 7);
		PORTD &= ~(1 << 6);
		PORTD &= ~(1 << 5);
		PORTD &= ~(1 << 4);
		PORTD &= ~(1 << 3);
#endif

}
#endif

int main(void)
{
	uint8_t i;
#if defined(CONFIG_LOOPBACK)
	uint8_t j = 5;//25; // to slow down the test (50Hz main loop, 500ms consigne update)
#endif /* defined(CONFIG_LOOPBACK) */
	uint8_t vservo_delay = 0x2F;
//	vservo_delay = 0;

	/* Safety: in case of transient reboot */
	vservo_output_disable();
	
	_delay_ms(100);

#if 1
	/* Pinmux: Servos signals as output GPIO */
	DDRC |= (1 << 0);
	DDRC |= (1 << 1);
	DDRB |= (1 << 4);
	DDRB |= (1 << 3);
	DDRB |= (1 << 2);
	DDRB |= (1 << 1);
	DDRB |= (1 << 0);
	DDRD |= (1 << 7);
	DDRD |= (1 << 6);
	DDRD |= (1 << 5);
	DDRD |= (1 << 4);
	DDRD |= (1 << 3);
//	DDRB |= (1 << 5); // Led on arduino nano
#endif

	/* C3 as input (Calib) */
	DDRC &= ~(1 << 3);
	/* C2 as input (I2CAddr) */
	DDRC &= ~(1 << 2);

#if defined(CONFIG_HAS_LED)
	/* B5 as output (Led) */
	//DDRB |= (1 << 5);
	DDRD |= (1 << 0);

	/* Flash Led at startup */
	led_on();
	_delay_ms(1);
	led_off();
#endif

	/* When calibration mode is set, set a fix point (middle) */
	if (calib_button_pressed()) {
//	if (calib_btn_getvalue()) {
		for (unsigned char i = 0; i < SERVO_NB; i++)
			consignes[i] = CALIB_CONSIGNE / RESO_US;
		calib_mode = 1;
	}

#if defined(CONFIG_HAS_TIMER)
	/* set timer to overflow every 20ms (50Hz period) */
	/* in clock is 16MHz */
	/* prediv : 16MHz / 256 = 62500 */
	/* 62500 / 50Hz = 1250 */
	TCCR1B |= 0x04; // prediv 256

	//mode 14 fast pwm
	TCCR1A |= (1 << WGM11); 
	TCCR1B |= (1 << WGM12)  | (1 << WGM13); 

	ICR1 = 1250;

	TIMSK1 |= 1 << TOIE1; // activate overflow interrupt
	//sei();
	//for (;;);
#endif

#if defined(CONFIG_USART)
	/* usart configuration pin */
	//PORTC.DIRCLR = PIN2_bm; /*!< PC2 (RDX0) as input pin */
	//PORTC.DIRSET = PIN3_bm; /*!< PC3 (TXD0) as output pin */
	DDRD |= (1 << 1); /* PD1(TXD) as output */

//#define FOSC 1843200 // Clock Speed
#define FOSC F_CPU
//#define BAUD 57600
#define BAUD 115200

	/*Set baud rate */
	const uint16_t ubrr = FOSC/16/BAUD-1;
	//UBRR0H = (unsigned char)(ubrr>>8);
	//UBRR0L = (unsigned char)ubrr;
	UBRR0H = 0;
	//UBRR0L = 25; //38400
	UBRR0L = 8; //115200
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	//UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	UCSR0C = (3<<UCSZ00);
#endif /* defined(CONFIG_USART) */

	//SCL frequency = CPU Clock frequency / (16 + 2(TWBR) * PrescalerValue)
	// Prescaler bit 1, 0 of TWSR:
	// 00b : Prescaler = 1
	// 01b : Prescaler = 4
	// 10b : Prescaler = 16
	// 11b : Prescaler = 64

#ifdef CONFIG_I2C
	TWSR |= 0x00;
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

	I2C_init(i2caddr_use_alternative() ? SD21_I2C_ADDR2 : SD21_I2C_ADDR);
	//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWEA);
#endif

	sei();

	for (;;) {
#if defined(CONFIG_HAS_TIMER)
/// //		servo_consigne_us = 1000;
/// 		cli();
/// 		// max value of SG90
/// 		//servo_consigne_us = 700;
/// 		//servo_consigne_us = 2450;
/// 		for (i = 0; i < 2500 / 25; i++) {
/// 
/// 			if (i < servo_consigne_us / 25)
/// 				PORTB |= 1 << 5;
/// 			else
/// 				PORTB &= ~(1 << 5);
/// 
/// 			//_delay_us(25);
/// 			_delay_us(12);
/// 		}
/// 		sei();
/// 
/// 		PORTB &= ~(1 << 5);
#else
//		servo_consigne_us = 1000;

		cli();
		// max value of SG90
		//servo_consigne_us = 700;
		//servo_consigne_us = 2450;
		for (i = 0; i < 2500 / RESO_US; i++) {

//			for (j = 0; j < 12; j++) {
//				if (i < consignes[j])
//					PORTB |= 1 << 5;
//				else
//					PORTB &= ~(1 << 5);
//			}
//			_delay_us(8);

#if defined(CONFIG_HW_REV2)
			/* Servo #01 = PORTC0 */
			if (i < consignes[0])
				PORTC |= 1 << 0;
			else
				PORTC &= ~(1 << 0);

			/* Servo #02 = PORTC1 */
			if (i < consignes[1])
				PORTC |= 1 << 1;
			else
				PORTC &= ~(1 << 1);
#elif defined(CONFIG_HW_REV3)
			/* Servos 01 & 02 were swap due to pcb routing constraints */
			/* Servo #02 = PORTC0 */
			if (i < consignes[1])
				PORTC |= 1 << 0;
			else
				PORTC &= ~(1 << 0);

			/* Servo #01 = PORTC1 */
			if (i < consignes[0])
				PORTC |= 1 << 1;
			else
				PORTC &= ~(1 << 1);
#endif

			/* Servo #03 = PORTB4 */
			if (i < consignes[2])
				PORTB |= 1 << 4;
			else
				PORTB &= ~(1 << 4);

			/* Servo #04 = PORTB3 */
			if (i < consignes[3])
				PORTB |= 1 << 3;
			else
				PORTB &= ~(1 << 3);

			/* Servo #05 = PORTB2 */
			if (i < consignes[4])
				PORTB |= 1 << 2;
			else
				PORTB &= ~(1 << 2);

			/* Servo #06 = PORTB1 */
			if (i < consignes[5])
				PORTB |= 1 << 1;
			else
				PORTB &= ~(1 << 1);

			/* Servo #07 = PORTB0 */
			if (i < consignes[6])
				PORTB |= 1 << 0;
			else
				PORTB &= ~(1 << 0);

			/* Servo #08 = PORTD7 */
			if (i < consignes[7])
				PORTD |= 1 << 7;
			else
				PORTD &= ~(1 << 7);

			/* Servo #09 = PORTD6 */
			if (i < consignes[8])
				PORTD |= 1 << 6;
			else
				PORTD &= ~(1 << 6);

			/* Servo #10 = PORTD5 */
			if (i < consignes[9])
				PORTD |= 1 << 5;
			else
				PORTD &= ~(1 << 5);

			/* Servo #11 = PORTD4 */
			if (i < consignes[10])
				PORTD |= 1 << 4;
			else
				PORTD &= ~(1 << 4);

			/* Servo #12 = PORTD3 */
			if (i < consignes[11])
				PORTD |= 1 << 3;
			else
				PORTD &= ~(1 << 3);
//			if (i < consignes[11])
//				PORTB |= 1 << 5;
//			else
//				PORTB &= ~(1 << 5);

			_delay_us(16); // RESO_US - 9
		}
		sei();

		PORTC &= ~(1 << 0);
		PORTC &= ~(1 << 1);
		PORTB &= ~(1 << 4);
		PORTB &= ~(1 << 3);
		PORTB &= ~(1 << 2);
		PORTB &= ~(1 << 1);
		PORTB &= ~(1 << 0);
		PORTD &= ~(1 << 7);
		PORTD &= ~(1 << 6);
		PORTD &= ~(1 << 5);
		PORTD &= ~(1 << 4);
		PORTD &= ~(1 << 3);
//		PORTB &= ~(1 << 5);
#endif /* defined(CONFIG_HAS_TIMER) */

//		if (calib_button_pressed())
//			PORTB |= 1 << 5;
//		else
//			PORTB &= ~(1 << 5);

#if defined(CONFIG_LOOPBACK)
		if (!--j) {
			j = 5;//25;
			
//			if (servo_consigne_us >= 500)
//				servo_consigne_us -= 25;
//			else
//				servo_consigne_us = 2500;
			if (servo_consigne_us <= 2500)
				servo_consigne_us += 25;
			else
				servo_consigne_us = 500;
		}
#endif /* defined(CONFIG_LOOPBACK) */

		/* Finalize the 20ms period (50Hz) of servos' signals */
		_delay_us(20000 - 2500);

		/* VServo is delayed after pulse generation */
		if (vservo_delay == 0)
			vservo_output_enable();
		else {
			vservo_output_disable();
			vservo_delay--;
		}
	}
}
