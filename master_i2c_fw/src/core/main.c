//#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#define SD21_I2C_ADDR	(0xC2 >> 1)
#define SD21_I2C_RADDR	((SD21_I2C_ADDR << 1) | 0x01)
#define SD21_I2C_WADDR	((SD21_I2C_ADDR << 1) | 0x00)


#define FOSC F_CPU


//#define OLDDRV
#ifdef OLDDRV
/* cf. Datasheet p.275 */
#define START		0x08
#define MT_SLA_ACK	0x18
#define MT_DATA_ACK	0x28

//#define TWI_FREQ	400000UL
#define TWI_FREQ	100000UL

static inline void _i2c_start()
{
	/* Send Start condition */
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	/* Wait for transmit */
	while (!(TWCR & (1<<TWINT)))
		;

	/* Check status */
	if ((TWSR & 0xF8) != START)
		;//ERROR();
}

static inline void _i2c_stop()
{
	/* Send Stop condition */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

static void send_i2c(uint8_t SLA_W, uint8_t DATA)
{
	_i2c_start();

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
	_i2c_stop();
}

#else ///////////////////////////////////////

#define I2C_READ 0x01
#define I2C_WRITE 0x00

//#define F_SCL 100000UL // SCL frequency
#define F_SCL 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void i2c_init(void)
{
	TWBR = (uint8_t)TWBR_val;
}

uint8_t i2c_start(uint8_t address)
{
	// reset TWI control register
	TWCR = 0;
	// transmit START condition 
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }
	
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	
	return 0;
}

uint8_t i2c_write(uint8_t data)
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); 
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address << 1 | I2C_WRITE)) return 1;
	
	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}
	
	i2c_stop();
	
	return 0;
}

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_READ)) return 1;
	
	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();
	
	i2c_stop();
	
	return 0;
}

uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start(devaddr | 0x00)) return 1;

	i2c_write(regaddr);

	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}

	i2c_stop();

	return 0;
}

uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start(devaddr)) return 1;

	i2c_write(regaddr);

	if (i2c_start(devaddr | 0x01)) return 1;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();

	i2c_stop();

	return 0;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

#endif

int main(void)
{

#if 0
#if defined(__AVR__)
/* analog to digital conversion */
PORTA.DIR = 0x00; /*!< PORTA as input pin */
PORTA.OUT = 0x00;
PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

/* Port B - Jtag disable (fuse bit required) */
MCU_MCUCR = MCU_JTAGD_bm; /* Fuse4 bit0 to set to 1 with flasher */

/* twi configuration pin */
PORTC.DIRSET = PIN1_bm; /*!< PC1 (SCL) as output pin */
/* usart configuration pin */
PORTC.DIRCLR = PIN2_bm; /*!< PC2 (RDX0) as input pin */
PORTC.DIRSET = PIN3_bm; /*!< PC3 (TXD0) as output pin */
#endif
#endif

	/* B5 as output */
	//PORTB.DIRSET = PIN5_bm;
	DDRB |= (1 << 5);

	/* usart configuration pin */
	//PORTC.DIRCLR = PIN2_bm; /*!< PC2 (RDX0) as input pin */
	//PORTC.DIRSET = PIN3_bm; /*!< PC3 (TXD0) as output pin */
	DDRD |= (1 << 1); /* PD1(TXD) as output */


//#define FOSC 1843200 // Clock Speed
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

#if 0
	        usart_setup(con->usart, con->speed);

		        if (!main_console) {
				                main_console = con;

						                stdout = &uart_fdstream;
								                stdin = &uart_fdstream;
										        }
}
#endif

	//Out 1
	//PORTB.OUT |= 1 << PIN5_bp;
	//PORTB |= 1 << 5;

	//Out 0
	//PORTB.OUT &= ~(1 << PIN5_bp);
	//PORTB &= ~(1 << 5);



	//SCL frequency = CPU Clock frequency / (16 + 2(TWBR) * PrescalerValue)
	// Prescaler bit 1, 0 of TWSR:
	// 00b : Prescaler = 1
	// 01b : Prescaler = 4
	// 10b : Prescaler = 16
	// 11b : Prescaler = 64


////	/* PC4 as output  (SDA0) */
//	DDRC |= (1 << 4);
////	/* PC5 as output  (SCL0) */
//	DDRC |= (1 << 5);

	_delay_ms(10);

#ifdef OLDDRV
	TWSR |= 0x00;
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

	/* PC4 as output (SDA0) & PC5 as output  (SCL0) */
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWEA);
#else
	i2c_init();
#endif

	for (;;) {
		/* Out 1 */
		//PORTB.OUT |= 1 << PIN5_bp;
		PORTB |= 1 << 5;
		UDR0 = '\n';

		_delay_ms(100);

		/* Out 0 */
		//PORTB.OUT &= ~(1 << PIN5_bp);
		PORTB &= ~(1 << 5);

		//_delay_ms(490);
		//printf("hello\n");
		UDR0 = 'x';
		UDR0 = '\r';

//		static uint16_t servo_consigne_us = 2500;
//		if (servo_consigne_us <= 2500)
//			servo_consigne_us += 25;
//		else
//			servo_consigne_us = 500;

		static uint16_t servo_consigne_us = 1000;
		if (servo_consigne_us <= 2000)
			servo_consigne_us += 25;
		else
			servo_consigne_us = 1000;

#ifdef OLDDRV
		{
		static uint8_t i = 0;
//		send_i2c(SD21_I2C_WADDR, i); // Reg. offset
		send_i2c(SD21_I2C_WADDR, i++); // Reg. value
//		send_i2c(SD21_I2C_WADDR, 0x55); // Reg. offset
//		send_i2c(SD21_I2C_WADDR, 0xaa); // Reg. value
		}
#else
		//uint8_t data[] = { 0x55, 0xaa };
		static uint8_t servo_id = 0;
		uint8_t data[] = { (servo_id * 3)+1, 0xaa, 0xbb };
		data[1] = (uint8_t)(servo_consigne_us & 0xff);
		data[2] = (uint8_t)((servo_consigne_us >> 8) & 0xff);
		i2c_transmit((SD21_I2C_WADDR) >> 1, data, 3);

		_delay_ms(20);
		continue;
		goto next;

		//data[0] = (servo_id * 3);
		//data[1] = (uint8_t)(0);
		//i2c_transmit((SD21_I2C_WADDR) >> 1, data, 2);
		//_delay_ms(20);


		data[0] = (servo_id * 3) +1;
		data[1] = (uint8_t)(servo_consigne_us & 0xff);
		i2c_transmit((SD21_I2C_WADDR) >> 1, data, 2);
		_delay_ms(20);

		data[0] = (servo_id * 3) +2;
		data[1] = (uint8_t)((servo_consigne_us >> 8) & 0xff);
		i2c_transmit((SD21_I2C_WADDR) >> 1, data, 2);
		
		_delay_ms(20);

next:
		servo_id++;
		if (servo_id > 20) servo_id = 0;

////		servo_id ++;
////		uint16_t servo_consigne_us2 = servo_consigne_us * 2;
////		if (servo_consigne_us2 > 2000) servo_consigne_us2 -= 1000;
////		data[0] = (servo_id * 3) + 1;
////		data[1] = (uint8_t)(servo_consigne_us2);
////		data[2] = (uint8_t)((servo_consigne_us2 >> 8) & 0xff);
////		i2c_transmit((SD21_I2C_WADDR) >> 1, data, 3);

//		i2c_start(SD21_I2C_WADDR);
//		i2c_write(0x55); // set pointer to X axis MSB
//		i2c_stop();
//
//		i2c_start(SD21_I2C_WADDR);
//		i2c_write(0xAA); // set pointer to X axis MSB
//		i2c_stop();
#endif
	}
}
