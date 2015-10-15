/**
 * \file
 *
 * \brief Getting Started Application.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage Getting Started Application
 *
 * \section Purpose
 *
 * The Getting Started example will help new users get familiar with Atmel's
 * SAM family of microcontrollers. This basic application shows the startup
 * sequence of a chip and how to use its core peripherals.
 *
 * \section Requirements
 *
 * This application has been tested on following boards:
 * - SAM D21/R21/L21/C21 Xplained Pro
 * - SAM D10 Xplained Mini
 *
 * \section Description
 *
 * The program demo how LED,button,delay,interrupt and timer/counter work .
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 38400 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# The LED(s) should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
 *     -- Getting Started Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Pressing and release button SW0 should make LED0 on and off
 *    blinking.
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_uart_serial.h"
#include "string.h"

#define STRING_EOL    "\r"
#define STRING_HEADER "-- ADC Test --\r\n" 

#ifdef __cplusplus
extern "C" {
#endif

static struct usart_module cdc_uart_module;
static struct tc_module tc_instance;

#define TC_COUNT_VALUE 55535

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/* Updates the board LED to the current button state. */
static void update_led_state(void)
{
	bool pin_state = port_pin_get_input_level(BUTTON_0_PIN);
	if (pin_state) {
		port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
	} else {
		port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
	}
}

/** Callback function for the EXTINT driver, called when an external interrupt
 *  detection occurs.
 */
static void extint_callback(void)
{
	update_led_state();
}

/** Configures and registers the External Interrupt callback function with the
 *  driver.
 */
static void configure_eic_callback(void)
{
	extint_register_callback(extint_callback,
			BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
}

/** Configures the External Interrupt Controller to detect changes in the board
 *  button state.
 */
static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_BOTH;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
}


/** TC Callback function.
 */
static void tc_callback_to_counter(
		struct tc_module *const module_inst)
{
	static uint32_t count = 0;
	count ++;
	if(count%800 == 0){
		printf("The output is triggered by TC counter\r\n");
	}

	tc_set_count_value(module_inst,TC_COUNT_VALUE);
}

/** Configures  TC function with the  driver.
 */
static void configure_tc(void)
{
	struct tc_config config_tc;

	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.counter_16_bit.value = TC_COUNT_VALUE;

	tc_init(&tc_instance, CONF_TC_INSTANCE, &config_tc);
	tc_enable(&tc_instance);
}

/** Registers TC callback function with the  driver.
 */
static void configure_tc_callbacks(void)
{
	tc_register_callback(
			&tc_instance,
			tc_callback_to_counter,
			TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
}

/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
*/


struct port_config pin_data;
struct port_config pin_clk;

#define Signal_H			true
#define Signal_L			false

#define PIN_I2C_CLK 		PIN_PA09
#define PIN_I2C_DATA 		PIN_PA08

#define SDA_IN()			pin_data.direction = PORT_PIN_DIR_INPUT; \
							port_pin_set_config(PIN_I2C_DATA, &pin_data);

#define SDA_OUT()			pin_data.direction = PORT_PIN_DIR_OUTPUT; \
							port_pin_set_config(PIN_I2C_DATA, &pin_data);
							

#define IIC_SCL(x)			port_pin_set_output_level(PIN_I2C_CLK, (x))
#define IIC_SDA(x)			port_pin_set_output_level(PIN_I2C_DATA, (x))
#define READ_SDA			port_pin_get_input_level(PIN_I2C_DATA)
	
static void IIC_Init(void)
{
	IIC_SCL(1);
	IIC_SDA(1);
	
	port_get_config_defaults(&pin_clk);
	pin_clk.direction = PORT_PIN_DIR_OUTPUT;	
	port_pin_set_config(PIN_I2C_CLK, &pin_clk);
	port_pin_set_output_level(PIN_I2C_CLK, 1);

	port_get_config_defaults(&pin_data);
	pin_data.direction = PORT_PIN_DIR_OUTPUT;	
	port_pin_set_config(PIN_I2C_DATA, &pin_clk);
	port_pin_set_output_level(PIN_I2C_DATA, 1);
	
}

static void IIC_Start(void)
{
	SDA_OUT();     
	IIC_SDA(1);
	IIC_SCL(1);
	delay_us(4);
	IIC_SDA(0);	
	delay_us(4);
	IIC_SCL(0);
}

static void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	delay_us(4);
	IIC_SCL(1);
	IIC_SDA(1);
	delay_us(4);
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	
	SDA_IN();
	IIC_SDA(1);
	delay_us(1);
	IIC_SCL(1);
	delay_us(1);
	
	while(READ_SDA)
	{
		ucErrTime++;
		
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	
	IIC_SCL(0);
	
	return 0;
}

static void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}

static void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}

static void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	
	SDA_OUT();
	IIC_SCL(0);
	
	for(t=0; t<8; t++)
	{
		IIC_SDA((txd&0x80)>>7);
		txd<<=1;
		delay_us(2);   
		IIC_SCL(1);
		delay_us(2);
		IIC_SCL(0);
		delay_us(2);
	}
}

 void IIC_Read_Byte(unsigned char ack,unsigned char *data)
 {
	 unsigned char i;
	 unsigned char receive=0;
	 
	 SDA_IN();
	 
	 for(i=0; i<8; i++)
	 {
		 IIC_SCL(0);
		 delay_us(2);
		 IIC_SCL(1);
		 
		 receive <<= 1;
		 
		 if(READ_SDA)receive++;
		 
		 delay_us(1);
	 }
	 
	 *data = receive;
	 
	 if (!ack)
		IIC_NAck();
	 else
		IIC_Ack();
	 
 }

//#define WaitTime					100

#define Address_ADE7953				0x38
#define Command_W					(Address_ADE7953 << 1)
#define Command_R					(Command_W + 1)

#define cmd_IRMSA   				0x021A
#define cmd_IRMSA_31A   			0x031A


#define cmd_VAR_NOLOAD				0x0204


#define RegBit_8					1
#define RegBit_16					2
#define RegBit_24					3
#define RegBit_32					4



/*
void Read_16bit_Register(unsigned char command_H, unsigned char command_L, unsigned char *RegisterData)
{
	unsigned char buffer[2];
	
	IIC_Start();  

	IIC_Send_Byte(Command_W);
	IIC_Wait_Ack();

	IIC_Send_Byte(command_H);
	IIC_Wait_Ack();

	IIC_Send_Byte(command_L);	
	IIC_Wait_Ack();

	IIC_Start();  
	IIC_Send_Byte(Command_R);
	IIC_Wait_Ack();

	//delay_ms(WaitTime);

	IIC_Read_Byte(1,&buffer[0]);
	IIC_Read_Byte(0,&buffer[1]);
	IIC_Stop();
	
	*RegisterData = buffer[0];
	RegisterData++;
	*RegisterData = buffer[1];
}
*/
void Read_Register(unsigned char command_H, unsigned char command_L, 
						unsigned char *RegisterData,unsigned char x_Bit)
{
	unsigned char buffer[4];
	unsigned char count;
		
	IIC_Start();  
	
	IIC_Send_Byte(Command_W);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(command_H);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(command_L);	
	IIC_Wait_Ack();
	
	IIC_Start();  
	IIC_Send_Byte(Command_R);
	IIC_Wait_Ack();
	
	//delay_ms(WaitTime);
	count = x_Bit;
	
	for(unsigned char i = 0; i < x_Bit; i++)
	{
		count--;
		
		if(count == 0)
			IIC_Read_Byte(0,&buffer[i]);
		else
			IIC_Read_Byte(1,&buffer[i]);
	}
	
	IIC_Stop();

	for(unsigned char i = 0; i < x_Bit; i++)
	{
		*RegisterData = buffer[i];
		RegisterData++;	
	}		
}

//#define SS      252                     //定?SS所??的GPIO接口??  
//#define SCLK    253                     //定?SCLK所??的GPIO接口??  
//#define MOSI    254                     //定?SCLK所??的GPIO接口??  
//#define MISO    255                     //定?MISO所??的GPIO接口??  
//#define OUTP    1                       //表示GPIO接口方向??出  
//#define INP	  0                       //表示GPIO接口方向??入  
     
#define SS 				PIN_PA05
#define SCLK 			PIN_PA07	 
#define MOSI 			PIN_PA06
#define MISO 			PIN_PA04

#define SS()			pin_data.direction = PORT_PIN_DIR_OUTPUT; \
						port_pin_set_config(PIN_PA05, &pin_data);

#define SCLK()			pin_data.direction = PORT_PIN_DIR_OUTPUT; \
						port_pin_set_config(PIN_PA07, &pin_data);


#define MOSI()			pin_data.direction = PORT_PIN_DIR_OUTPUT; \
						port_pin_set_config(PIN_PA06, &pin_data);

#define MOSO()			pin_data.direction = PORT_PIN_DIR_INPUT; \
						port_pin_set_config(PIN_PA04, &pin_data);
						
#define DRDY()			pin_data.direction = PORT_PIN_DIR_INPUT; \
						port_pin_set_config(PIN_PB09, &pin_data);	 

#define SPI_CLK(x)			port_pin_set_output_level(PIN_PA07, (x))
#define SPI_SS(x)			port_pin_set_output_level(PIN_PA05, (x))
#define SPI_MOSI(x)			port_pin_set_output_level(PIN_PA06, (x))
#define SPI_MOSO			port_pin_get_input_level(PIN_PA04)
#define ADS1256_DRDY		port_pin_get_input_level(PIN_PB09)	 

void spi_init()  
{  
	DRDY();
	SS();
	SCLK();
	MOSI();
	MOSO();
	SPI_CLK(0);
	SPI_MOSI(0);
    //set_gpio_direction(SS, OUTP);  
    //set_gpio_direction(SCLK, OUTP);  
    //set_gpio_direction(MOSI, OUTP);  
    //set_gpio_direction(MISO, INP);  
    //set_gpio_value(SCLK, 0);                    //CPOL=0 
    //set_gpio_value(MOSI, 0);
}  

void ss_enable(int enable)  
{  
    if (enable)  
		//set_gpio_value(SS, 0);                  //SS Low
		SPI_SS(0);  
    else  
		//set_gpio_value(SS, 1);                  //SS HI  
		SPI_SS(1);	
}  
          
void spi_write_byte(unsigned char b)  
{  
    int i;  
    for (i=7; i>=0; i--) 
	{  
		//set_gpio_value(SCLK, 0);
		SPI_CLK(0);  
		//set_gpio_value(MOSI, b&(1<<i));          
		SPI_MOSI(b&(1<<i));
		delay_us(10);                              
		//set_gpio_value(SCLK, 1);                // CPHA=1  
		SPI_CLK(1);
		delay_us(10);      
    }  
	SPI_CLK(0);  
}  
    
unsigned char spi_read_byte()  
{  
   int i;  
   unsigned char r = 0;  
   for (i=0; i<8; i++) 
   {  
       //set_gpio_value(SCLK, 0);  
       SPI_CLK(0);
	   delay_us(10); 
	   //set_gpio_value(SCLK, 1);                // CPHA=1  
	   SPI_CLK(1);
		//r = (r <<1) | get_gpio_value(MISO);     
		r = (r <<1) | SPI_MOSO; 
		delay_us(10); 
   }  
   SPI_CLK(0);
   return r;
}  

  
void spi_write (unsigned char* buf, int len)  
{  
    int i;  
    //spi_init();           
    //ss_enable(1);			
    //delay_us(10); 
	//for (i=0; i<len; i++)  
	//	spi_write_byte(buf[i]);  
    //delay_us(10);  
    //ss_enable(0);
	
	for (i=0; i<len; i++)
		spi_write_byte(buf[i]);		 
}  

void spi_read(unsigned char* buf, int len)  
{  
    int i;  
   // spi_init();                    
   // ss_enable(1);                 
   // delay_us(10);  
   // for (i=0; i<len; i++)  
   //	buf[i] = spi_read_byte();  
   // delay_us(10);  
   // ss_enable(0);
   
    for (i=0; i<len; i++)
   		buf[i] = spi_read_byte();                  
}


int main(void)
{
	struct port_config pin;
	unsigned int count;
	unsigned char Write_Reg_DRATE[3]={0x53,0x01,0xc0};
	unsigned char Read_Reg_DRATE[2]={0x11,0x02};
	unsigned char read_buffer[10]={0};
	unsigned char read_data;
	unsigned int result;
	unsigned char Init_Data[5]={0x51,0x02,0x08,0x20,0xA1};
	unsigned char RDATAC[1]={0x03};
	unsigned char SDATAC[1]={0x0F};
	unsigned char RDATA[1] ={0x01};
	system_init();

	/*Configure UART console.*/
	configure_console();

	/*Initialize the delay driver*/
	delay_init();

	/* Output example information */
	puts(STRING_HEADER);

	/*Enable system interrupt*/
	system_interrupt_enable_global();


	//SPI0TX = 0x51;  // ADS1256_CMD_WREG | ADS1256_REG_MUX;
	//SPI0TX = 0x02;	// write 3 registers starting REG_MUX
	//SPI0TX = 0x08;	// REG_MUX
	//SPI0TX = 0x20;	// REG_ADCON
	//SPI0TX = 0xA1;	// REG_DRATE, set to 1ksps
	
	spi_init();
		
	ss_enable(1);
	delay_us(10);
	spi_write(Init_Data,5);
	delay_us(10);
	ss_enable(0);
	
	ss_enable(1);
	delay_us(10);
	spi_write(Read_Reg_DRATE,2);
	spi_read(read_buffer,3);
	delay_us(10);
	ss_enable(0);
	delay_us(10);
	
	count = 1;
	
	while(1)
	{
		result = 0;
		
		ss_enable(1);	
		delay_us(2);	
		//spi_write(RDATAC,1);
		spi_write(RDATA,1);
		while(ADS1256_DRDY);
		//while(1){
			//while(!ADS1256_DRDY);
			//if(ADS1256_DRDY == 0){
				spi_read(read_buffer,3);
			//	break;
			//}
		//}	
		//spi_write(SDATAC,1);
		delay_us(2);
		ss_enable(0);
		result = read_buffer[0] << 16 | read_buffer[1] << 8 | read_buffer[2];
		printf("#%d,0x%x\r\n",count++,result );	
		delay_ms(100);
	}
	
}

#ifdef __cplusplus
}
#endif
