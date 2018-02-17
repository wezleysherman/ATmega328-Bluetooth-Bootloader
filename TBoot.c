#include <avr/boot.h>
#include "chipdef.h"
#include <avr/io.h>
#include <util/setbaud.h>

#define _B1024
#define __AVR_ATmega168__
#define APP_PAGES ((2*8192 / SPM_PAGESIZE)- (2*1024 / SPM_PAGESIZE )) 
#define APP_END APP_PAGES * SPM_PAGESIZE 
#define SERIAL_BAUD_RATE 10
#ifndef BAUD                          /* if not defined in Makefile... */
#define BAUD  9600                     /* set a safe default baud rate */
#endif

                                  /* These are defined for convenience */
#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)
#define BL_VERSION_MAJOR '0'

#define BL_VERSION_MINOR '1'

void (*jumpToApp)(void) = 0x0000;
unsigned int bufferIndex = 0;
unsigned int pageIndex = 1;
unsigned char checksum;

#define serialReceiveReady()  (UCSR0A & (1<<RXC0))
#define serialDataRegisterReady()  (UCSR0A & (1<<UDRE0))
#define serialReceiveNow()  (UDR0)
#define serialTransmitMacro(byte) { while (!serialDataRegisterReady()); UDR0 = byte }

void initUSART(void) {                                /* requires BAUD */
  UBRR0H = UBRRH_VALUE;                        /* defined in setbaud.h */
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
                                  /* Enable USART transmitter/receiver */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (3 << UCSZ00);   /* 8 data bits, 1 stop bit */
}


unsigned char serialReceive(void)
{
  loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
  return UDR0; 
}


void serialTransmit(unsigned char byte)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = byte;  
}



unsigned char* getInputBuffer() {
	unsigned char inputBuffer [128];
	unsigned char input;
	do {
		input = serialReceive();
		inputBuffer[bufferIndex] = input;
		serialTransmit('*');
		serialReceive();
		bufferIndex ++;
	} while(input != '*');
	return inputBuffer;
}

unsigned char getHex(unsigned char* pageBuffer) {
	unsigned char data = pageBuffer[pageIndex];
	serialTransmit(data);
	if(data >= 'A') {
		data -= 'A' - 10;
	} else {
		data -= '0';
	}
	pageIndex++;

	return data;
}

unsigned char getBootData(unsigned char* pageBuffer) {
	unsigned char data;
	data = getHex(pageBuffer) << 4;
	data |= getHex(pageBuffer);
	checksum += data;
	return data;
}

int main(void) {
	initUSART();
	serialTransmit('*');
	top:
	while(1) {
	    unsigned char in = serialReceive();
	    serialTransmit('*');
	    serialReceive();
	    if(in == 'u') {
	    	unsigned char memory[SPM_PAGESIZE];
	    	unsigned int address;
	    	unsigned char count, type, i;
	    	unsigned int mem_data;
	    	type = 0;
	    	address = 0; 
	    	// erase app partition pages
	    	while(address < APP_END) {
	    		boot_page_erase(address);
	    		boot_spm_busy_wait();
	    		address += SPM_PAGESIZE;
	    	}
	    	do {
	    		unsigned char* pageBuffer = getInputBuffer();
	    		
	    		if(pageBuffer[0] != ':') {
	    			serialTransmit('E');
	    			goto top;
	    		}
	    		
	    		checksum = 0;
	    		count = getBootData(pageBuffer);
	    		address = getBootData(pageBuffer);
	    		address <<= 8;
	    		address |= getBootData(pageBuffer);
	    		type = getBootData(pageBuffer);
	    		for(i = 0; i < count; i++) {
	    			memory[i] = getBootData(pageBuffer);
	    		}

	    		getBootData(pageBuffer);
	    		if(checksum) {
	    			serialTransmit('C');
	    			goto top;
	    		}

	    		if(type == 0) {
	    			while(count < SPM_PAGESIZE)
	    				memory[count++] = 0xFF; // Fill the buffer
	    			for(i = 0; i < SPM_PAGESIZE;) {
	    				mem_data = memory[i++];
	    				mem_data |= (memory[i++] << 8);
	    				boot_page_fill(address, mem_data);
	    				address += 2;
	    			}
	    			boot_page_write(address-SPM_PAGESIZE);
	    			boot_spm_busy_wait();
	    			boot_rww_enable();
	    			bufferIndex = 0;
					pageIndex = 1;
					memset(pageBuffer, 0 , strlen(pageBuffer));
	    			serialTransmit('S');
	    		} else {
	    			serialTransmit('D');
	    		}

	    	} while(type != 1);

	    	if(type == 1) {
	    		serialTransmit('L');
	    		jumpToApp();
	    		serialTransmit('M');
	    	}
	    } else if(in == 'x') {
	    	serialTransmit('P');
	    	jumpToApp();
	    }
	}

  return 0;
}