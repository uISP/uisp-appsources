
/* Name: main.c
 * Project: AVR USB driver for CDC-SPI interface on Low-Speed USB
 *              for ATtiny2313
 * Author: Osamu Tamura
 * Creation Date: 2010-01-10
 * Tabsize: 4
 * Copyright: (c) 2010 by Recursion Co., Ltd.
 * License: Proprietary, free under certain conditions. See Documentation.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"


#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define SPI_PIN     PINB
#define SPI_SS      0 		// b1:DTS, b0:Break
#define SPI_SPEED   4		// 1:1MHz, 0:125kHz
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCK     7

#define HW_CDC_BULK_OUT_SIZE     8
#define HW_CDC_BULK_IN_SIZE      8


enum {
    SEND_ENCAPSULATED_COMMAND = 0,
    GET_ENCAPSULATED_RESPONSE,
    SET_COMM_FEATURE,
    GET_COMM_FEATURE,
    CLEAR_COMM_FEATURE,
    SET_LINE_CODING = 0x20,
    GET_LINE_CODING,
    SET_CONTROL_LINE_STATE,
    SEND_BREAK
};


static PROGMEM char configDescrCDC[] = {   /* USB configuration descriptor */
    9,          /* sizeof(usbDescrConfig): length of descriptor in bytes */
    USBDESCR_CONFIG,    /* descriptor type */
    67,
    0,          /* total length of data returned (including inlined descriptors) */
    2,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
    (1 << 7) | USBATTR_SELFPOWER,       /* attributes */
#else
    (1 << 7),                           /* attributes */
#endif
    USB_CFG_MAX_BUS_POWER/2,            /* max USB current in 2mA units */

    /* interface descriptor follows inline: */
    9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE, /* descriptor type */
    0,          /* index of this interface */
    0,          /* alternate setting for this interface */
    USB_CFG_HAVE_INTRIN_ENDPOINT,   /* endpoints excl 0: number of endpoint descriptors to follow */
    USB_CFG_INTERFACE_CLASS,
    USB_CFG_INTERFACE_SUBCLASS,
    USB_CFG_INTERFACE_PROTOCOL,
    0,          /* string index for interface */

    /* CDC Class-Specific descriptor */
    5,           /* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
    0x24,        /* descriptor type */
    0,           /* header functional descriptor */
    0x10, 0x01,

    4,           /* sizeof(usbDescrCDC_AcmFn): length of descriptor in bytes */
    0x24,        /* descriptor type */
    2,           /* abstract control management functional descriptor */
    0x06,        /* SEND_BREAK,SET_LINE_CODING,GET_LINE_CODING,SET_CONTROL_LINE_STATE    */

    5,           /* sizeof(usbDescrCDC_UnionFn): length of descriptor in bytes */
    0x24,        /* descriptor type */
    6,           /* union functional descriptor */
    0,           /* CDC_COMM_INTF_ID */
    1,           /* CDC_DATA_INTF_ID */

    5,           /* sizeof(usbDescrCDC_CallMgtFn): length of descriptor in bytes */
    0x24,        /* descriptor type */
    1,           /* call management functional descriptor */
    3,           /* allow management on data interface, handles call management by itself */
    1,           /* CDC_DATA_INTF_ID */

    /* Endpoint Descriptor */
    7,           /* sizeof(usbDescrEndpoint) */
    USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
    0x83,        /* IN endpoint number 3 */
    0x03,        /* attrib: Interrupt endpoint */
    8, 0,        /* maximum packet size */
    USB_CFG_INTR_POLL_INTERVAL,        /* in ms */

    /* Interface Descriptor  */
    9,           /* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE,           /* descriptor type */
    1,           /* index of this interface */
    0,           /* alternate setting for this interface */
    2,           /* endpoints excl 0: number of endpoint descriptors to follow */
    0x0A,        /* Data Interface Class Codes */
    0,
    0,           /* Data Interface Class Protocol Codes */
    0,           /* string index for interface */

    /* Endpoint Descriptor */
    7,           /* sizeof(usbDescrEndpoint) */
    USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
    0x01,        /* OUT endpoint number 1 */
    0x02,        /* attrib: Bulk endpoint */
    HW_CDC_BULK_OUT_SIZE, 0,        /* maximum packet size */
    0,           /* in ms */

    /* Endpoint Descriptor */
    7,           /* sizeof(usbDescrEndpoint) */
    USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
    0x81,        /* IN endpoint number 1 */
    0x02,        /* attrib: Bulk endpoint */
    HW_CDC_BULK_IN_SIZE, 0,        /* maximum packet size */
    0,           /* in ms */
};


uchar usbFunctionDescriptor(usbRequest_t *rq)
{

    if(rq->wValue.bytes[1] == USBDESCR_DEVICE){
        usbMsgPtr = (uchar *)usbDescriptorDevice;
        return usbDescriptorDevice[0];
    }else{  /* must be config descriptor */
        usbMsgPtr = (uchar *)configDescrCDC;
        return sizeof(configDescrCDC);
    }
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar    sendEmptyFrame;

uchar usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
		uchar	value;

        /*  GET_LINE_CODING -> usbFunctionRead()    */
        if( rq->bRequest==GET_LINE_CODING )
            return 0xff;
        
		value	= rq->wValue.word;
        /*  DTR => SPI_SS+1	*/
        if( rq->bRequest==SET_CONTROL_LINE_STATE )
            SPI_PORT    = (SPI_PORT&~(2<<SPI_SS))|((value&1)<<(SPI_SS+1));

        /*  Break => SPI_SS	*/
        else if( rq->bRequest==SEND_BREAK )
            SPI_PORT    = (SPI_PORT&~(1<<SPI_SS))|((value&1)<<SPI_SS);
    }

    return 0;
}


/*---------------------------------------------------------------------------*/
/* usbFunctionRead                                                           */
/*---------------------------------------------------------------------------*/

uchar usbFunctionRead( uchar *data, uchar len )
{
    /*  reply USART configuration    */
#define    BAUD    9600
    data[0]    = BAUD & 0xff;
    data[1]    = (uchar)(BAUD>>8);
    data[2]    = 0;
    data[3]    = 0;
    data[4]    = 0;
    data[5]    = 0;
    data[6]    = 8;

    return 7;
}


/*  spi buffer */
static uchar    rx_buf[HW_CDC_BULK_IN_SIZE];
static uchar    tx_buf[HW_CDC_BULK_OUT_SIZE];
static uchar    iwptr, uwptr;


void usbFunctionWriteOut( uchar *data, uchar len )
{

    /*  host -> spi     */
    while( len-- )
        tx_buf[uwptr++] = *data++;

    /*  postpone receiving next data    */
   	usbDisableAllRequests();
}


static void hardwareInit(void)
{
uchar	i;

    /* activate pull-ups except on USB lines */
    USB_CFG_IOPORT   = (uchar)~((1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT));
    /* all pins input except USB (-> USB reset) */
#ifdef USB_CFG_PULLUP_IOPORT    /* use usbDeviceConnect()/usbDeviceDisconnect() if available */
    USBDDR    = 0;    /* we do RESET by deactivating pullup */
    usbDeviceDisconnect();
#else
    USBDDR    = (1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT);
#endif

    /* keep 300 mS  */
	for( i=150; --i; ) {
		wdt_reset();
		_delay_ms(2.0);
	}

#ifdef USB_CFG_PULLUP_IOPORT
    usbDeviceConnect();
#else
    USBDDR  = 0;      /*  remove USB reset condition */
#endif
}


int main(void)
{
    odDebugInit();
    hardwareInit();
    usbInit();

    /*  set SCL, DO, and SS as output  */
    SPI_DDR |= (1<<SPI_SCK)|(1<<SPI_MISO)|(3<<SPI_SS);
    SPI_PORT    = (uchar)~((1<<SPI_SCK)|(1<<SPI_MISO));

    /*  SPI mode    */
    USICR   = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK);

    sei();
    for(;;){    /* main event loop */
		wdt_reset();
        usbPoll();

        /*  host => device  */
        if( uwptr!=0 && iwptr==0 ) {
           	usbEnableAllRequests();

	        while( iwptr<uwptr ) {
	            USIDR   = tx_buf[iwptr];
	            USISR   = (1<<USIOIF);

				if( SPI_PIN&(1<<SPI_SPEED) ) {
					do {
						//	clk=1MHz
						wdt_reset();
		    			USICR   |= (1<<USITC);
		            } while( !(USISR&(1<<USIOIF)) );
				}
				else {
					do {
						//	clk=125kHz
						_delay_us(3.5);
		    			USICR   |= (1<<USITC);
		            } while( !(USISR&(1<<USIOIF)) );
				}
	            rx_buf[iwptr++] = USIDR;
			}
			uwptr	= 0;
        }

        /*  host <= device  */
        if( usbInterruptIsReady() && (iwptr|sendEmptyFrame) ) {
            usbSetInterrupt(rx_buf, iwptr);
            sendEmptyFrame    = iwptr & HW_CDC_BULK_IN_SIZE;
            iwptr	= 0;
        }
    }
    return 0;
}

