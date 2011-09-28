/* 
 * Copyright (C) 2011 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <LUFA/Version.h>
#include <LUFA/Drivers/USB/USB.h>
//#include <usart.h>
#include "keybitmap.h"
#include "scancodes.h"
#include "desc.h"

// More readable typedef for USB keyboard report structure.
typedef USB_KeyboardReport_Data_t KeyboardReport;

// Function declarations
static void receiveNextReport(void);
static void sendNextReport(void);

// Indicates what report mode the host has requested, true for normal HID reporting mode, false for
// special boot protocol reporting mode.
static bool usingReportProtocol = true;

// Current Idle period. This is set by the host via a Set Idle HID class request to silence the
// device's reports for either the entire idle duration, or until the report status changes (e.g.
// the user presses a key).
static uint16_t idleCount = 500;

// Current Idle period remaining. When the idleCount value is set, this tracks the remaining number
// of idle milliseconds. This is separate to the idleCount timer and is incremented and compared as
// the host may request the current idle period via a Get Idle HID class request, thus its value
// must be preserved.
static uint16_t idleRemaining = 0;

// Maintain state between reception of bytes from the PS/2 keyboard.
#define RELEASE (1<<0)
#define EXTENDED (1<<1)
#define CHANGED (1<<2)
static volatile uint8_t state;

// Entry point: initialise hardware and interrupts, then start the main loop.
//
int main(void) {
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);

	USB_Init();
	EICRA = (1<<ISC01);   // Interrupt on the falling edge...
	EICRB = 0x00;
	EIMSK = (1<<INT0);    // ...of INT0 (PD0): the PS/2 clk
	DDRD = 0x00;          // Port D is all inputs
	PORTD = 0x00;         // Fiddle DDRD for open collector outs
	//usartInit(38400);
	//usartSendFlashString(PSTR("MakeStuff PS/2 to USB Adaptor...\r"));
	bmInit();
	state = 0x00;

	sei();
	for ( ; ; ) {
		if ( USB_DeviceState == DEVICE_STATE_Configured ) {
			sendNextReport();
			receiveNextReport();
		}
		USB_USBTask();
	}
}

// Process an incoming LED report from the host.
//
static void processLEDReport(const uint8_t ledReport) {
/*	uint8_t LEDMask = LEDS_LED2;

	if (LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	  LEDMask |= LEDS_LED1;

	if (LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	  LEDMask |= LEDS_LED3;

	if (LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
	  LEDMask |= LEDS_LED4;

	LEDs_SetAllLEDs(LEDMask);
*/
}

// Process the received PS/2 scancode.
//
static void processScanCode(uint8_t scanCode) {
	if ( scanCode == 0xE0 ) {
		// This is an extended key code
		state |= EXTENDED;
	} else if ( scanCode == 0xF0 ) {
		// A key has been released; the next byte will tell us which one.
		state |= RELEASE;
	} else {
		// This code represents a keypress or release. Check to see if it can be dropped.
		if ( state & EXTENDED ) {
			// Special cases in the extended range...
			if ( scanCode == 0x12 ) {
				// Print Screen generates two scan-codes, x12 and x7C. x12 appears to be some sort
				// of special code because the arrow keys generate it when Num Lock is off. I'm
				// choosing to ignore it.
				state &= ~(EXTENDED|RELEASE);
				return;
			}
		} else {
			// Special cases in the base range
			if ( (scanCode == 0x77) && bmIsPressed(0x014) ) {
				// Pause/Break generates two scan-codes, 14 and 77. Unfortunately, 77 is the proper
				// code for Num Lock, so we should ignore any 77 events when 14 is pressed.
				state &= ~(EXTENDED|RELEASE);
				return;
			}
		}

		// Update the bitmap so we can keep track of what's pressed and what's not
		if ( state & RELEASE ) {
			// A key has been released
			DDRD |= 0x01;  // Drive clk low to tell keyboard to wait
			state |= CHANGED; // Tell USB loop to send a report
			if ( state & EXTENDED ) {
				// An extended key has been released
				bmSetReleased(scanCode, true);
				//usartSendByte('x');
			} else {
				// A regular key has been released
				bmSetReleased(scanCode, false);
			}
			//usartSendByteHex(scanCode);
			//usartSendByte('R');
			//usartSendByte('\r');
		} else {
			// A key has been pressed
			if ( state & EXTENDED ) {
				// An extended key has been pressed
				if ( !bmIsPressed(scanCode + 256) ) {
					DDRD |= 0x01;  // Drive clk low to tell keyboard to wait
					state |= CHANGED; // Tell USB loop to send a report
					bmSetPressed(scanCode, true);
					//usartSendByte('x');
					//usartSendByteHex(scanCode);
					//usartSendByte('P');
					//usartSendByte('\r');
				}
			} else {
				// A regular key has been pressed
				if ( !bmIsPressed(scanCode) ) {
					DDRD |= 0x01;  // Drive clk low to tell keyboard to wait
					state |= CHANGED; // Tell USB loop to send a report
					bmSetPressed(scanCode, false);
					//usartSendByteHex(scanCode);
					//usartSendByte('P');
					//usartSendByte('\r');
				}
			}
		}

		// This was the last byte in this PS/2 event message, so clear the flags ready for the next
		state &= ~(EXTENDED|RELEASE);
	}
}

// Interrupt service routine called on the falling edge of the PS/2 clock
//
ISR(INT0_vect) {
	static unsigned char data;
	static uint8_t clk = 0;
	static uint8_t parity;
	if ( clk == 0 ) {
		// Start bit - do nothing
		parity = 0x02;
	} else if ( clk >= 1 && clk <= 8 ) {
		// Data bits - store
		data >>= 1;
		if ( PIND & 0x02 ) {
			parity ^= 0x02;
			data |= 0x80;
		}
	} else if ( clk == 9 ) {
		// Parity bit - check it
		//if ( parity != (PIND & 0x02) ) {
		//	usartSendByte('#');
		//}
	} else if ( clk == 10 ) {
		// Stop bit - process code
		processScanCode(data);
		clk = 255;
	}
	clk++;
}

// Called on connect (i.e device physically plugged in)
//
void EVENT_USB_Device_Connect(void) {
	usingReportProtocol = true;
}

// Called on disconnect (i.e device physically unplugged)
void EVENT_USB_Device_Disconnect(void) {
	// Do nothing
}

// Called when enumeration is complete and device is being configured by the OS
//
void EVENT_USB_Device_ConfigurationChanged(void) {
	bool success = true;

	// Setup HID Report Endpoints
	success &= Endpoint_ConfigureEndpoint(
		KEYBOARD_IN_EPNUM, EP_TYPE_INTERRUPT, ENDPOINT_DIR_IN,
		KEYBOARD_EPSIZE, ENDPOINT_BANK_SINGLE);
	success &= Endpoint_ConfigureEndpoint(
		KEYBOARD_OUT_EPNUM, EP_TYPE_INTERRUPT, ENDPOINT_DIR_OUT,
		KEYBOARD_EPSIZE, ENDPOINT_BANK_SINGLE);

	// Turn on Start-of-Frame events for tracking HID report period exiry
	USB_Device_EnableSOFEvents();
}

// Handle incoming messages on endpoint zero
//
void EVENT_USB_Device_ControlRequest(void) {
	// Handle HID Class specific requests
	switch ( USB_ControlRequest.bRequest ) {
	case HID_REQ_GetReport:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			KeyboardReport thisReport = {0,};
			Endpoint_ClearSETUP();
			Endpoint_Write_Control_Stream_LE(&thisReport, sizeof(thisReport));
			Endpoint_ClearOUT();
		}
		break;
		
	case HID_REQ_SetReport:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			uint8_t ledStatus;
			Endpoint_ClearSETUP();
			
			// Wait until the LED report has been sent by the host
			while ( !Endpoint_IsOUTReceived() ) {
				if ( USB_DeviceState == DEVICE_STATE_Unattached ) {
					return;
				}
			}
			
			// Read in the LED report from the host
			ledStatus = Endpoint_Read_8();
			
			Endpoint_ClearOUT();
			Endpoint_ClearStatusStage();
			
			// Process the incoming LED report
			processLEDReport(ledStatus);
		}
		break;
		
	case HID_REQ_GetProtocol:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			Endpoint_ClearSETUP();
			
			// Write the current protocol flag to the host
			Endpoint_Write_8(usingReportProtocol);
			
			Endpoint_ClearIN();
			Endpoint_ClearStatusStage();
		}
		break;
		
	case HID_REQ_SetProtocol:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			Endpoint_ClearSETUP();
			Endpoint_ClearStatusStage();
			
			// Set or clear the flag depending on what the host wants
			usingReportProtocol = (USB_ControlRequest.wValue != 0);
		}
		break;
		
	case HID_REQ_SetIdle:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			Endpoint_ClearSETUP();
			Endpoint_ClearStatusStage();
			
			// Get idle period in MSB, idleCount must be multiplied by 4 to get no. of milliseconds
			idleCount = ((USB_ControlRequest.wValue & 0xFF00) >> 6);
		}
		break;
		
	case HID_REQ_GetIdle:
		if ( USB_ControlRequest.bmRequestType ==
		     (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE) )
		{
			Endpoint_ClearSETUP();
			
			// Write the current idle duration to the host, must be divided by 4 before sent to host
			Endpoint_Write_8(idleCount >> 2);
			
			Endpoint_ClearIN();
			Endpoint_ClearStatusStage();
		}
		break;
	}
}

// Called every 1ms...
//
void EVENT_USB_Device_StartOfFrame(void) {
	// Decrement the idle time remaining counter if it has not already elapsed.
	if ( idleRemaining ) {
		idleRemaining--;
	}
}

// Compose a report to send to the host
//
static void handleRegularReport(void) {
	KeyboardReport report = {0,};
	uint16_t i;
	uint8_t j = 0;
	uint8_t usbCode;
	report.Modifier =
		(bmIsPressed(0x014) ? 0x01 : 0x00) |  // Left Ctrl
		(bmIsPressed(0x012) ? 0x02 : 0x00) |  // Left Shift
		(bmIsPressed(0x011) ? 0x04 : 0x00) |  // Left Alt
		(bmIsPressed(0x11F) ? 0x08 : 0x00) |  // Left Windows Key
		(bmIsPressed(0x114) ? 0x10 : 0x00) |  // Right Ctrl
		(bmIsPressed(0x059) ? 0x20 : 0x00) |  // Right Shift
		(bmIsPressed(0x111) ? 0x40 : 0x00) |  // Right Alt
		(bmIsPressed(0x127) ? 0x80 : 0x00);   // Right Windows Key
	for ( i = 0; i < 512; i++ ) {
		// TODO: It would be more efficient to check the byte values first, rather than checking
		//       each bit individually.
		if ( bmIsPressed(i) ) {
			usbCode = translateCode(i);
			if ( usbCode ) {
				report.KeyCode[j++] = usbCode;
				if ( j == 6 ) {
					// No room for any more codes
					return;
				}
			}
		}
	}
	Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
	Endpoint_ClearIN();
}

static const uint8_t specialKeys[] PROGMEM = {
	0x6b, // Left arrow
	0x01, // Extended range
	0x05, // Left ctrl & alt
	0x50, // Left arrow
	0x00,
	0x74, // Right arrow
	0x01, // Extended range
	0x05, // Left ctrl & alt
	0x4f, // Left arrow
	0x00,
	0x05, // F1
	0x00, // Base range
	0x00, // No modifier
	0x68, // F13
	0x00,
	0x06, // F2
	0x00, // Base range
	0x00, // No modifier
	0x69, // F14
	0x00,
	0x04, // F3
	0x00, // Base range
	0x00, // No modifier
	0x6a, // F15
	0x00,
	0x0c, // F4
	0x00, // Base range
	0x00, // No modifier
	0x6b, // F16
	0x00,
	0x03, // F5
	0x00, // Base range
	0x00, // No modifier
	0x6c, // F17
	0x00,
	0x0b, // F6
	0x00, // Base range
	0x00, // No modifier
	0x6d, // F18
	0x00,
	0x83, // F7
	0x00, // Base range
	0x00, // No modifier
	0x6e, // F19
	0x00,
	0x0a, // F8
	0x00, // Base range
	0x00, // No modifier
	0x6f, // F20
	0x00,
	0x01, // F9
	0x00, // Base range
	0x00, // No modifier
	0x70, // F21
	0x00,
	0x09, // F10
	0x00, // Base range
	0x00, // No modifier
	0x71, // F22
	0x00,
	0x78, // F11
	0x00, // Base range
	0x00, // No modifier
	0x72, // F23
	0x00,
	0x07, // F12
	0x00, // Base range
	0x00, // No modifier
	0x73, // F24
	0x00,
	0x00  // End of list
};

static void handleSpecialReport(void) {
	KeyboardReport report = {0,};
	const uint8_t *p = specialKeys;
	uint8_t byte = pgm_read_byte(p++);
	while ( byte ) {
		uint16_t scanCode = (pgm_read_byte(p++) << 8) + byte;
		//usartSendWordHex(scanCode);
		//usartSendByte('\r');
		if ( bmIsPressed(scanCode) ) {
			uint8_t i = 0;
			//usartSendByte('%');

			// Right-ctrl key down
			report.Modifier = 0x10;  // Press rctrl
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			// Right-ctrl key up
			while ( !Endpoint_IsReadWriteAllowed() );
			report.Modifier = 0x00;  // Release rctrl
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			// Key macro down
			while ( !Endpoint_IsReadWriteAllowed() );
			report.Modifier = pgm_read_byte(p++);
			byte = pgm_read_byte(p++);
			while ( byte ) {
				report.KeyCode[i++] = byte;
				byte = pgm_read_byte(p++);
			}
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			// Key macro up
			while ( !Endpoint_IsReadWriteAllowed() );
			report.Modifier = 0x00;
			for ( i = 0; i < 6; i++ ) {
				report.KeyCode[i] = 0x00;
			}
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			// Right-ctrl key down
			while ( !Endpoint_IsReadWriteAllowed() );
			report.Modifier = 0x10;  // Press rctrl
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			// Right-ctrl key up
			while ( !Endpoint_IsReadWriteAllowed() );
			report.Modifier = 0x00;  // Release rctrl
			Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
			Endpoint_ClearIN();

			return;  // Only send the first one found
		} else {
			p++;  // Skip over modifier
			do {
				byte = pgm_read_byte(p++);
			} while ( byte );
		}
		byte = pgm_read_byte(p++);
	}

	// If we got this far then we found nothing, so send a blank report. This is so when a special
	// key is released, the host is notified.
	Endpoint_Write_Stream_LE(&report, sizeof(report), NULL);
	Endpoint_ClearIN();
}

// Send a report to the host, if necessary
//
static void sendNextReport(void) {
	Endpoint_SelectEndpoint(KEYBOARD_IN_EPNUM);
	if ( Endpoint_IsReadWriteAllowed() ) {
		if ( state & CHANGED ) {
			state &= ~CHANGED;
			if ( bmIsPressed(0x12f) ) {
				handleSpecialReport();
			} else {
				handleRegularReport();
			}
			DDRD &= 0xFE;  // Release clk: ready for more bytes from keyboard
		}
	}
}

// Reads the next LED status report from the host from the LED data endpoint, if one has been sent.
static void receiveNextReport(void) {
	Endpoint_SelectEndpoint(KEYBOARD_OUT_EPNUM);

	// Check if Keyboard LED Endpoint contains a packet
	if ( Endpoint_IsOUTReceived() ) {
		// Check to see if the packet contains data
		if ( Endpoint_IsReadWriteAllowed() ) {
			// Read in the LED report from the host, and process it
			uint8_t ledReport = Endpoint_Read_8();
			processLEDReport(ledReport);
		}
		Endpoint_ClearOUT(); // Clear endpoint ready for next report
	}
}
