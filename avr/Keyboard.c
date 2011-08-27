/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)
  Copyright 2010  Denver Gingerich (denver [at] ossguy [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Keyboard demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "Keyboard.h"
#include <usart.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/** Indicates what report mode the host has requested, true for normal HID reporting mode, false for special boot
 *  protocol reporting mode.
 */
static bool UsingReportProtocol = true;

/** Current Idle period. This is set by the host via a Set Idle HID class request to silence the device's reports
 *  for either the entire idle duration, or until the report status changes (e.g. the user presses a key).
 */
static uint16_t IdleCount = 500;

/** Current Idle period remaining. When the IdleCount value is set, this tracks the remaining number of idle
 *  milliseconds. This is separate to the IdleCount timer and is incremented and compared as the host may request
 *  the current idle period via a Get Idle HID class request, thus its value must be preserved.
 */
static uint16_t IdleMSRemaining = 0;


#define RELEASE (1<<0)
#define EXTENDED (1<<1)
#define CHANGED (1<<2)
static volatile uint8_t state;
static uint8_t pressed[64];

static const uint8_t usbCodes[] PROGMEM = {
	// --------------------------------------------- Base ------------------------------------------
	// 0x00 - 0x0F
	0x00,
	0x42, // F9
	0x00,
	0x3E, // F5

	0x3C, // F3
	0x3A, // F1
	0x3B, // F2
	0x45, // F12

	0x00,
	0x43, // F10
	0x41, // F8
	0x3F, // F6

	0x3D, // F4
	0x2B, // Tab
	0x35, // `
	0x00,


	// 0x10 - 0x1F
	0x00,
	0x00,
	0x00,
	0x00,

	0x48, // Pause/Break
	0x14, // Q
	0x1E, // 1
	0x00,

	0x00,
	0x00,
	0x1D, // Z
	0x16, // S

	0x04, // A
	0x1A, // W
	0x1F, // 2
	0x00,


	// 0x20 - 0x2F
	0x00,
	0x06, // C
	0x1B, // X
	0x07, // D

	0x08, // E
	0x21, // 4
	0x20, // 3
	0x00,

	0x00,
	0x2C, // Space
	0x19, // V
	0x09, // F

	0x17, // T
	0x15, // R
	0x22, // 5
	0x00,


	// 0x30 - 0x3F
	0x00,
	0x11, // N
	0x05, // B
	0x0B, // H

	0x0A, // G
	0x1C, // Y
	0x23, // 6
	0x00,

	0x00,
	0x00,
	0x10, // M
	0x0D, // J

	0x18, // U
	0x24, // 7
	0x25, // 8
	0x00,


	// 0x40 - 0x4F
	0x00,
	0x36, // ,
	0x0E, // K
	0x0C, // I

	0x12, // O
	0x27, // 0
	0x26, // 9
	0x00,

	0x00,
	0x37, // .
	0x38, // /
	0x0F, // L

	0x33, // ;
	0x13, // P
	0x2D, // -
	0x00,


	// 0x50 - 0x5F
	0x00,
	0x00,
	0x34, // '
	0x00,

	0x2F, // [
	0x2E, // =
	0x00,
	0x00,

	0x39, // Caps Lock
	0x00,
	0x28, // Return
	0x30, // ]

	0x00,
	0x32, // #
	0x00,
	0x00,


	// 0x60 - 0x6F
	0x00,
	0x64, // Backslash
	0x00,
	0x00,

	0x00,
	0x00,
	0x2A, // Backspace
	0x00,

	0x00,
	0x59, // Keypad "1"
	0x00,
	0x5C, // Keypad "4"

	0x5F, // Keypad "7"
	0x00,
	0x00,
	0x00,


	// 0x70 - 0x7F
	0x62, // Keypad "0/Ins"
	0x63, // Keypad "."
	0x5A, // Keypad "2"
	0x5D, // Keypad "5"

	0x5E, // Keypad "6"
	0x60, // Keypad "8"
	0x29, // Escape
	0x53, // Num Lock

	0x44, // F11
	0x57, // Keypad "+"
	0x5B, // Keypad "3"
	0x56, // Keypad "-"

	0x55, // Keypad "*"
	0x61, // Keypad "9"
	0x47, // Scroll Lock
	0x00,


	// 0x80 - 0x8F
	0x00,
	0x00,
	0x00,
	0x40, // F7

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x90 - 0x9F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xA0 - 0xAF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xB0 - 0xBF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xC0 - 0xCF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xD0 - 0xDF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xE0 - 0xEF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xF0 - 0xFF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// ------------------------------------------- Extended ----------------------------------------
	// 0x00 - 0x0F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x10 - 0x1F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x20 - 0x2F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x30 - 0x3F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x40 - 0x4F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x54, // Keypad "/"
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x50 - 0x5F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x58, // Keypad Enter
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x60 - 0x6F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x4D, // End
	0x00,
	0x50, // Left Arrow

	0x4A, // Home
	0x00,
	0x00,
	0x00,


	// 0x70 - 0x7F
	0x49, // Insert
	0x4C, // Delete
	0x51, // Down Arrow
	0x00,

	0x4F, // Right Arrow
	0x52, // Up Arrow
	0x00,
	0x00,

	0x00,
	0x00,
	0x4E, // Page Down
	0x00,

	0x46, // Print Screen
	0x4B, // Page Up
	0x00,
	0x00,


	// 0x80 - 0x8F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0x90 - 0x9F
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xA0 - 0xAF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xB0 - 0xBF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xC0 - 0xCF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xD0 - 0xDF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xE0 - 0xEF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,


	// 0xF0 - 0xFF
	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00,

	0x00,
	0x00,
	0x00,
	0x00
};

uint8_t translateCode(uint16_t scanCode) {
	return pgm_read_byte(&usbCodes[scanCode]);
}

int main(void) {
	uint8_t i;

	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	USB_Init();
	EICRA = (1<<ISC01);   // interrupt on the falling edge
	EICRB = 0x00;
	EIMSK = (1<<INT0);
	DDRD = 0x00;  // Port D inputs
	PORTD = 0x00; // Fiddle DDRD for open collector outs
	usartInit(38400);
	usartSendFlashString(PSTR("NanduinoJTAG...\r"));
	SetupHardware();
	state = 0x00;
	for ( i = 0; i < 32; i++ ) {
		pressed[i] = 0x00;
	}

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

	for (;;)
	{
		HID_Task();
		USB_USBTask();
	}
}

void setPressed(uint8_t key, bool extended) {
	if ( extended ) {
		pressed[(key>>3) + 32] |= (1<<(key&7));
	} else {
		pressed[key>>3] |= (1<<(key&7));
	}
}

void setReleased(uint8_t key, bool extended) {
	if ( extended ) {
		pressed[(key>>3) + 32] &= ~(1<<(key&7));
	} else {
		pressed[key>>3] &= ~(1<<(key&7));
	}
}

bool isPressed(uint16_t key) {
	return pressed[key>>3] & (1<<(key&7));
}

void gotScanCode(uint8_t scanCode) {
	//uint16_t oldScanCode = 0x0000;
	if ( scanCode == 0xE0 ) {
		// This is an extended key code
		state |= EXTENDED;
	} else if ( scanCode == 0xF0 ) {
		// A key has been released; the next byte will tell us which one.
		state |= RELEASE;
	} else {
		// This code represents a keypress or release
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
			if ( (scanCode == 0x77) && isPressed(0x014) ) {
				// Pause/Break generates two scan-codes, 14 and 77. Unfortunately, 77 is the proper
				// code for Num Lock, so we should ignore any 77 events when 14 is pressed.
				state &= ~(EXTENDED|RELEASE);
				return;
			}
		}
		DDRD |= 0x01;  // Drive clk low to tell keyboard to wait
		state |= CHANGED;  // tell USB loop to send a report
		if ( state & RELEASE ) {
			// A key has been released
			if ( state & EXTENDED ) {
				// An extended key has been released
				setReleased(scanCode, true);
				usartSendByte('x');
			} else {
				// A regular key has been released
				setReleased(scanCode, false);
			}
			usartSendByteHex(scanCode);
			usartSendByte('R');
			usartSendByte('\r');
		} else {
			// A key has been pressed
			if ( state & EXTENDED ) {
				// An extended key has been pressed
				if ( !isPressed(scanCode + 256) ) {
					setPressed(scanCode, true);
					usartSendByte('x');
					usartSendByteHex(scanCode);
					usartSendByte('P');
					usartSendByte('\r');
				}
			} else {
				// A regular key has been pressed
				if ( !isPressed(scanCode) ) {
					setPressed(scanCode, false);
					usartSendByteHex(scanCode);
					usartSendByte('P');
					usartSendByte('\r');
				}
			}
		}
		state &= ~(EXTENDED|RELEASE);
	}
}

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
		if ( parity != (PIND & 0x02) ) {
			usartSendByte('#');
		}
	} else if ( clk == 10 ) {
		// Stop bit - print code
		gotScanCode(data);
		clk = 255;
	}
	clk++;
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);

	/* Default to report protocol on connect */
	UsingReportProtocol = true;
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the keyboard device endpoints.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(KEYBOARD_IN_EPNUM, EP_TYPE_INTERRUPT, ENDPOINT_DIR_IN,
	                                            KEYBOARD_EPSIZE, ENDPOINT_BANK_SINGLE);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(KEYBOARD_OUT_EPNUM, EP_TYPE_INTERRUPT, ENDPOINT_DIR_OUT,
	                                            KEYBOARD_EPSIZE, ENDPOINT_BANK_SINGLE);

	/* Turn on Start-of-Frame events for tracking HID report period exiry */
	USB_Device_EnableSOFEvents();

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_KeyboardReport_Data_t thisReport = {0,};
				Endpoint_ClearSETUP();
				Endpoint_Write_Control_Stream_LE(&thisReport, sizeof(thisReport));
				Endpoint_ClearOUT();
			}

			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Wait until the LED report has been sent by the host */
				while (!(Endpoint_IsOUTReceived()))
				{
					if (USB_DeviceState == DEVICE_STATE_Unattached)
					  return;
				}

				/* Read in the LED report from the host */
				uint8_t LEDStatus = Endpoint_Read_8();

				Endpoint_ClearOUT();
				Endpoint_ClearStatusStage();

				/* Process the incoming LED report */
				ProcessLEDReport(LEDStatus);
			}

			break;
		case HID_REQ_GetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current protocol flag to the host */
				Endpoint_Write_8(UsingReportProtocol);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
		case HID_REQ_SetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Set or clear the flag depending on what the host indicates that the current Protocol should be */
				UsingReportProtocol = (USB_ControlRequest.wValue != 0);
			}

			break;
		case HID_REQ_SetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Get idle period in MSB, IdleCount must be multiplied by 4 to get number of milliseconds */
				IdleCount = ((USB_ControlRequest.wValue & 0xFF00) >> 6);
			}

			break;
		case HID_REQ_GetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current idle duration to the host, must be divided by 4 before sent to host */
				Endpoint_Write_8(IdleCount >> 2);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
	}
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	/* One millisecond has elapsed, decrement the idle time remaining counter if it has not already elapsed */
	if (IdleMSRemaining)
	  IdleMSRemaining--;
}

/** Processes a received LED report, and updates the board LEDs states to match.
 *
 *  \param[in] LEDReport  LED status report from the host
 */
void ProcessLEDReport(const uint8_t LEDReport)
{
	uint8_t LEDMask = LEDS_LED2;

	if (LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	  LEDMask |= LEDS_LED1;

	if (LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	  LEDMask |= LEDS_LED3;

	if (LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
	  LEDMask |= LEDS_LED4;

	/* Set the status LEDs to the current Keyboard LED status */
	LEDs_SetAllLEDs(LEDMask);
}

void composeReport(USB_KeyboardReport_Data_t *report) {
	uint16_t i;
	uint8_t j = 0;
	uint8_t usbCode;
	report->Modifier =
		(isPressed(0x014) ? 0x01 : 0x00) |
		(isPressed(0x012) ? 0x02 : 0x00) |
		(isPressed(0x011) ? 0x04 : 0x00) |
		(isPressed(0x11F) ? 0x08 : 0x00) |
		(isPressed(0x114) ? 0x10 : 0x00) |
		(isPressed(0x059) ? 0x20 : 0x00) |
		(isPressed(0x111) ? 0x40 : 0x00) |
		(isPressed(0x127) ? 0x80 : 0x00);
	for ( i = 0; i < 512; i++ ) {
		if ( isPressed(i) ) {
			usbCode = translateCode(i);
			if ( usbCode ) {
				report->KeyCode[j++] = usbCode;
				if ( j == 6 ) {
					// No room for any more codes
					return;
				}
			}
		}
	}
}

/** Sends the next HID report to the host, via the keyboard data endpoint. */
void SendNextReport(void) {
	//static USB_KeyboardReport_Data_t prevReport = {0,};
	Endpoint_SelectEndpoint(KEYBOARD_IN_EPNUM);
	if ( Endpoint_IsReadWriteAllowed() ) {
		if ( state & CHANGED ) {
			USB_KeyboardReport_Data_t thisReport = {0,};
			state &= ~CHANGED;
			composeReport(&thisReport);
			Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
			Endpoint_ClearIN();
			DDRD &= 0xFE;  // Release clk: ready for more bytes from keyboard
		}
	}
}

/*		if ( (PIND & 0x80) == 0x00 ) {
			thisReport.KeyCode[i++] = HID_KEYBOARD_SC_F;
		}

		if ( memcmp(&prevReport, &thisReport, sizeof(USB_KeyboardReport_Data_t)) != 0 ) {
			prevReport = thisReport;
			if ( i == 1 ) {
				// Pressed
				thisReport.Modifier = 0x10;  // Press rctrl
				thisReport.KeyCode[0] = 0x00;
				Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				Endpoint_ClearIN();

				while ( !Endpoint_IsReadWriteAllowed() );
				thisReport.Modifier = 0x00;  // Release rctrl
				Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				Endpoint_ClearIN();

				while ( !Endpoint_IsReadWriteAllowed() );
				thisReport.Modifier = 0x05;  // Press lctrl
				thisReport.KeyCode[0] = 0x4f;  // Press lalt
				Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				Endpoint_ClearIN();

				//while ( !Endpoint_IsReadWriteAllowed() );
				//thisReport.KeyCode[2] = 0x4F;  // Press rarrow
				//Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				//Endpoint_ClearIN();
				usartSendByte('!');
			} else {
				// Released
				thisReport.Modifier = 0x00;  // Release rarrow
				thisReport.KeyCode[0] = 0x00;  // Release rarrow
				Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				Endpoint_ClearIN();

				//while ( !Endpoint_IsReadWriteAllowed() );
				//thisReport.KeyCode[0] = 0x00;  // Release lctrl
				//thisReport.KeyCode[1] = 0x00;  // Release lalt
				//Endpoint_Write_Stream_LE(&thisReport, sizeof(thisReport), NULL);
				//Endpoint_ClearIN();
				usartSendByte('.');
			}
		}
	}
}
*/

/** Reads the next LED status report from the host from the LED data endpoint, if one has been sent. */
void ReceiveNextReport(void)
{
	/* Select the Keyboard LED Report Endpoint */
	Endpoint_SelectEndpoint(KEYBOARD_OUT_EPNUM);

	/* Check if Keyboard LED Endpoint contains a packet */
	if (Endpoint_IsOUTReceived())
	{
		/* Check to see if the packet contains data */
		if (Endpoint_IsReadWriteAllowed())
		{
			/* Read in the LED report from the host */
			uint8_t LEDReport = Endpoint_Read_8();

			/* Process the read LED report from the host */
			ProcessLEDReport(LEDReport);
		}

		/* Handshake the OUT Endpoint - clear endpoint and ready for next report */
		Endpoint_ClearOUT();
	}
}

/** Function to manage HID report generation and transmission to the host, when in report mode. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Send the next keypress report to the host */
	SendNextReport();

	/* Process the LED report sent from the host */
	ReceiveNextReport();
}

