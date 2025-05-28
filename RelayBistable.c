/*
 * RelayBistable.c
 *
 * Created: 26.03.2025 10:46:42
 *  Author: Vadim Kulakov, vad7@yahoo.com
 * 
 * ATTiny13A
 *
 * Connections:
 * pin 6 (PB1) <- IN
 * pin 7 (PB2) -> Bistable relay coil
 * pin 2 (PB3) -> Bistable relay coil
 * pin 3 (PB4) -> Optional LED for relay status
 * pin 1 (PB5) -> Optional info LED, relay power status, and pulse 0.5 sec on module power on. (MAX 1mA)
 *
 * Fuses(0=set): BODLEVEL = 2.7V (BODLEVEL[1:0]=01), RSTDISBL=0, CKSEL[1:0]=10, CKDIV8=0
 *
 */ 
#define F_CPU 1200000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define INPUT				(1<<PORTB1)
#define INPUT_IN			PINB
#define INPUT_IS_ACTIVE		((flags & (1<<bActiveLevel)) == ((INPUT_IN & INPUT) != 0))

#define RELAY_C1			(1<<PORTB2)
#define RELAY_C2			(1<<PORTB3)
#define RELAY_RESET			{ PORTB &= ~RELAY_C2; PORTB |= RELAY_C1; }	// 0
#define RELAY_SET			{ PORTB &= ~RELAY_C1; PORTB |= RELAY_C2; }	// 1
#define RELAY_STANDBY		{ PORTB &= ~RELAY_C1; PORTB &= ~RELAY_C2; }	// -
#define SETUP_RELAY			{ DDRB |= RELAY_C1 | RELAY_C2; }

#define LED1_PORT			PORTB
#define LED1				(1<<PORTB4)
#define LED1_ON				LED1_PORT |= LED1
#define LED1_OFF			LED1_PORT &= ~LED1
#define SETUP_LEDS			{ LED1_OFF; DDRB |= LED1; }

#define INFO_PORT			PORTB
#define INFO				(1<<PORTB5)
#define INFO_ON				LED1_PORT |= INFO
#define INFO_OFF			LED1_PORT &= ~INFO
#define SETUP_INFO			{ INFO_OFF; DDRB |= INFO; }

#define SETUP_PINS			{ SETUP_LEDS; SETUP_INFO; SETUP_RELAY; }
#define SETUP_UNUSED_PINS	{ PORTB |= (1<<PORTB0); }	// Pullup

#define SETUP_WATCHDOG { WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0); } // Watchdog 0.5s

struct _EEPROM {
	uint8_t		Action;				// 0 - direct, 1 - switch by pulse
	uint8_t		ActiveLevel;		// 0 - Low, >0 - High
	uint8_t		ResetRelayAtStartup;// 1 - Reset relay at power on 
	uint8_t		PauseAfterSwitch;	// *0.1 sec, input immunity after relay switching
	uint8_t		PulseTime;			// *0.1 sec, powering coil time
	uint8_t		OnTime;				// *sec, when non zero reset after specified time
	uint8_t		fDebounce;			// 0/1 - input debounce flag
	uint8_t		DebounceTime;		// 0.01 sec - input debounce time
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

#define bActiveLevel	0
#define bAction			1
#define bOn				2			// Relay output level, 0 - RESET, 1 - SET
#define bDebounce		3

register uint8_t flags asm("15");
uint8_t pause_counter = 0;
uint8_t pulse_counter = 0;
uint8_t ontime_counter = 0;
uint8_t timer_cnt = 0;
uint8_t need_switch = 0;			// 0 - no, 1 - switch, 2 - reset
uint8_t prev_level = 255;

void Delay100ms(uint8_t ms) {
	while(ms-- > 0) {
		_delay_ms(100);
		wdt_reset();
	}
}

void Delay10ms(uint8_t ms) {
	while(ms-- > 0) {
		_delay_ms(10);
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

ISR(INT0_vect)
{
	if(!pause_counter) {
		need_switch = 1;
	}
}

ISR(TIM0_OVF_vect)	// ~0.1s
{
	if(++timer_cnt == 10) { // 1 sec
		timer_cnt = 0;
		if(ontime_counter) if(--ontime_counter) {
			need_switch = 2;	// reset
		}
	}
	if(pause_counter) if(--pause_counter == 0) {
		if(!(flags & (1<<bAction)) && INPUT_IS_ACTIVE != ((flags & (1<<bOn)) != 0) && pulse_counter == 0) {
			need_switch = 1; // for reliability (direct only)
		}
	}
	if(pulse_counter) if(--pulse_counter == 0) { 
		INFO_OFF;
		RELAY_STANDBY
	}
}

void GetSettings(void)
{
	flags = (eeprom_read_byte(&EEPROM.Action) != 0) << bAction;
	flags |= (eeprom_read_byte(&EEPROM.ActiveLevel) != 0) << bActiveLevel;
	flags |= (eeprom_read_byte(&EEPROM.fDebounce) != 0) << bDebounce;
}

void ResetSettings(void)
{
	eeprom_update_byte(&EEPROM.Action, 0);			// 0 - direct, 1 - switch by pulse
	eeprom_update_byte(&EEPROM.ActiveLevel, 1);		// 0 - Low, >0 - High
	eeprom_update_byte(&EEPROM.PulseTime, 2);
	eeprom_update_byte(&EEPROM.PauseAfterSwitch, 5);
	eeprom_update_byte(&EEPROM.OnTime, 0);
	eeprom_update_byte(&EEPROM.ResetRelayAtStartup, 0);
	eeprom_update_byte(&EEPROM.fDebounce, 1);
	eeprom_update_byte(&EEPROM.DebounceTime, 15);	// *0.01s, 15 - Action = direct, 5 - Action = switch by pulse
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0); // Clock prescaler: 8
	SETUP_PINS
	SETUP_UNUSED_PINS
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0) | (0<<ISC01) | (1<<ISC00); // Sleep idle enable, Any logical change on INT0 generates an int
	PRR = (1<<PRADC);
	GIMSK |= (1<<INT0); // INT0: External Interrupt Request 0 Enable
	GIFR = (1<<INTF0);
	// Timer 8 bit
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00); // Timer0 prescaller: 1024
	TIMSK0 |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	OCR0A = 117; // OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out
	//ADMUX = (0<<REFS0) | (1<<ADLAR) | (1<<MUX1)|(1<<MUX0); // Voltage Reference VCC, ADC Left Adjust, in: ADC3 (PB3)
	//ADCSRA = (1<<ADEN) | (1<<ADSC ) | (1<<ADIE) | (1<<ADATE) | (1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); // ADC Interrupt Enable, ADC Start, ADC Auto Trigger Enable, ADC Prescaler: 64
	if(eeprom_read_byte(&EEPROM.Action) == 0xFF) ResetSettings();
	GetSettings();
	sei();
	INFO_ON;
	Delay100ms(5);
	INFO_OFF;
	uint8_t r = eeprom_read_byte(&EEPROM.ResetRelayAtStartup);
	if(flags & (1<<bAction)) {
		if(r) need_switch = 2;
	} else if(INPUT_IS_ACTIVE) need_switch = 1;
	else if(r) need_switch = 2;
    while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		wdt_reset();
		if(need_switch) {
			if(need_switch == 2) {	// reset
				RELAY_RESET;
				flags &= ~(1<<bOn);
			} else {
				if(flags & (1<<bDebounce)) {
					Delay10ms(eeprom_read_byte(&EEPROM.DebounceTime));
					if(INPUT_IS_ACTIVE == prev_level) goto xSkipSwitch;
					prev_level = INPUT_IS_ACTIVE;
				}
				if(flags & (1<<bAction)) { // switch
					if(flags & (1<<bOn)) {
						RELAY_RESET;
						flags &= ~(1<<bOn);
					} else {
						RELAY_SET;
						flags |= (1<<bOn);
					}
				} else {
					if(INPUT_IS_ACTIVE) {
						RELAY_SET;
						flags |= (1<<bOn);
					} else {
						RELAY_RESET;
						flags &= ~(1<<bOn);
					}
				}
			}
			TCNT0 = 0;	// reset timer
			INFO_ON;
			if(flags & (1<<bOn)) {
				LED1_ON; 
				ontime_counter = eeprom_read_byte(&EEPROM.OnTime);
			} else LED1_OFF;
			pulse_counter = eeprom_read_byte(&EEPROM.PulseTime);
			pause_counter = eeprom_read_byte(&EEPROM.PauseAfterSwitch);
xSkipSwitch:			
			need_switch = 0;
		}
		sleep_cpu();	// power down
	}
}
