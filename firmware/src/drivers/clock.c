#include "drivers/clock.h"

#include <avr/interrupt.h>
#include <avr/common.h>
#include <avr/io.h>

// - - - macros - - - //

// some timing constants
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

// the prescaler is set so that the timer ticks every 64 clock cycles, and the the overflow handler is called every 256 ticks
#define MICROSECONDS_PER_TIMER_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER_OVERFLOW / 1000)

// the fractional number of milliseconds per timer overflow; we shift right by three to fit these numbers into a byte
#define FRACT_INC ((MICROSECONDS_PER_TIMER_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

// - - - static vars - - - //

// used to track overflow in timers since timers will run for certain fractions of desired times
volatile uint32_t timer_overflow_count;
volatile uint32_t timer_millis;
volatile uint32_t timer_fract;

// - - - implementations - - - //

void clockInit()
{
	// interrupts must be enabled for timers to work correctly
	sei();
	
	// set up clock hardware timer
	clockResume();

	// reset timer vars
	timer_overflow_count = 0;
	timer_millis = 0;
	timer_fract = 0;
}

uint32_t clockMillis()
{
	uint32_t m;
	uint8_t oldSREG = SREG;

	// disable interrupts and read our timer value
	cli();
	m = timer_millis;
	SREG = oldSREG;

	// return millisecond reading
	return m;
}

uint32_t clockMicros()
{
	uint32_t m;
	uint8_t t;
	uint8_t oldSREG = SREG;

	// disable interrupts and read our timer value
	cli();
	m = timer_overflow_count;
	t = TCNT1;
	if ((TIFR1 & _BV(TOV1)) && (t < 255))
		m++;

	// restore interrupts and return our reading
	SREG = oldSREG;
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void clockPause()
{
	TIMSK1 &= ~(1 << TOIE1);
	TCCR1A = 0;
	TCCR1B = 0;
}

void clockResume()
{
	// use fast 8-bit hardware PWM
	TCCR1A |= (1 << WGM10);
	TCCR1B |= (1 << WGM12);

	// set timer prescaler to 64
	TCCR1B |= (1 << CS11);
	TCCR1B |= (1 << CS10);

	// enable timer overflow interrupt
	TIMSK1 |= (1 << TOIE1);
}

// called when our millisecond-tracking timer 0 overflows
ISR(TIMER1_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint32_t m = timer_millis;
	uint32_t f = timer_fract;

	// apply timer increment and track overflows
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX)
	{
		f -= FRACT_MAX;
		m += 1;
	}

	// compute remaining fraction and milliseconds elapsed
	timer_fract = f;
	timer_millis = m;
	timer_overflow_count ++;
}
