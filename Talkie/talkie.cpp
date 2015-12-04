// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

// Though the Wave Shield DAC pins are configurable, much else in this code
// is still very Uno-specific; the timers and timer control registers, the
// PWM output pin, etc.  Compatibility with other boards is not guaranteed.

#include "talkie.h"

//#define PIEZO         // If set, connect piezo on pins 3 & 11, is louder

#define FS    8000      // Speech engine sample rate
#define TICKS (FS / 40) // Speech data rate

// Some of these variables could go in the Talkie object, but the hardware
// specificity (reliance on certain timers and/or PWM pins) kills any point
// in multiple instances; there can be only one.  So they're declared as
// static here to keep the header simple and self-documenting.
#if TICKS < 255
static volatile uint8_t  interruptCount;
#else
static volatile uint16_t interruptCount;
#endif
static volatile uint8_t  *csPort, *clkPort, *datPort;
static volatile uint16_t synthEnergy;
static volatile int16_t  synthK1, synthK2;
static volatile int8_t   synthK3, synthK4, synthK5, synthK6,
                         synthK7, synthK8, synthK9, synthK10;
static uint16_t          buf, synthRand = 1;
static int16_t           x0, x1, x2, x3, x4, x5, x6, x7, x8, x9;
static uint8_t           periodCounter, nextPwm = 0x80,
                         synthPeriod, bufBits,
                         csBitMask, clkBitMask, datBitMask;
static const uint8_t     *ptrAddr;

static const int16_t PROGMEM
  tmsK1[]     = {-32064,-31872,-31808,-31680,-31552,-31424,-31232,-30848,
                 -30592,-30336,-30016,-29696,-29376,-28928,-28480,-27968,
                 -26368,-24256,-21632,-18368,-14528,-10048, -5184,     0,
                   5184, 10048, 14528, 18368, 21632, 24256, 26368, 27968},
  tmsK2[]     = {-20992,-19328,-17536,-15552,-13440,-11200, -8768, -6272,
                  -3712, -1088,  1536,  4160,  6720,  9216, 11584, 13824,
                  15936, 17856, 19648, 21248, 22656, 24000, 25152, 26176,
                  27072, 27840, 28544, 29120, 29632, 30080, 30464, 32384};

static const int8_t PROGMEM
  tmsK3[]     = {-110, -97, -83, -70, -56, -43, -29, -16,
                   -2,  11,  25,  38,  52,  65,  79,  92},
  tmsK4[]     = { -82, -68, -54, -40, -26, -12,   1,  15,
                   29,  43,  57,  71,  85,  99, 113, 126},
  tmsK5[]     = { -82, -70, -59, -47, -35, -24, -12,  -1,
                   11,  23,  34,  46,  57,  69,  81,  92},
  tmsK6[]     = { -64, -53, -42, -31, -20,  -9,   3,  14,
                   25,  36,  47,  58,  69,  80,  91, 102},
  tmsK7[]     = { -77, -65, -53, -41, -29, -17,  -5,   7,
                   19,  31,  43,  55,  67,  79,  90, 102},
  tmsK8[]     = { -64, -40, -16,   7,  31,  55,  79, 102},
  tmsK9[]     = { -64, -44, -24,  -4,  16,  37,  57,  77},
  tmsK10[]    = { -51, -33, -15,   4,  22,  32,  59,  77},
  chirp[]     = {   0,  42, -44,  50, -78,  18,  37,  20,
                    2, -31, -59,   2,  95,  90,   5,  15,
                   38,  -4, -91, -91, -42, -35, -36,  -4,
                   37,  43,  34,  33,  15,  -1,  -8, -18,
                  -19, -17,  -9, -10,  -6,   0,   3,   2, 1};

static const uint8_t PROGMEM
  tmsEnergy[] = {   0,   2,   3,   4,   5,   7,  10,  15,
                   20,  32,  41,  57,  81, 114, 161, 255},
  tmsPeriod[] = {   0,  16,  17,  18,  19,  20,  21,  22,
                   23,  24,  25,  26,  27,  28,  29,  30,
                   31,  32,  33,  34,  35,  36,  37,  38,
                   39,  40,  41,  42,  43,  45,  47,  49,
                   51,  53,  54,  57,  59,  61,  63,  66,
                   69,  71,  73,  77,  79,  81,  85,  87,
                   92,  95,  99, 102, 106, 110, 115, 119,
                  123, 128, 133, 138, 143, 149, 154, 160};

// Constructor for PWM mode
Talkie::Talkie(void) {
	pinMode(3, OUTPUT);  // OC2B
#ifdef PIEZO
	pinMode(11, OUTPUT); // OC2A
#endif
	csBitMask = 0;       // DAC not in use
}

// Constructor for DAC mode
Talkie::Talkie(uint8_t cs, uint8_t clk, uint8_t dat) {
	csPort     = portOutputRegister(digitalPinToPort(cs));
	csBitMask  = digitalPinToBitMask(cs);
	clkPort    = portOutputRegister(digitalPinToPort(clk));
	clkBitMask = digitalPinToBitMask(clk);
	datPort    = portOutputRegister(digitalPinToPort(dat));
	datBitMask = digitalPinToBitMask(dat);
	pinMode(cs , OUTPUT);
	pinMode(clk, OUTPUT);
	pinMode(dat, OUTPUT);
	*csPort   |=  csBitMask;  // Deselect
	*clkPort  &= ~clkBitMask; // Clock low
}

void Talkie::say(const uint8_t *addr, boolean block) {

	// Enable the speech system whenever say() is called.

	if(!csBitMask) {
		// Set up Timer2 for 8-bit, 62500 Hz PWM on OC2B
		TCCR2A  = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
		TCCR2B  = _BV(CS20); // No prescale
		TIMSK2  = 0;         // No interrupt
		OCR2B   = 0x80;      // 50% duty cycle
#ifdef PIEZO
		OCR2A   = 0x80;
		TCCR2A |= _BV(COM2A1) | _BV(COM2A0); // OC2A inverting mode
#endif
	}

	// Reset synth state and 'ROM' reader
	x0 = x1 = x2 = x3 = x4 = x5 = x6 = x7 = x8 =
	  periodCounter = buf = bufBits = 0;
	ptrAddr        = addr;
	interruptCount = TICKS;

	// Set up Timer1 to trigger periodic synth calc at 'FS' Hz
	TCCR1A = 0;                             // No output
	TCCR1B = _BV(WGM12) | _BV(CS10);        // CTC mode, no prescale
	OCR1A  = ((F_CPU + (FS / 2)) / FS) - 1; // 'FS' Hz (w/rounding)
	TCNT1  = 0;                             // Reset counter
	TIMSK1 = _BV(OCIE1A);                   // Compare match interrupt on

	if(block) while(TIMSK1 & _BV(OCIE1A));
}

boolean Talkie::talking(void) {
	return TIMSK1 & _BV(OCIE1A);
}

static inline uint8_t rev(uint8_t a) { // Reverse bit sequence in 8-bit value
	a = ( a         >> 4) | ( a         << 4); // 76543210 -> 32107654
	a = ((a & 0xCC) >> 2) | ((a & 0x33) << 2); // 32107654 -> 10325476
	a = ((a & 0xAA) >> 1) | ((a & 0x55) << 1); // 10325476 -> 01234567
	return a;
}

static uint8_t getBits(uint8_t bits) {
	uint8_t value;
	if(bits > bufBits) {
		buf     |= rev(pgm_read_byte(ptrAddr)) << (8 - bufBits);
		bufBits += 8;
		ptrAddr++; // Don't post-inc in pgm_read_byte! Is a macro.
	}
	value    = buf >> (16 - bits);
	buf    <<= bits;
	bufBits -= bits;
	return value;
}

static void dacOut(uint8_t value) {
	uint8_t bit;

	*csPort  &= ~csBitMask; // Select DAC

	// Clock out 4 bits DAC config (not in loop because it's constant)
	*datPort &= ~datBitMask; // 0 = Select DAC A, unbuffered
	*clkPort |=  clkBitMask; *clkPort &= ~clkBitMask;
	*clkPort |=  clkBitMask; *clkPort &= ~clkBitMask;
	*datPort |=  datBitMask; // 1X gain, enable = 1
	*clkPort |=  clkBitMask; *clkPort &= ~clkBitMask;
	*clkPort |=  clkBitMask; *clkPort &= ~clkBitMask;

	// Output is expanded from 8 to 12 bits for DAC.  Perhaps the
	// synthesizer math could be fiddled to generate 12-bit values.
	for(bit=0x80; bit; bit>>=1) { // Clock out first 8 bits of data
		if(value & bit) *datPort |=  datBitMask;
		else            *datPort &= ~datBitMask;
		*clkPort |= clkBitMask; *clkPort &= ~clkBitMask;
	}
	for(bit=0x80; bit >= 0x10; bit>>=1) { // Low 4 bits = repeat hi 4
		if(value & bit) *datPort |=  datBitMask;
		else            *datPort &= ~datBitMask;
		*clkPort |= clkBitMask; *clkPort &= ~clkBitMask;
	}
	*csPort  |=  csBitMask; // Unselect DAC
}

#define read8(base, bits)  pgm_read_byte(&base[getBits(bits)]);
#define read16(base, bits) pgm_read_word(&base[getBits(bits)]);

ISR(TIMER1_COMPA_vect) {
	int16_t u0;

	if(csBitMask) dacOut(nextPwm);
#ifdef PIEZO
	else          OCR2A = OCR2B = nextPwm;
#else
	else          OCR2B = nextPwm;
#endif

	if(++interruptCount >= TICKS) {
		// Read speech data, processing the variable size frames
		uint8_t energy;
		if((energy = getBits(4)) == 0) {  // Rest frame
			synthEnergy = 0;
		} else if(energy == 0xF) {        // Stop frame; silence
			TIMSK1 &= ~_BV(OCIE1A);   // Stop interrupt
			nextPwm = 0x80;           // Neutral
			if(csBitMask) dacOut(nextPwm);
			else          TCCR2A = 0; // Stop PWM out
			return;
		} else {
			synthEnergy    = pgm_read_byte(&tmsEnergy[energy]);
			uint8_t repeat = getBits(1);
			synthPeriod    = pgm_read_byte(&tmsPeriod[getBits(6)]);
			if(!repeat) { // A repeat frame uses last coefficients
				synthK1 = read16(tmsK1, 5); // All frames
				synthK2 = read16(tmsK2, 5); // use the first
				synthK3 = read8( tmsK3, 4); // 4 coefficients
				synthK4 = read8( tmsK4, 4);
				if(synthPeriod) {
					synthK5  = read8(tmsK5 , 4); // Voiced
					synthK6  = read8(tmsK6 , 4); // frames
					synthK7  = read8(tmsK7 , 4); // use
					synthK8  = read8(tmsK8 , 3); // six
					synthK9  = read8(tmsK9 , 3); // extra
					synthK10 = read8(tmsK10, 3); // coeffs
				}
			}
		}
		interruptCount = 0;
	}

	if(synthPeriod) { // Voiced source
		if(++periodCounter >= synthPeriod) periodCounter = 0;
		u0 = (periodCounter >= sizeof(chirp)) ? 0 :
		     (pgm_read_byte(&chirp[periodCounter]) *
                     (uint32_t)synthEnergy) >> 8;
	} else {          // Unvoiced source
		synthRand = (synthRand >> 1) ^ ((synthRand & 1) ? 0xB800 : 0);
		u0        = (synthRand & 1) ? synthEnergy : -synthEnergy;
	}
	u0     -=       ((synthK10 *          x9) +
	                 (synthK9  *          x8)) >>  7;
	x9      = x8  + ((synthK9  *          u0 ) >>  7);
	u0     -=       ((synthK8  *          x7 ) >>  7);
	x8      = x7  + ((synthK8  *          u0 ) >>  7);
	u0     -=       ((synthK7  *          x6 ) >>  7);
	x7      = x6  + ((synthK7  *          u0 ) >>  7);
	u0     -=       ((synthK6  *          x5 ) >>  7);
	x6      = x5  + ((synthK6  *          u0 ) >>  7);
	u0     -=       ((synthK5  *          x4 ) >>  7);
	x5      = x4  + ((synthK5  *          u0 ) >>  7);
	u0     -=       ((synthK4  *          x3 ) >>  7);
	x4      = x3  + ((synthK4  *          u0 ) >>  7);
	u0     -=       ((synthK3  *          x2 ) >>  7);
	x3      = x2  + ((synthK3  *          u0 ) >>  7);
	u0     -=       ((synthK2  * (int32_t)x1 ) >> 15);
	x2      = x1  + ((synthK2  * (int32_t)u0 ) >> 15);
	u0     -=       ((synthK1  * (int32_t)x0 ) >> 15);
	x1      = x0  + ((synthK1  * (int32_t)u0 ) >> 15);

	if(     u0 >  511) u0 =  511; // Output clamp
	else if(u0 < -512) u0 = -512;

	x0      =  u0;
	nextPwm = (u0 >> 2) + 0x80;
}
