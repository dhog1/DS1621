#ifndef _AVR035_H_
#define _AVR035_H_

#define _BV(_bit) (1<<_bit)

        /* 8-bit data type */
#define byte     unsigned char
#define uint8_t  unsigned char
#define int8_t   signed char
#define boolean  bool
        /* 16-bit data type */
#define uint16_t unsigned int
#define int16_t  signed int
#define word     unsigned int
        /* 32-bit data */
#define uint32_t unsigned long int
#define int32_t  signed long int
#define dword    unsigned long int

// MIN/MAX macros
#define _MIN(a,b)			((a<b)?(a):(b))
#define _MAX(a,b)			((a>b)?(a):(b))

// w is 16 bits
#define LOWBYTE(w)   ((uint8_t) ((w) & 0x00FF))
#define HIGHBYTE(w)  ((uint8_t) ((w) >> 8))
#define SWAPBYTES(w) ((w) = LOWBYTE(w) << 8 | HIGHBYTE(w))

#define BITREAD(value, _bit)            (((value) >> (_bit)) & 0x01)
#define BITSET(value, _bit)             ((value) |= (1UL << (_bit)))
#define BITCLEAR(value, _bit)           ((value) &= ~(1UL << (_bit)))
#define BITWRITE(value, _bit, bitvalue) (bitvalue ? BITSET(value, _bit) : BITCLEAR(value, _bit))

    // from AVR035: Efficient C Coding for AVR

    #define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
    #define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
    #define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
    #define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

    #define SETBITMASK(x,y) (x |= (y))
    #define CLEARBITMASK(x,y) (x &= (~y))
    #define FLIPBITMASK(x,y) (x ^= (y))
    #define CHECKBITMASK(x,y) (x & (y))

    #define VARFROMCOMB(x, y) x
    #define BITFROMCOMB(x, y) y

    #define C_SETBIT(comb) SETBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
    #define C_CLEARBIT(comb) CLEARBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
    #define C_FLIPBIT(comb) FLIPBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
    #define C_CHECKBIT(comb) CHECKBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))

    #endif

// Usage
//    #define _SCK_PIN  PORTB, 5
//
//    C_SETBIT(_SCK_PIN);
//    if (C_CHECKBIT(_SCK_PIN)) {do something, pin is high}