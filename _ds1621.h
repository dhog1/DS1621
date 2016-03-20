/*
    AVR ATMega 168/328 @16 MHz MCU
                   for 8 MHz MCU set appropriate TWBR (find string #define _TWI_INIT, look comment)


                DS1621 (LM1621) I2C Digital Thermometer and Thermostat, 9-bit (12-bit HIGH) resolution


            SDA  |------------|  Vdd        SDA, SCL   -  I2C interface
               --|1  <       8|--           A0, A1, A2 -  adress pin
            SCL  |            |  A0         Tout       -  Termostat out (HIGH/LOW active, selectable)
               --|2          7|--
            Tout |            |  A1
               --|3          6|--
            GND  |            |  A2
               --|4          5|--
                 |____________|


    Measures Temperatures From -55°C to +125°C in 0.5°C Increments.
    Temperature is Read as a 9-Bit Value (2-Byte Transfer) 
    Converts Temperature to Digital Word in Less than 1s 
    Thermostatic Settings are User Definable And Nonvolatile 
    Power Supply Range (2.7V to 5.5V) 
    Data is Read From/Written Via a 2-Wire Serial Interface (Open Drain I/O Lines) 
*/

#ifndef _ds1621_h
#define _ds1621_h

#define HI_PRECISION          // if defined, _1621_hiread() returns temperature with 12-bit resolution  
#define TERMOSTAT_INT         // if defined, _TPIN (any MCU pin choosen) reflects termostat condition


#include <avr035.h>
#include <delay.h>

#ifdef HI_PRECISION
    #include <math.h>
#endif


#define _1621_I2C_ADDRESS   0x4F    // default;    0b1001A0A1A2  Ax default HIGH, i.e. Ax = 1

#ifdef TERMOSTAT_INT
                                             // MCU pin wired to tout 1621 pin definition
                                             // here it's PORTD5 pin (D5), maybe any
    #define _TPIN_PRT      PORTD, PORTD5
    #define _TPIN_DDR      DDRD,  DDD5
    #define _TPIN_IO       PIND,  PIND5
    #define _TPIN_PCINT    PCINT21
    #define _TPIN_PCMSK    PCMSK2
    #define _TPIN_PCIE     PCIE2
    #define _TPIN_PCIF     PCIF2

    #define SET_TERMO_ALARM  { C_CLEARBIT(_TPIN_DDR); C_SETBIT(_TPIN_PRT); \
                               PCICR |= (1<<_TPIN_PCIE); PCIFR |= (1<<_TPIN_PCIF); _TPIN_PCMSK |= (1<<_TPIN_PCINT); }
    #define OFF_TERMO_ALARM  { _TPIN_PCMSK &= ~(1<<_TPIN_PCINT); }

#endif

#define _1621_REG_CONFIG    0xAC    // 1621 CONFIG register
    #define BIT_DONE    7           //  Conversion Done bit. “1” = Conversion complete, “0” = Conversion in progress.
    #define BIT_THF     6           // Temperature High Flag. This bit will be set to “1” when the temperature is greater than or 
                                    // equal to the value of TH. It will remain “1” until reset by writing “0” into this location
                                    // or removing power from the device. This feature provides a method of determining if the DS1621
                                    // has ever been subjected to temperatures above TH while power has been applied. 
    #define BIT_TLF     5           //  Temperature Low Flag. This bit will be set to “1” when the temperature is less than or equal 
                                    // to the value of TL. It will remain “1” until reset by writing “0” into this location or 
                                    // removing power from the  device. This feature provides a method of determining if the DS1621
                                    // has ever been subjected to temperatures below TL while power has been applied. 
    #define BIT_NVB     4           //  Nonvolatile  Memory  Busy  flag. “1” = Write to an EEPROM memory cell in progress,
                                    // “0” = nonvolatile memory is not busy. A copy to EEPROM may take up to 10 ms.
    #define BIT_POL     1           // Output Polarity Bit. “1” = active high, “0” = active low. This bit is nonvolatile.
    #define BIT_1SHOT   0           // One Shot Mode. If 1SHOT is “1”, the DS1621 will perform one temperature conversion upon 
                                    // receipt of the Start Convert T protocol. If 1SHOT is “0”, the DS1621 will continuously perform 
                                    // temperature conversions. This bit is nonvolatile.

/*
                    Data (temperature) format
Since data is transmitted over the 2-wire bus MSB first, temperature data may be written to/read from the 
DS1621 as either a single byte (with temperature resolution of 1°C) or as two bytes. The second byte 
would contain the value of the least significant (0.5°C) bit of the temperature reading as shown in Table 
1. Note that the remaining 7 bits of this byte are set to all "0"s. 
 
Temperature is represented in the DS1621 in terms of a ½°C LSB, yielding the following 9-bit format: 

      TEMPERATURE, TH, and TL FORMAT                                  |      THERMOSTAT OUTPUT OPERATION (Active = High)
          MSB                LSB               means                  |       |               |
    1 1 1 0 0 1 1 1     0 0 0 0 0 0 0 0        -25C                   |       |               |
    0 0 0 1 1 0 0 1     0 0 0 0 0 0 0 0        +25C                   |       I<==============I==============->
    0 0 0 0 0 0 0 0     1 0 0 0 0 0 0 0        +1/2C                  |       I |             I ^
    1 1 1 1 1 1 1 1     1 0 0 0 0 0 0 0        -1/2C                  |       I v             I |
    0 1 1 1 1 1 0 1     0 0 0 0 0 0 0 0        +125C                  |       I               I
    1 1 0 0 1 0 0 1     0 0 0 0 0 0 0 0        -55C                   |       I               I
                                                                      |<======I===============>________________
                                                                             TL               TH            T-->

Higher resolutions may be obtained by reading the temperature and truncating the 0.5°C bit (the LSB) 
from the read value. This value is TEMP_READ. A Read Counter command should be issued to yield the 
COUNT_REMAIN value. The Read Slope command should then be issued to obtain the COUNT_PER_C value.
The higher resolution temperature may be then be calculated by the user using the following: 
 
TEMPERATURE = TEMP_READ-0.25 + (COUNT_PER_C - COUNT_REMAIN)/COUNT_PER_C

*/

                           // "commands"
#define _1621_read_temp     0xAA    // This command reads the last temperature conversion result. The DS1621 will send 2 bytes,
                                    // in the format described earlier, which are the contents of this register
#define _1621_read_count    0xA8    // This command reads the value Count_Remain
#define _1621_read_slope    0xA9    // This command reads the value Count_Per_C
#define _1621_start_conv    0xEE    // This command begins a temperature conversion. No further data is required. In one-shot mode the 
                                    // temperature conversion will be performed and then the DS1621 will remain idle. In continuous mode this 
                                    // command will initiate continuous conversions
#define _1621_stop_conv     0x22    // This command stops temperature conversion. No further data is required. This command may be used to 
                                    // halt a DS1621 in continuous conversion mode. After issuing this command, the current temperature 
                                    // measurement will be completed and the DS1621 will remain idle until a Start Convert T is issued to 
                                    // resume continuous operation
#define _1621_access_TH     0xA1    // If R/W is “0” this command writes to the TH (HIGH TEMPERATURE) register. After issuing this
                                    // command, the next 2 bytes written to the DS1621, in the same format as described for reading 
                                    // temperature, will set the high temperature threshold for operation of the T OUT output. If R/W is “1” the 
                                    // value stored in this register is read back. 
#define _1621_access_TL     0xA2    // If R/W is “0” this command writes to the TL (LOW TEMPERATURE) register. After issuing this 
                                    // command, the next 2 bytes written to the DS1621, in the same format as described for reading 
                                    // temperature, will set the high temperature threshold for operation of the T OUT output. If R/W is “1” the 
                                    // value stored in this register is read back.


#define _TWI_INIT  { TWSR &= 0xFC; TWBR = 0x0C; }   // @16Mhz MCU twi 100kHz TWBR = 0x48; twi 400kHz TWBR = 0x0C
                                                    // @8Mhz MCU  twi 100kHz TWBR = 0x20; twi 400kHz TWBR = 0x02 
#ifdef TERMOSTAT_INT
volatile unsigned char termostat_alarm = 0;
#endif

//          Function description

/*
    void _1621_init(signed char TL, signed char TH, unsigned char active_high)
    
    params: TL          - low termostat limit
            TH          - high termostat limit
            active_high - 1621 tout pin active state; 0 - low, !=0 - high

    The first 1621 call. Set termostat range and tout pin active polarity. If TERMOSTAT_INT defined,
    unsigned char termostat_alarm will be available. Var termostat_alarm changes from 0 to non-0 as
    termostat condition changes.
    Ex.:    _1621_init(45, 75, 0);
        init _1621 with termostat range 45C as low and 75C as high limits. If actual temperature is lower then 75C, the
        1621 tout pin is HIGH. When temperature will be higher then 75C, tout 1621 pin goes LOW (as polarity 0 was set)
        and will be LOW until the temperature drops lower then 45C. Then tout pin goes HIGH.
        
*/
void _1621_init(signed char TL, signed char TH, unsigned char active_high);

/*
    int16_t _1621_read(int16_t *temp)
    
    params: *temp   - pointer to int16_t var to hold temperature value
    return: signed temperature value multiply by 10; 265 means +26.5C, -75 means -7.5C, 220 means +22.0C
    
    _16121() read sensor value in 9-bit resolution
    
    Ex.: int tm;
         ...
         _1621_read(&tm);
         ...
         if (_1621(&tm) > 0) printf("The temperature is positive"); 
*/
int16_t _1621_read(int16_t *temp);

#ifdef HI_PRECISION
/*
    uint16_t _1621_hiread(uint16_t *temp)
    
    params: *temp   - pointer to uint16_t var to hold temperature value
    return: first  (MSB) byte: bit7 - sign (0 - "+", 1 - "-"), bit6..bit0 - unsigned temperature value integer part
            second (LSB) byte: fractional part of temperature in milligrad

    _1621_hiread() read sensor value in 12-bit resolution
    
    Ex.: uint16_t tm;
         char sgn[2] = "+";
         unsigned char val, frac_val;
         ...
         _1621_hiread(&tm);
         if (tm & 0x8000) {  // check sign
            tm &= ~0x8000;   // if "minus" reset sign bit
            sgn[1] = '-';
         };
         val = (tm & 0xFF00) >> 8;
         frac_val = (unsigned char)(tm & 0x00FF)
         printf("Temperature is: %s%d.%d", sgn, val, frac_val);   // something like 27.625
*/
uint16_t _1621_hiread(uint16_t *temp);
#endif

/*
    unsigned char _1621_is_TL(signed char *tl)
    
    params: *tl  - pointer to signed char var to hold termostat LOW temperature; here function will write low temperature
                   limit of termostat
    return: 0 if termostat low limit temperature was not achieved during last MCU reset or power on
            1 if termostat low limit temperature was achieved during last MCU reset or power on     
    
    _1621_is_TL() reports if termostat low limit have been ever achieved (during last MCU reset or power on)
    
    Ex.: signed char t_low;
         ...
         if (_1621_is_TL(t_low)) printf("The temperature was lower termostat low limit %d", t_low);
*/
unsigned char _1621_is_TL(signed char *tl);

/*
    unsigned char _1621_is_TH(signed char *th)
    
    params: *th  - pointer to signed char var to hold termostat LOW temperature; here function will write low temperature
                   limit of termostat
    return: 0 if termostat high limit temperature was not achieved during last MCU reset or power on
            1 if termostat high limit temperature was achieved during last MCU reset or power on     
    
    _1621_is_TH() reports if termostat high limit have been ever achieved (during last MCU reset or power on)
    
    Ex.: signed char t_hi;
         ...
         if (_1621_is_TL(t_hi)) printf("The temperature was higher termostat high limit %d", t_hi);
*/
unsigned char _1621_is_TH(signed char *th);

/*
    void _1621_reset_TL(void)
    
    _1621_reset_TL() resets (clears) information if termostat low limit have been ever achieved (during last MCU reset or power on)
*/
void _1621_reset_TL(void);

/*
    void _1621_reset_TH(void)
    
    _1621_reset_TH() resets (clears) information if termostat high limit have been ever achieved (during last MCU reset or power on)
*/
void _1621_reset_TH(void);

/*
    void _1621_set_mode(unsigned char continuous);
    
    params: continuous  = 0 for 1-shot measurement, i.e. measurement by request;
                       != 0 for continuous measurement (termostat usage for example)
*/
void _1621_set_mode(unsigned char continuous);

/*
    void _1621_stop(void)
    
    if 1621 was in continuous measurement mode stops temperature measurement 
    
*/
void _1621_stop(void);

/*
    void _1621_start(void)
    
    starts 1621 temperature measurement
    
    _1621_read() and _1621_hiread() functions internaly turn on temperature measurement before reading,
    so no need for this function usage when you want read current temperature 
*/
void _1621_start(void);


#endif
