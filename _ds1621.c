
void twi_master_trans(uint8_t twi_addr, char* outdata, uint8_t outlen, char* indata, uint8_t inlen) {

uint8_t i;
                                            // send START condition
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);
    while (!(TWCR & (1<<TWINT)));

    if (outlen) {                           // if there are data to be sent

        TWDR = twi_addr << 1;               // twi slave bus address + Write (SLA+W)
        TWCR = (1<<TWINT)|(1<<TWEN);
        while (!(TWCR & (1<<TWINT)));

        for (i=0; i<outlen; i++) {          // send data bytes
            TWDR = *(outdata+i);
            TWCR = (1<<TWINT)|(1<<TWEN);
            while (!(TWCR & (1<<TWINT)));
        };
    };

    if (inlen) {                            // if there are data to be received

        if (outlen) {                       // send Repeated Start after TX to switch in MR mode
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);
            while (!(TWCR & (1<<TWINT)));
        }

        TWDR = (twi_addr << 1) | 0x01;      // twi slave bus addr + Read (SLA+R)
        TWCR = (1<<TWINT)|(1<<TWEN);
        while (!(TWCR & (1<<TWINT)));

        for (i=0; i<inlen-1; i++) {
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);   // send ACK
            while (!(TWCR & (1<<TWINT)));
            *(indata+i) = TWDR;
        };

        TWCR = (1<<TWINT)|(1<<TWEN);            // last data byte w/o ACK
        while (!(TWCR & (1<<TWINT)));
        *(indata+inlen-1) = TWDR;
    };

    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);     // STOP condition
}


void _1621_init(signed char TL, signed char TH, unsigned char active_high) {

unsigned char buf[3];

    _TWI_INIT;

    buf[0] = _1621_access_TL;
    buf[1] = TL;
    buf[2] = 0;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 3, 0, 0);
    
    buf[0] = _1621_access_TH;
    buf[1] = TH;
    buf[2] = 0;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 3, 0, 0);
    
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    if (active_high) buf[1] |= (1<<BIT_POL); else buf[1] &= ~(1<<BIT_POL);
    buf[1] |= (1<<BIT_1SHOT);          // single
    twi_master_trans(_1621_I2C_ADDRESS, buf, 2, 0, 0);
    
    buf[0] = _1621_start_conv;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, 0, 0);
    
    buf[0] = _1621_REG_CONFIG;
    while (!(buf[1] & (1<<BIT_DONE))) {
        delay_ms(1);
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    };
#ifdef TERMOSTAT_INT
    SET_TERMO_ALARM;
#endif
}


void _1621_set_mode(unsigned char continuous) {

unsigned char buf[2];

    _TWI_INIT;
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    if (continuous) buf[1] &= ~(1<<BIT_1SHOT); else buf[1] |= (1<<BIT_1SHOT);
    twi_master_trans(_1621_I2C_ADDRESS, buf, 2, 0, 0);
}


void _1621_stop(void) {

unsigned char buf;

    _TWI_INIT;
    buf = _1621_stop_conv;
    twi_master_trans(_1621_I2C_ADDRESS, &buf, 1, 0, 0);    
}


void _1621_start(void) {

unsigned char buf;

    _TWI_INIT;
    buf = _1621_start_conv;
    twi_master_trans(_1621_I2C_ADDRESS, &buf, 1, 0, 0);
}


unsigned char _1621_is_TL(signed char *tl) {

unsigned char buf[3];

    buf[0] = _1621_access_TL;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    *tl = buf[1];
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    if (buf[1] & (1<<BIT_TLF)) return 1; else return 0;
}


unsigned char _1621_is_TH(signed char *th) {

unsigned char buf[3];

    buf[0] = _1621_access_TH;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    *th = buf[1];
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    if (buf[1] & (1<<BIT_THF)) return 1; else return 0;
}


void _1621_reset_TL(void) {

unsigned char buf[2];

    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    buf[1] &= ~(1<<BIT_TLF);
    twi_master_trans(_1621_I2C_ADDRESS, buf, 2, 0, 0);
}


void _1621_reset_TH(void) {

unsigned char buf[2];

    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    buf[1] &= ~(1<<BIT_THF);
    twi_master_trans(_1621_I2C_ADDRESS, buf, 2, 0, 0);
}


int16_t _1621_read(int16_t *temp) {

unsigned char buf[3];
unsigned char conf;

    _TWI_INIT;
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &conf, 1);
    
    conf &= ~(1<<BIT_DONE);
    twi_master_trans(_1621_I2C_ADDRESS, &conf, 1, 0, 0);
    
    nrdy_cnt = 0;
    if (conf & (1<<BIT_1SHOT)) {

        buf[0] = _1621_start_conv;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, 0, 0);

        buf[0] = _1621_REG_CONFIG;
        buf[1] = conf;
        while (!(buf[1] & (1<<BIT_DONE))) {
            delay_us(20);
            nrdy_cnt++;
            twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
       };

        buf[0] = _1621_read_temp;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    
    } else {
        
        buf[0] = _1621_REG_CONFIG;
        while (!(conf & (1<<BIT_DONE))) {
            delay_us(20);
            nrdy_cnt++;
            twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &conf, 1);
        };

        buf[0] = _1621_read_temp;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    };

    *temp = ((int16_t) buf[1]) * 10;       // signed temperature value multiply by 10; -125 means -12.5
    if (buf[2] & 0x80) *temp += 5;
    return *temp;
}


#ifdef HI_PRECISION

uint16_t _1621_hiread(uint16_t *temp) {

unsigned char buf[3];
unsigned char conf;
signed char b;
float a, c;

    _TWI_INIT;
    buf[0] = _1621_REG_CONFIG;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &conf, 1);
    
    conf &= ~(1<<BIT_DONE);
    twi_master_trans(_1621_I2C_ADDRESS, &conf, 1, 0, 0);
    
    nrdy_cnt = 0;
    if (conf & (1<<BIT_1SHOT)) {

        buf[0] = _1621_start_conv;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, 0, 0);

        buf[0] = _1621_REG_CONFIG;
        buf[1] = conf;
        while (!(buf[1] & (1<<BIT_DONE))) {
            delay_us(20);
            nrdy_cnt++;
            twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
       };

        buf[0] = _1621_read_temp;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    
    } else {
        
        buf[0] = _1621_REG_CONFIG;
        while (!(conf & (1<<BIT_DONE))) {
            delay_us(20);
            nrdy_cnt++;
            twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &conf, 1);
        };

        buf[0] = _1621_read_temp;
        twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 2);
    };

    a = (float)(buf[1]) - 0.25F;
    buf[0] = _1621_read_count;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[1], 1);
    buf[0] = _1621_read_slope;
    twi_master_trans(_1621_I2C_ADDRESS, buf, 1, &buf[2], 1);
    a += 1.0 - (float)buf[1]/(float)buf[2];

    a = modf(a, &c);
    b = (signed char)c;
    buf[1] = (unsigned char)(a * 1000); 
    *temp = (uint16_t)(b << 8) | (uint16_t)buf[1];       // return temp & 0xFF00 - signed integer part of temperature
                                                         //        temp & 0x00FF - fractional part of temperature in millicen
    return *temp;
}

#endif


#ifdef TERMOSTAT_INT

interrupt [PC_INT2] void pin_change_isr2(void) {        // Pin change 16-23 interrupt service routine
    termostat_alarm = ~termostat_alarm;
}

#endif
