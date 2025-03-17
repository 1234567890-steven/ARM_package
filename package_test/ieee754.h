#ifndef ieee754_H
#define ieee754_H

#include "Arduino.h"

class IEEE754
{
    public:
        IEEE754();
        void float_to_hex(float num, byte (&data)[4]);
        float hex_to_float(byte data[4]);
    private:
        uint8_t bytes[4];
        // byte data[4] = { 0 };
    
};

#endif