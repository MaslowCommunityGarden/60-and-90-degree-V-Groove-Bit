#include "Maslow.h"

#ifndef SimavrSerial_h_
#define SimavrSerial_h_

#ifdef SIMAVR
#define Serial SimavrSerial
#endif

// This class is a wrapper around Serial, to be used when running in the simavr simulator
// (https://github.com/buserror/simavr)
//
// Simavr's UART seems to lock up and crash when you write more than a few characters to it.
// SimavrSerial works around this issue by flushing after every single character. On a real
// controller, this might cause a performance issue, so this class is only used if SIMAVR is defined.
// If SIMAVR is defined, though, any reference to Serial is replaces to a reference to SimavrSerial.
// This means that every Serial method that is used in our code also needs a wrapper in SimavrSerial,
// of the simavr environment will not compile.
//
// Not the cleanest thing ever, but it's the best I could come up with that 
// will have zero impact on the actual production code.
class SimavrSerial_ : public Print
{
    public:
    virtual size_t write(uint8_t);
    virtual int available();
    virtual int read();
    void begin(unsigned long baud);
};

extern SimavrSerial_ SimavrSerial;
#endif