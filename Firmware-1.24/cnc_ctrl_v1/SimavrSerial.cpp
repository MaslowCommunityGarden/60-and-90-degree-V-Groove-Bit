#include "SimavrSerial.h"

#ifdef Serial
#undef Serial
#endif

SimavrSerial_ SimavrSerial;

size_t SimavrSerial_::write(uint8_t c)
{
    size_t n = Serial.write(c);
    Serial.flush();

    return n;
}

void SimavrSerial_::begin(unsigned long baud) {
    Serial.begin(baud);
}

int SimavrSerial_::available() {
    return Serial.available();
}

int SimavrSerial_::read() {
    return Serial.read();
}
