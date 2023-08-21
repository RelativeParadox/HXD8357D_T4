#ifndef _HXD8357D_T4_H_
#define _HXD8357D_T4_H_

#include <Arduino.h>

//#if (!defined(__IMXRT1062__))  ||  (!defined(CORE_TEENSY))
//    #error "Library ILI9341_T4 only supports Teensy 4/4.1/micromod."
//#endif

class HXD8357DT4 {

public:
    HXD8357DT4(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset);

    void begin(uint16_t id = 0x9325);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
    void drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
    void fillScreen(uint16_t color);
    void reset(void);
    void setRegisters8(uint8_t *ptr, uint8_t n);
    void setRegisters16(uint16_t *ptr, uint8_t n);
    void setRotation(uint8_t x);

    // These methods are public in order for BMP examples to work:
    void setAddrWindow(int x1, int y1, int x2, int y2);
    void pushColors(uint16_t *data, uint8_t len, boolean first);

    uint16_t color565(uint8_t r, uint8_t g, uint8_t b), readPixel(int16_t x, int16_t y), readID(void);
    uint32_t readReg(uint8_t r);

private:
    void init();

    void write8(uint8_t value);

    void setWriteDir(void);
    void setReadDir(void);

    void writeRegister8(uint8_t a, uint8_t d);
    void writeRegister16(uint16_t a, uint16_t d);
    void writeRegister24(uint8_t a, uint32_t d);
    void writeRegister32(uint8_t a, uint32_t d);

    void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d);
    void setLR(void), flood(uint16_t color, uint32_t len);

    uint8_t driver;
    uint8_t read8fn(void);

    volatile uint8_t *csPort, *cdPort, *wrPort, *rdPort;
    uint8_t csPinSet, cdPinSet, wrPinSet, rdPinSet, csPinUnset, cdPinUnset,
            wrPinUnset, rdPinUnset, _reset;

};

#endif //_HXD8357D_T4_H_
