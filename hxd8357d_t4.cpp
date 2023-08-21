#include "hxd8357d_t4.h"

#define TFTWIDTH 240
#define TFTHEIGHT 320

//Hardware Pin Controls
#define SET_RD_ACTIVE *rdPort &= rdPinUnset
#define SET_RD_IDLE *rdPort |= rdPinSet
#define SET_WR_ACTIVE *wrPort &= wrPinUnset
#define SET_WR_IDLE *wrPort |= wrPinSet
#define SET_CD_COMMAND *cdPort &= cdPinUnset
#define SET_CD_DATA *cdPort |= cdPinSet
#define SET_CS_ACTIVE *csPort &= csPinUnset
#define SET_CS_IDLE *csPort |= csPinSet
#define WR_STROBE {SET_WR_ACTIVE; SET_WR_IDLE;}

//HX8357 Series Op Codes
#define HX8357_NOP 0x00
#define HX8357_SWRESET 0x01
#define HX8357_RDDID 0x04
#define HX8357_RDDST 0x09

#define HX8357B_RDPOWMODE 0x0A
#define HX8357B_RDMADCTL 0x0B
#define HX8357B_RDCOLMOD 0x0C
#define HX8357B_RDDIM 0x0D
#define HX8357B_RDDSDR 0x0F

#define HX8357_SLPIN 0x10
#define HX8357_SLPOUT 0x11
#define HX8357B_PTLON 0x12
#define HX8357B_NORON 0x13

#define HX8357_INVOFF 0x20
#define HX8357_INVON 0x21
#define HX8357_DISPOFF 0x28
#define HX8357_DISPON 0x29

#define HX8357_CASET 0x2A
#define HX8357_PASET 0x2B
#define HX8357_RAMWR 0x2C
#define HX8357_RAMRD 0x2E

#define HX8357B_PTLAR 0x30
#define HX8357_TEON 0x35
#define HX8357_TEARLINE 0x44
#define HX8357_MADCTL 0x36
#define HX8357_COLMOD 0x3A

#define HX8357_SETOSC 0xB0
#define HX8357_SETPWR1 0xB1
#define HX8357B_SETDISPLAY 0xB2
#define HX8357_SETRGB 0xB3
#define HX8357D_SETCOM 0xB6

#define HX8357B_SETDISPMODE 0xB4
#define HX8357D_SETCYC 0xB4
#define HX8357B_SETOTP 0xB7
#define HX8357D_SETC 0xB9

#define HX8357B_SET_PANEL_DRIVING 0xC0
#define HX8357D_SETSTBA 0xC0
#define HX8357B_SETDGC 0xC1
#define HX8357B_SETID 0xC3
#define HX8357B_SETDDB 0xC4
#define HX8357B_SETDISPLAYFRAME 0xC5
#define HX8357B_GAMMASET 0xC8
#define HX8357B_SETCABC 0xC9
#define HX8357_SETPANEL 0xCC

#define HX8357B_SETPOWER 0xD0
#define HX8357B_SETVCOM 0xD1
#define HX8357B_SETPWRNORMAL 0xD2

#define HX8357B_RDID1 0xDA
#define HX8357B_RDID2 0xDB
#define HX8357B_RDID3 0xDC
#define HX8357B_RDID4 0xDD

#define HX8357D_SETGAMMA 0xE0

#define HX8357B_SETGAMMA 0xC8
#define HX8357B_SETPANELRELATED 0xE9

#define HX8357B_MADCTL_MY 0x80
#define HX8357B_MADCTL_MX 0x40
#define HX8357B_MADCTL_MV 0x20
#define HX8357B_MADCTL_ML 0x10
#define HX8357B_MADCTL_RGB 0x00
#define HX8357B_MADCTL_BGR 0x08
#define HX8357B_MADCTL_MH 0x04

#define ILI9341_COLADDRSET 0x2A
#define ILI9341_PAGEADDRSET 0x2B

// Initialization command table
#define TFTLCD_DELAY 0xFF
static const uint8_t HX8357D_regValues[] PROGMEM = {
        HX8357_SWRESET,
        0,
        HX8357D_SETC,
        3,
        0xFF,0x83,0x57,
        TFTLCD_DELAY,
        250,
        HX8357_SETRGB,
        4,
        0x00,0x00,0x06,0x06,
        HX8357D_SETCOM,
        1,
        0x25, // -1.52V
        HX8357_SETOSC,
        1,
        0x68, // Normal mode 70Hz, Idle mode 55 Hz
        HX8357_SETPANEL,
        1,
        0x05, // BGR, Gate direction swapped
        HX8357_SETPWR1,
        6,
        0x00,0x15,0x1C,
        0x1C,0x83,0xAA,
        HX8357D_SETSTBA,
        6,
        0x50,0x50,0x01,
        0x3C,0x1E,0x08,
        // MEME GAMMA HERE
        HX8357D_SETCYC,
        7,
        0x02,0x40,0x00,
        0x2A,0x2A,0x0D,
        0x78,
        HX8357_COLMOD,
        1,
        0x55,
        HX8357_MADCTL,
        1,
        0xC0,
        HX8357_TEON,
        1,
        0x00,
        HX8357_TEARLINE,
        2,
        0x00,0x02,
        HX8357_SLPOUT,
        0,
        TFTLCD_DELAY,
        150,
        HX8357_DISPON,
        0,
        TFTLCD_DELAY,
        50,
};

HXD8357DT4::HXD8357DT4(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset){
    // Convert pin numbers to registers and bitmasks
    _reset = reset;

    csPort = portOutputRegister(digitalPinToPort(cs));
    cdPort = portOutputRegister(digitalPinToPort(cd));
    wrPort = portOutputRegister(digitalPinToPort(wr));
    rdPort = portOutputRegister(digitalPinToPort(rd));

    csPinSet = digitalPinToBitMask(cs);
    cdPinSet = digitalPinToBitMask(cd);
    wrPinSet = digitalPinToBitMask(wr);
    rdPinSet = digitalPinToBitMask(rd);
    csPinUnset = ~csPinSet;
    cdPinUnset = ~cdPinSet;
    wrPinUnset = ~wrPinSet;
    rdPinUnset = ~rdPinSet;

    *csPort |= csPinSet; // Set all control bits to HIGH (idle)
    *cdPort |= cdPinSet; // Signals are ACTIVE LOW
    *wrPort |= wrPinSet;
    *rdPort |= rdPinSet;

    pinMode(cs, OUTPUT); // Enable outputs
    pinMode(cd, OUTPUT);
    pinMode(wr, OUTPUT);
    pinMode(rd, OUTPUT);

    if (reset) {
        digitalWrite(reset, HIGH);
        pinMode(reset, OUTPUT);
    }

    init();
}

void HXD8357DT4::init(){
    setWriteDir(); // Set up LCD data port(s) for WRITE operations
    rotation = 0;
    cursor_y = cursor_x = 0;
    textcolor = 0xFFFF;
    _width = TFTWIDTH;
    _height = TFTHEIGHT;
    int x = GPIO6_DR;
}

void HXD8357DT4::begin(uint16_t id) {
    uint8_t i = 0;

    reset();

    delay(200);
    // HX8357D
    driver = 3;
    SET_CS_ACTIVE;
    while (i < sizeof(HX8357D_regValues)) {
        uint8_t r = pgm_read_byte(&HX8357D_regValues[i++]);
        uint8_t len = pgm_read_byte(&HX8357D_regValues[i++]);
        if (r == TFTLCD_DELAY) {
            delay(len);
        } else {
            // Serial.print("Register $"); Serial.print(r, HEX);
            // Serial.print(" datalen "); Serial.println(len);

            SET_CS_ACTIVE;
            SET_CD_COMMAND;
            write8(r);
            SET_CD_DATA;
            for (uint8_t d = 0; d < len; d++) {
                uint8_t x = pgm_read_byte(&HX8357D_regValues[i++]);
                write8(x);
            }
            SET_CS_IDLE;
        }
    }
    return;
}

void HXD8357DT4::reset() {
    SET_CS_IDLE;
    //  CD_DATA;
    SET_WR_IDLE;
    SET_RD_IDLE;

    if (_reset) {
        digitalWrite(_reset, LOW);
        delay(2);
        digitalWrite(_reset, HIGH);
    }

    // Data transfer sync
    SET_CS_ACTIVE;
    SET_CD_COMMAND;
    write8(0x00);
    for (uint8_t i = 0; i < 3; i++)
        WR_STROBE; // Three extra 0x00s
    SET_CS_IDLE;
}

// Sets the LCD address window.
// Relevant to rect/screen fills and H/V lines.  Input coordinates are
// assumed pre-sorted (e.g. x2 >= x1).
void HXD8357DT4::setAddrWindow(int x1, int y1, int x2, int y2) {
    SET_CS_ACTIVE;

    uint32_t t;
    t = x1;
    t <<= 16;
    t |= x2;
    writeRegister32(ILI9341_COLADDRSET, t); // HX8357D uses same registers!
    t = y1;
    t <<= 16;
    t |= y2;
    writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!

    SET_CS_IDLE;
}

void HXD8357DT4::flood(uint16_t color, uint32_t len) {
    uint16_t blocks;
    uint8_t i, hi = color >> 8, lo = color;

    SET_CS_ACTIVE;
    SET_CD_COMMAND;

    write8(HX8357_RAMWR);

    // Write first pixel normally, decrement counter by 1
    SET_CD_DATA;
    write8(hi);
    write8(lo);
    len--;

    blocks = (uint16_t)(len / 64); // 64 pixels/block
    if (hi == lo) {
        // High and low bytes are identical.  Leave prior data
        // on the port(s) and just toggle the write strobe.
        while (blocks--) {
            i = 16; // 64 pixels/block / 4 pixels/pass
            do {
                WR_STROBE;
                WR_STROBE;
                WR_STROBE;
                WR_STROBE; // 2 bytes/pixel
                WR_STROBE;
                WR_STROBE;
                WR_STROBE;
                WR_STROBE; // x 4 pixels
            } while (--i);
        }
        // Fill any remaining pixels (1 to 64)
        for (i = (uint8_t)len & 63; i--;) {
            WR_STROBE;
            WR_STROBE;
        }
    } else {
        while (blocks--) {
            i = 16; // 64 pixels/block / 4 pixels/pass
            do {
                write8(hi);
                write8(lo);
                write8(hi);
                write8(lo);
                write8(hi);
                write8(lo);
                write8(hi);
                write8(lo);
            } while (--i);
        }
        for (i = (uint8_t)len & 63; i--;) {
            write8(hi);
            write8(lo);
        }
    }
    SET_CS_IDLE;
}

void HXD8357DT4::pushColors(uint16_t *data, uint8_t len, boolean first) {

}