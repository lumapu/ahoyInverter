//-----------------------------------------------------------------------------
// 2023 Ahoy, https://github.com/lumpapu/ahoy
// Creative Commons - http://creativecommons.org/licenses/by-nc-sa/4.0/deed
//-----------------------------------------------------------------------------

#include "Arduino.h"
#include <RF24.h>
#include "SPI.h"
#include <cstdint>
#include <iostream>
#include <sstream>
#include <algorithm> 
#define SPI_SPEED           1000000

#if defined(ESP32)
    #define PIN_CS          5
    #define PIN_CE          4
    #define PIN_IRQ         16
    #define PIN_MISO        12 // 19
    #define PIN_MOSI        13 // 23
    #define PIN_SCLK        14 // 18
#else
    #define PIN_CS          15
    #define PIN_CE          0
    #define PIN_IRQ         2
#endif

#define MAX_RF_PAYLOAD_SIZE 27

#define CRC8_INIT               0x00
#define CRC8_POLY               0x01
#define CRC16_MODBUS_POLYNOM    0xA001

typedef enum {
    TurnOn                  = 0,  // 0x00
    TurnOff                 = 1,  // 0x01
    Restart                 = 2,  // 0x02
    Lock                    = 3,  // 0x03
    Unlock                  = 4,  // 0x04
    ActivePowerContr        = 11, // 0x0b
    ReactivePowerContr      = 12, // 0x0c
    PFSet                   = 13, // 0x0d
    CleanState_LockAndAlarm = 20, // 0x14
    SelfInspection          = 40, // 0x28, self-inspection of grid-connected protection files
    Init                    = 0xff
} DevControlCmdType;

#define CP_U32_BigEndian(buf, v) do { \
    uint8_t *b = buf; \
    b[3] = ((v >> 24) & 0xff); \
    b[2] = ((v >> 16) & 0xff); \
    b[1] = ((v >>  8) & 0xff); \
    b[0] = ((v      ) & 0xff); \
} while (0)

std::string inverter_id = "116111223344"; // example inverter needs to be added to dtu

bool gotIrq;
SPIClass* mSpi;
RF24 mNrf24;

uint8_t mRxLen;
uint8_t mRxBuf[MAX_RF_PAYLOAD_SIZE];
uint8_t mTxBuf[4][MAX_RF_PAYLOAD_SIZE];
uint8_t mTxCh[] = {3, 3, 3, 3, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
uint8_t mTxMs[] = {36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36};
uint16_t mTxSendMs[30];
uint32_t mStart;

uint8_t mRxCh = 0;
uint8_t mChList[5] = {03, 23, 40, 61, 75};

uint16_t mCrc = 0xffff;
bool mRetransmit = false;
uint8_t mSendCnt = 0;

uint16_t mAcPower = 0;
uint16_t counter = 0;
uint16_t mPowerLimit = 8000; // 800.0W

uint8_t gotRx = 0;
uint32_t mMillis = millis();

// will be filled in Setup() method
uint64_t invId = 0;
uint64_t dtu   = 0; // the ID will be read of the received data - this "inverter" 
                    // can be read from multiple inverters from now on (without change)


uint64_t convertSerialNumber(const std::string& serialNumber) {
    if (serialNumber.length() < 8) {
        std::cerr << "Serial number too short." << std::endl;
        return 0;
    }

    // Extract the last 8 characters (last 4 bytes)
    std::string last4Bytes = serialNumber.substr(serialNumber.length() - 8);
    std::reverse(last4Bytes.begin(), last4Bytes.end());
    // Reverse the bytes and add "01" at the end
    std::string converted = last4Bytes + "01";

    // Convert the string to uint64_t
    std::stringstream ss;
    ss << std::hex << converted;
    uint64_t result;
    ss >> result;

    return result;
}
void write();

IRAM_ATTR void handleIntr(void) {
    gotIrq = true;
}

uint8_t crc8(uint8_t buf[], uint8_t len) {
    uint8_t crc = CRC8_INIT;
    for(uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for(uint8_t b = 0; b < 8; b ++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRC8_POLY : 0x00);
        }
    }
    return crc;
}

uint16_t crc16(uint8_t buf[], uint8_t len, uint16_t start) {
    uint16_t crc = start;
    uint8_t shift = 0;

    for(uint8_t i = 0; i < len; i ++) {
        crc = crc ^ buf[i];
        for(uint8_t bit = 0; bit < 8; bit ++) {
            shift = (crc & 0x0001);
            crc = crc >> 1;
            if(shift != 0)
                crc = crc ^ CRC16_MODBUS_POLYNOM;
        }
    }
    return crc;
}

void initPacket(uint64_t ivId, uint8_t buf[], uint8_t mid, uint8_t pid) {
    buf[0] = mid;
    CP_U32_BigEndian(&buf[1], ivId >> 8);
    CP_U32_BigEndian(&buf[5], ivId >> 8);
    buf[9] = pid;
    memset(&buf[10], 0x00, (MAX_RF_PAYLOAD_SIZE-10));
}

uint8_t initPayload(uint8_t cmd) {
    if(0x0b == cmd) {
        for(uint8_t i = 0; i < 4; i++) {
            initPacket(invId, mTxBuf[i], 0x95, (i+1));
        }

        mTxBuf[3][10+2]  = (mAcPower >> 8) & 0xff; // AC Power
        mTxBuf[3][10+3]  = (mAcPower     ) & 0xff; // AC Power
        mAcPower++;

        // 41.8V
        mTxBuf[0][10+2] = 0x01; // Voltage CH1
        mTxBuf[0][10+3] = 0xa2; // Voltage CH1
        // 8.49A
        mTxBuf[0][10+4] = 0x03; // Current CH1
        mTxBuf[0][10+5] = 0x51; // Current CH1
        //354.9W
        mTxBuf[0][10+8] = 0x0d; // DC Power CH1
        mTxBuf[0][10+9] = 0xdd; // DC Power CH1

        //8.20A
        mTxBuf[0][10+6] = 0x03; // Current CH2
        mTxBuf[0][10+7] = 0x34; // Current CH2
        //342.8W
        mTxBuf[0][10+10] = 0x0d; // DC Power CH2
        mTxBuf[0][10+11] = 0x64; // DC Power CH2
        
        // 123.456
        mTxBuf[0][10+12] = 0x00; // Yield Total CH1
        mTxBuf[0][10+13] = 0x01; // Yield Total CH1
        mTxBuf[0][10+14] = 0xe2; // Yield Total CH1
        mTxBuf[0][10+15] = 0x40; // Yield Total CH1

        // 132.567
        mTxBuf[1][10+0] = 0x00; // Yield Total CH2
        mTxBuf[1][10+1] = 0x02; // Yield Total CH2
        mTxBuf[1][10+2] = 0x05; // Yield Total CH2
        mTxBuf[1][10+3] = 0xd7; // Yield Total CH2

        // 381.4W
        mTxBuf[2][10+0] = 0x0e; // DC Power CH4
        mTxBuf[2][10+1] = 0xe6; // DC Power CH4

        // 61.447
        mTxBuf[2][10+2] = 0x00; // Yield Total CH3
        mTxBuf[2][10+3] = 0x00; // Yield Total CH3
        mTxBuf[2][10+4] = 0xf0; // Yield Total CH3
        mTxBuf[2][10+5] = 0x07; // Yield Total CH3

        // 47.777
        mTxBuf[2][10+6] = 0x00; // Yield Total CH4
        mTxBuf[2][10+7] = 0x00; // Yield Total CH4
        mTxBuf[2][10+8] = 0xba; // Yield Total CH4
        mTxBuf[2][10+9] = 0xal; // Yield Total CH4

        for(uint8_t i = 0; i < 4; i++) {
            if(3 == i) {
                mTxBuf[i][9] |= 0x80;
                mCrc = crc16(&mTxBuf[i][10], 14, mCrc);
                mTxBuf[i][24] = (mCrc >> 8) & 0xff;
                mTxBuf[i][25] = (mCrc     ) & 0xff;
            } else {
                mCrc = crc16(&mTxBuf[i][10], 16, mCrc);
            }
            mTxBuf[i][26] = crc8(mTxBuf[i], 26);
        }
        return 4; // number of frames
    } else if(0x00 == cmd) { // HW Version
        initPacket(invId, mTxBuf[0], 0x95, 0x81);
        mTxBuf[0][11] = 0x10;
        mTxBuf[0][12] = 0x10;
        mTxBuf[0][13] = 0x12;
        mTxBuf[0][14] = 0x10;
        mTxBuf[0][15] = 0x10;
        mTxBuf[0][16] = 0x01;
    } else if(0x01 == cmd) { // FW Version
        initPacket(invId, mTxBuf[0], 0x95, 0x81);
        mTxBuf[0][10] = 0x80;
        mTxBuf[0][11] = 0x01;
        mTxBuf[0][12] = 0x00;
        mTxBuf[0][13] = 0x17;
    } else if(0x02 == cmd) { // Grid profile
        initPacket(invId, mTxBuf[0], 0x95, 0x81);
        mTxBuf[0][10] = 0x80;
        mTxBuf[0][11] = 0x01;
        mTxBuf[0][12] = 0x00;
        mTxBuf[0][13] = 0x17;
    } else if(0x05 == cmd) { // System Config
        initPacket(invId, mTxBuf[0], 0x95, 0x81);
        uint16_t limitPct = mPowerLimit / 12000;
        mTxBuf[0][12] = (limitPct >> 8) & 0xff;
        mTxBuf[0][13] = (limitPct) & 0xff;
    } else
        Serial.println("unkown command: " + String(cmd));

    mCrc = crc16(&mTxBuf[0][10], 14, mCrc);
    mTxBuf[0][24] = (mCrc >> 8) & 0xff;
    mTxBuf[0][25] = (mCrc     ) & 0xff;
    mTxBuf[0][26] = crc8(mTxBuf[0], 26);
    return 1; // number of frames
}

uint8_t initCtrlPayload(uint8_t cmd) {
    uint8_t payloadLen = 4;
    if(ActivePowerContr == cmd) {
        mPowerLimit = (mRxBuf[12] << 8) | mRxBuf[13];
        Serial.println("got Power limit " + String(mPowerLimit / 10.0) + "W");
        initPacket(invId, mTxBuf[0], 0xD1, 0x81);
        mTxBuf[0][12] = cmd;
    } else if(TurnOn == cmd) {
        Serial.println("Turn on");
        initPacket(invId, mTxBuf[0], 0xD1, 0x81);
        mTxBuf[0][12] = cmd;
    } else if(TurnOff == cmd) {
        Serial.println("Turn off");
        initPacket(invId, mTxBuf[0], 0xD1, 0x81);
        mTxBuf[0][12] = cmd;
    } else if(Restart == cmd) {
        initPacket(invId, mTxBuf[0], 0xD1, 0x81);
        mTxBuf[0][12] = cmd;
        ESP.restart();
    } else
        Serial.println("unkown command: " + String(cmd));

    mCrc = crc16(&mTxBuf[0][10], payloadLen, mCrc);
    mTxBuf[0][10+payloadLen] = (mCrc >> 8) & 0xff;
    mTxBuf[0][11+payloadLen] = (mCrc     ) & 0xff;
    mTxBuf[0][12+payloadLen] = crc8(mTxBuf[0], 12+payloadLen);

    return 1;
}

void dumpBuf(uint8_t buf[], uint8_t len) {
    for(uint8_t i = 0; i < len; i++) {
        if(buf[i] < 16)
            Serial.print("0");
        Serial.print(String(buf[i], HEX) + " ");
    }
    Serial.println("");
}
#ifndef UNIT_TEST
void setup() {
    invId = convertSerialNumber(inverter_id);
    Serial.begin(115200);
    while (!Serial)
        yield();

    gotIrq = false;

    Serial.println("");
    Serial.println("start");
    pinMode(PIN_IRQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_IRQ), handleIntr, FALLING);

#if defined(ESP32)
    mSpi = new SPIClass(VSPI);
    mSpi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);
#else
    mSpi = new SPIClass();
    mSpi->begin();
#endif

    mNrf24.begin(mSpi, PIN_CE, PIN_CS);
    mNrf24.setRetries(3, 15); // 3*250us + 250us and 15 loops -> 15ms

    mNrf24.setChannel(61);
    mNrf24.startListening();
    mNrf24.setDataRate(RF24_250KBPS);
    mNrf24.setAutoAck(true);
    mNrf24.enableDynamicPayloads();
    mNrf24.setCRCLength(RF24_CRC_16);
    mNrf24.setAddressWidth(5);
    mNrf24.openReadingPipe(1, reinterpret_cast<uint8_t*>(&invId));
    mNrf24.setPALevel(0x01);

    // enable all receiving interrupts
    mNrf24.maskIRQ(false, false, false);

    if(mNrf24.isChipConnected()) {
        Serial.println("Radio found");
        mNrf24.printPrettyDetails();
    } else
        Serial.println("NRF24 can't be reached");
}
void loop() {
    if(gotIrq) {
        gotIrq = false;

        bool tx_ok, tx_fail, rx_ready;
        mNrf24.whatHappened(tx_ok, tx_fail, rx_ready); // resets the IRQ pin to HIGH

        if(rx_ready) {
            while(mNrf24.available()) {
                mRxLen = mNrf24.getDynamicPayloadSize(); // if payload size > 32, corrupt payload has been flushed
                if (mRxLen > 0)
                    mNrf24.read(mRxBuf, mRxLen);

                mCrc = 0xffff; // reset CRC
                mMillis = millis() + 30;
                mStart = millis();
                mSendCnt = 0;
                //RX 15 11 22 33 44 83 53 57 6d 82 39 
                dtu = (mRxBuf[8] << 24) | (mRxBuf[7] << 16) | (mRxBuf[6] << 8) | (mRxBuf[5]);
                dtu = (dtu << 8) | 0x01;
                if(mRxLen == 27)
                    gotRx = initPayload(mRxBuf[10]);
                else {
                    if(mRxBuf[0] == 0x51)
                        gotRx = initCtrlPayload(mRxBuf[10]);
                    else
                        gotRx = 0;
                    mRetransmit = true;
                }
            }
        }
    } else if((gotRx) && (millis() > mMillis)) {
        gotRx--;

        uint8_t *send = mTxBuf[mSendCnt];
        if(255 != gotRx) {
            if(mRetransmit)
                send = mTxBuf[(mRxBuf[9] & 0x7f)-1];
            
            mNrf24.stopListening();
            mNrf24.setChannel(mTxCh[mSendCnt]);
            mNrf24.openWritingPipe(reinterpret_cast<uint8_t*>(&dtu));
            mNrf24.write(send, 27, false); // false = request ACK response
            mTxSendMs[mSendCnt] = millis() - mStart;
            mMillis = millis() + mTxMs[mSendCnt]; //millis() + 40; // 
            mSendCnt++;
        } else
            gotRx = 0;

        if(0 == gotRx) {
            Serial.print("RX ");
            dumpBuf(mRxBuf, mRxLen);
            if(!mRetransmit) {
                for(uint8_t i = 0; i < mSendCnt; i++) {
                    Serial.print("TX CH");
                    if(mTxCh[i] < 10)
                        Serial.print("0");
                    Serial.print(String(mTxCh[i]));
                    Serial.print(" ");
                    Serial.print(mTxSendMs[i]);
                    Serial.print("ms ");
                    dumpBuf(mTxBuf[i], 27);
                }
            } else {
                Serial.print("TX CH");
                    if(mTxCh[0] < 10)
                        Serial.print("0");
                    Serial.print(String(mTxCh[0]));
                    Serial.print(" ");
                    Serial.print(mTxSendMs[0]);
                    Serial.print("ms ");
                dumpBuf(mTxBuf[0], 27);
            }
            Serial.println();

            if(0 == gotRx) {
                mNrf24.setChannel(61);
                mNrf24.startListening();
             }

            mRetransmit = false;
        }

        else if(0 == gotRx) {
            uint32_t startMicros = micros();
            while ((micros() - startMicros) < 5110) {
                yield();
            }
            mRxCh = (mRxCh + 1) % 5;
            mNrf24.setChannel(mChList[mRxCh]);
        }

        //Serial.println("---------------------------- " + String(counter++));
    }
}
#endif