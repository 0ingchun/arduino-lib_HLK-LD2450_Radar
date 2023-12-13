#ifndef LD2450_h
#define LD2450_h

#include "Arduino.h"

struct RadarTarget {
    int16_t x;      // X坐标，单位mm
    int16_t y;      // Y坐标，单位mm
    int16_t speed;  // 速度，单位cm/s
    uint16_t resolution; // 距离分辨率，单位mm
};

class LD2450 {
    public:
        LD2450(HardwareSerial &serialPort, long baudRate = 256000);
        void begin();
        bool enableConfiguration();
        bool disableConfiguration();
        bool setSingleTargetTracking();
        bool setMultiTargetTracking();
        bool readFirmwareVersion();
        bool setBaudRate(uint16_t baudRateIndex);
        bool restoreFactorySettings();
        bool rebootModule();
        bool setBluetooth(bool enabled);
        bool readMacAddress();
        bool readCurrentAreaFilter();
        bool setAreaFilter(byte filterConfig[], uint16_t filterConfigLength);
        bool readAck(uint16_t commandWord, byte ackValue[], uint16_t &ackValueLength);
        bool readRadarData(RadarTarget targets[], uint16_t &numTargets);
        bool executeConfigurationCommands(uint16_t commandWords[], byte* commandValues[], uint16_t commandValueLengths[], uint16_t numCommands);

    private:
        HardwareSerial *_serialPort;
        long _baudRate;
        void sendCommand(uint16_t commandWord, byte commandValue[], uint16_t commandValueLength);
        bool enableConfiguration();
        bool disableConfiguration();
        bool readFrame(byte buffer[], int &length);
};

#endif
