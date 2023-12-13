#include "LD2450.h"

// 构造函数：初始化LD2450类的新实例
LD2450::LD2450(HardwareSerial &serialPort, long baudRate) {
    _serialPort = &serialPort;
    _baudRate = baudRate;
}

// 初始化函数：设置串口波特率
void LD2450::begin() {
    _serialPort->begin(_baudRate);
}

// 读取数据：从雷达读取数据并处理
void LD2450::readData() {
    if (_serialPort->available()) {
        byte buffer[256];
        int length = _serialPort->readBytes(buffer, sizeof(buffer));
        processData(buffer, length);
    }
}

// 使能配置：发送使能配置命令到雷达
bool LD2450::enableConfiguration() {
    byte commandValue[] = {0x01, 0x00};
    sendCommand(0x00FF, commandValue, sizeof(commandValue));
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00FF, ackValue, ackValueLength);
}

// 禁用配置：发送结束配置命令到雷达
bool LD2450::disableConfiguration() {
    sendCommand(0x00FE, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00FE, ackValue, ackValueLength);
}

// 设置单目标追踪：发送设置单目标追踪命令到雷达
bool LD2450::setSingleTargetTracking() {
    sendCommand(0x0080, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x0080, ackValue, ackValueLength);
}

// 设置多目标追踪：发送设置多目标追踪命令到雷达
bool LD2450::setMultiTargetTracking() {
    sendCommand(0x0090, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x0090, ackValue, ackValueLength);
}

// 读取固件版本：发送读取固件版本命令到雷达
bool LD2450::readFirmwareVersion() {
    sendCommand(0x00A0, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A0, ackValue, ackValueLength);
}

// 设置波特率：发送设置波特率命令到雷达
bool LD2450::setBaudRate(uint16_t baudRateIndex) {
    byte commandValue[] = {(byte)(baudRateIndex & 0xFF), (byte)((baudRateIndex >> 8) & 0xFF)};
    sendCommand(0x00A1, commandValue, sizeof(commandValue));
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A1, ackValue, ackValueLength);
}

// 恢复出厂设置：发送恢复出厂设置命令到雷达
bool LD2450::restoreFactorySettings() {
    sendCommand(0x00A2, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A2, ackValue, ackValueLength);
}

// 重启模块：发送重启模块命令到雷达
bool LD2450::rebootModule() {
    sendCommand(0x00A3, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A3, ackValue, ackValueLength);
}

// 设置蓝牙：发送设置蓝牙开关命令到雷达
bool LD2450::setBluetooth(bool enabled) {
    byte commandValue[] = {enabled ? 0x01 : 0x00, 0x00};
    sendCommand(0x00A4, commandValue, sizeof(commandValue));
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A4, ackValue, ackValueLength);
}

// 读取MAC地址：发送读取MAC地址命令到雷达
bool LD2450::readMacAddress() {
    byte commandValue[] = {0x01, 0x00};
    sendCommand(0x00A5, commandValue, sizeof(commandValue));
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00A5, ackValue, ackValueLength);
}

// 读取当前区域过滤配置：发送读取区域过滤配置命令到雷达
bool LD2450::readCurrentAreaFilter() {
    sendCommand(0x00C1, nullptr, 0);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00C1, ackValue, ackValueLength);
}

// 设置区域过滤配置：发送设置区域过滤配置命令到雷达
bool LD2450::setAreaFilter(byte filterConfig[], uint16_t filterConfigLength) {
    sendCommand(0x00C2, filterConfig, filterConfigLength);
    byte ackValue[256];
    uint16_t ackValueLength = 0;
    return readAck(0x00C2, ackValue, ackValueLength);
}

// 读取雷达上报的数据，并解析每个目标的信息
// 参数: 
// targets[] - 存储解析后的目标数据
// numTargets - 返回检测到的目标数量
// 返回值: 
// true - 成功读取并解析数据
// false - 读取或解析失败
bool LD2450::readRadarData(RadarTarget targets[], uint16_t &numTargets) {
    if (_serialPort->available()) {
        byte buffer[256];
        int length = _serialPort->readBytes(buffer, sizeof(buffer));
        
        // 检查帧头，确保数据有效
        if (length > 4 && buffer[0] == 0xAA && buffer[1] == 0xFF) {
            int index = 4; // 跳过帧头和帧内数据长度字段
            numTargets = 0;
            // 循环处理每个目标的数据，直到遇到帧尾
            while (index + 8 <= length && buffer[index] != 0x55) {
                targets[numTargets].x = (int16_t)(buffer[index] | (buffer[index + 1] << 8));
                targets[numTargets].y = (int16_t)(buffer[index + 2] | (buffer[index + 3] << 8));
                targets[numTargets].speed = (int16_t)(buffer[index + 4] | (buffer[index + 5] << 8));
                targets[numTargets].resolution = (uint16_t)(buffer[index + 6] | (buffer[index + 7] << 8));
                index += 8;
                numTargets++;
            }
            return true;
        }
    }
    return false;
}


// 执行配置命令序列
// 参数: 
// commandWords[] - 命令字数组
// commandValues[] - 对应命令的值数组
// commandValueLengths[] - 命令值长度数组
// numCommands - 命令数量
// 返回值: 
// true - 所有命令执行成功
// false - 任一命令执行失败
bool LD2450::executeConfigurationCommands(uint16_t commandWords[], byte* commandValues[], uint16_t commandValueLengths[], uint16_t numCommands) {
    // 首先发送使能配置命令
    if (!enableConfiguration()) {
        return false;
    }

    // 循环发送命令并检查ACK
    for (uint16_t i = 0; i < numCommands; i++) {
        sendCommand(commandWords[i], commandValues[i], commandValueLengths[i]);
        byte ackValue[256];
        uint16_t ackValueLength = 0;
        if (!readAck(commandWords[i], ackValue, ackValueLength)) {
            disableConfiguration();
            return false;
        }
    }

    // 发送结束配置命令
    if (!disableConfiguration()) {
        return false;
    }

    return true;
}

// 发送命令到雷达
// 参数: 
// commandWord - 命令字
// commandValue - 命令值
// commandValueLength - 命令值长度
void LD2450::sendCommand(uint16_t commandWord, byte commandValue[], uint16_t commandValueLength) {
    sendFrame(commandWord, commandValue, commandValueLength);
}


// 读取一帧数据
// 参数: 
// buffer[] - 存储读取的数据
// length - 数据长度
// 返回值: 
// true - 成功读取一帧数据
// false - 读取失败
bool LD2450::readFrame(byte buffer[], int &length) {
    unsigned long startTime = millis();
    length = 0;  // 初始化length为0，表示还没有读取任何数据
    while (millis() - startTime < 1000) { // 等待最多1秒
        if (_serialPort->available()) {
            buffer[length++] = (byte)_serialPort->read();
            // 检测帧尾
            if (length >= 4 && buffer[length - 4] == 0x04 && buffer[length - 3] == 0x03 && 
                buffer[length - 2] == 0x02 && buffer[length - 1] == 0x01) {
                return true;
            }
        }
    }
    return false; // 超时
}


// 读取并解析ACK
// 参数: 
// commandWord - 命令字，用于确认收到的ACK与发送的命令相符
// ackValue[] - 存储解析后的ACK值
// ackValueLength - ACK值长度
// 返回值: 
// true - 成功读取并解析ACK
// false - 读取或解析失败
// bool LD2450::readAck(uint16_t commandWord, byte ackValue[], uint16_t &ackValueLength) {
//     byte buffer[256];
//     int length = 0;
//     if (readFrame(buffer, length)) {
//         // 解析ACK帧
//         // 确认帧头和帧尾
//         // 解析返回的命令字和返回值
//         return true;
//     }
//     return false;
// }
bool LD2450::readAck(uint16_t commandWord, byte ackValue[], uint16_t &ackValueLength) {
    byte buffer[256];
    int length = 0;
    if (readFrame(buffer, length)) {
        // 确认帧头和帧尾，解析ACK帧
        if (length > 10 && buffer[0] == 0xFD && buffer[1] == 0xFC && buffer[2] == 0xFB && buffer[3] == 0xFA) {
            uint16_t receivedCommandWord = (uint16_t)(buffer[6] | (buffer[7] << 8));
            if (receivedCommandWord == commandWord) {
                ackValueLength = (uint16_t)(buffer[4] | (buffer[5] << 8)) - 2; // 去除命令字长度
                for (int i = 0; i < ackValueLength; i++) {
                    ackValue[i] = buffer[8 + i];
                }
                return true;
            }
        }
    }
    return false;
}


// 发送数据帧到雷达
// 参数: 
// commandWord - 命令字
// commandValue[] - 命令值
// commandValueLength - 命令值长度
void LD2450::sendFrame(uint16_t commandWord, byte commandValue[], uint16_t commandValueLength) {
    // 发送帧头
    _serialPort->write(0xFD);
    _serialPort->write(0xFC);
    _serialPort->write(0xFB);
    _serialPort->write(0xFA);

    // 帧内数据长度
    uint16_t dataLength = 2 + commandValueLength;
    _serialPort->write((byte)(dataLength & 0xFF));
    _serialPort->write((byte)((dataLength >> 8) & 0xFF));

    // 命令字
    _serialPort->write((byte)(commandWord & 0xFF));
    _serialPort->write((byte)((commandWord >> 8) & 0xFF));

    // 命令值
    for (uint16_t i = 0; i < commandValueLength; i++) {
        _serialPort->write(commandValue[i]);
    }

    // 帧尾
    _serialPort->write(0x04);
    _serialPort->write(0x03);
    _serialPort->write(0x02);
    _serialPort->write(0x01);
}
