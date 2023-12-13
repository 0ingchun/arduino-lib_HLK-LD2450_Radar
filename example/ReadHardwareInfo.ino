#include <LD2450.h>

HardwareSerial mySerial(1); // 假设雷达连接在串口1
LD2450 radar(mySerial);

void setup() {
    Serial.begin(256000);
    radar.begin(); // 初始化雷达

    // 读取固件版本
    if (radar.readFirmwareVersion()) {
        Serial.println("固件版本读取成功");
    } else {
        Serial.println("固件版本读取失败");
    }
}

void loop() {
    // 循环体可以保持空白，如果不需要持续执行任何操作
}
