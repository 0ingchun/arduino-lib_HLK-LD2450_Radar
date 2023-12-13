#include <LD2450.h>

HardwareSerial mySerial(1);
LD2450 radar(mySerial);

void setup() {
    Serial.begin(256000);
    radar.begin();
}

void loop() {
    RadarTarget targets[10];
    uint16_t numTargets = 0;

    if (radar.readRadarData(targets, numTargets)) {
        Serial.print("检测到目标数量: ");
        Serial.println(numTargets);

        for (int i = 0; i < numTargets; i++) {
            Serial.print("目标 ");
            Serial.print(i + 1);
            Serial.print(": X = ");
            Serial.print(targets[i].x);
            Serial.print("mm, Y = ");
            Serial.print(targets[i].y);
            Serial.print("mm, 速度 = ");
            Serial.print(targets[i].speed);
            Serial.println("cm/s");
        }
    }

     delay(500); // 延迟一秒再次读取
}
