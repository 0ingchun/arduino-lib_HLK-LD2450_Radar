

#include <Arduino.h>

#include <avr/wdt.h>
#include <EEPROM.h>

#define ledPin 13

typedef struct RadarTarget
{
    uint16_t id;  // 目标ID
    int16_t x;           // X坐标，单位mm
    int16_t y;           // Y坐标，单位mm
    int16_t speed;       // 速度，单位cm/s
    uint16_t resolution; // 距离分辨率，单位mm
} RadarTarget_t;

RadarTarget_t nowTargets[3]; // 存储当前帧的目标
RadarTarget_t lastTargets[3]; // 存储上一帧的目标

int countLeftToRight = 0; // 从左到右的人数
int countRightToLeft = 0; // 从右到左的人数

int countTime = 0; // 统计时间，sec

int16_t v_th = 17; // 行人经过速度阈值，单位cm/s

// 定义全局变量以跟踪上次更新时间
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 0.5 * 60 * 1000; // 3分钟 = 180000毫秒

int leftToRightAddress = 0;  // 为从左到右的人流计数选择一个eeprom地址
int rightToLeftAddress = 2;  // 为从右到左的人流计数选择一个eeprom地址
int countTimeAddress = 4;  // 为上次更新时间选择一个eeprom地址


void saveCountToEEPROM(int address, int count) {  // 保存人流计数到EEPROM
    int storedCount = EEPROM.read(address) | (EEPROM.read(address + 1) << 8);
    if (storedCount != count) {
        EEPROM.write(address, count & 0xFF);
        EEPROM.write(address + 1, (count >> 8) & 0xFF);
    }
}


int readCountFromEEPROM(int address) {  // 从EEPROM中读取人流计数
    int storedCount = EEPROM.read(address) | (EEPROM.read(address + 1) << 8);
    return storedCount;
}


void clearEEPROM() {  // 清除EEPROM
    // for (int i = 0; i < EEPROM.length(); i++) {
    //     EEPROM.write(i, 0); // 将每个地址的EEPROM数据清除（设置为0）
    // }
    for (int i = 0; i < 7; i++) {
        EEPROM.write(i, 0); // 仅删除0123456地址，延长寿命，将每个地址的EEPROM数据清除（设置为0）
    }
}


void softwareReset() {
  wdt_enable(WDTO_15MS); // 开启看门狗定时器，设置超时为15毫秒
  while (1) {
    // 无限循环，等待看门狗重置微控制器
  }
}


/**
* Reads radar data and populates an array of RadarTarget objects.
*
* @param rec_buf The buffer that contains the radar data.
* @param len The length of the radar data buffer.
* @param targets An array of RadarTarget objects to store the radar targets.
* @param numTargets The number of radar targets to read.
*
* @return Returns 1 on successful reading of radar data, or 0 if the data is incomplete.
*
* @throws None
**/
int readRadarData(byte rec_buf[], int len, RadarTarget targets[], uint16_t numTargets) {
  // if (radar_serial.available() > 0) {
  //     byte rec_buf[256] = "";
  //     int len = radar_serial.readBytes(rec_buf, sizeof(rec_buf));

      for (int i = 0; i < len; i++) {
          if (rec_buf[i] == 0xAA && rec_buf[i + 1] == 0xFF && rec_buf[i + 2] == 0x03 && rec_buf[i + 3] == 0x00 && rec_buf[i + 28] == 0x55 && rec_buf[i + 29] == 0xCC) { // 检查帧头和帧尾
              String targetInfo = ""; // 存储目标信息的字符串
              int index = i + 4; // 跳过帧头和帧内数据长度字段

              for (int targetCounter = 0; targetCounter < numTargets; targetCounter++) {
                  if (index + 7 < len) {
                      RadarTarget target;
                      target.x = (int16_t)(rec_buf[index] | (rec_buf[index + 1] << 8));
                      target.y = (int16_t)(rec_buf[index + 2] | (rec_buf[index + 3] << 8));
                      target.speed = (int16_t)(rec_buf[index + 4] | (rec_buf[index + 5] << 8));
                      target.resolution = (uint16_t)(rec_buf[index + 6] | (rec_buf[index + 7] << 8));

                      // debug_serial.println(target.x);
                      // debug_serial.println(target.y);
                      // debug_serial.println(target.speed);

                      // 检查x和y的最高位，调整符号
                      if (rec_buf[index + 1] & 0x80) target.x -= 0x8000;
                      else target.x = -target.x;
                      if (rec_buf[index + 3] & 0x80) target.y -= 0x8000;
                      else target.y = -target.y;
                      if (rec_buf[index + 5] & 0x80) target.speed -= 0x8000;
                      else target.speed = -target.speed;

                      // 赋值目标信息
                      // debug_serial.println(targetCounter + 1);
                      targets[targetCounter].id = targetCounter + 1;
                      targets[targetCounter].x = target.x;
                      targets[targetCounter].y = target.y;
                      targets[targetCounter].speed = target.speed;
                      targets[targetCounter].resolution = target.resolution;

                      // // 输出目标信息
                      // debug_serial.print("目标 ");
                      // debug_serial.print(++targetCounter); // 增加目标计数器并输出
                      // debug_serial.print(": X: ");
                      // debug_serial.print(target.x);
                      // debug_serial.print("mm, Y: ");
                      // debug_serial.print(target.y);
                      // debug_serial.print("mm, 速度: ");
                      // debug_serial.print(target.speed);
                      // debug_serial.print("cm/s, 距离分辨率: ");
                      // debug_serial.print(target.resolution);
                      // debug_serial.print("mm; ");

                      // 添加目标信息到字符串
                      targetInfo += "目标 " + String(targetCounter + 1) + ": X: " + target.x + "mm, Y: " + target.y + "mm, 速度: " + target.speed + "cm/s, 距离分辨率: " + target.resolution + "mm; ";

                      index += 8; // 移动到下一个目标数据的开始位置
                  }
              }

              // 输出目标信息
              // debug_serial.println(targetInfo);

              // 输出原始数据
              // debug_serial.print("原始数据: ");
              for (int j = i; j < i + 30; j++) {
                  if (j < len) {
                      // debug_serial.print(rec_buf[j], HEX);
                      // debug_serial.print(" ");
                  }
              }
              // debug_serial.println("\n"); // 换行，准备输出下一帧数据

              i = index; // 更新外部循环的索引
          }
          // else return 0; // 数据不完整，返回-1
      }
      return 1;
  // }
  // else return 0;  // 串口缓冲区为空，返回0
}


void setup() {
  // 初始化led
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Serial.begin(256000);    // 用于PC通信的串口
  Serial2.begin(256000); // 与雷达模块通信的串口，波特率256000
  delay(10);

  Serial.println("人流统计系统。。。启动");

  // 读取数据，初始化为上次存储的人流计数
  Serial.println("从EEPROM读取的数据: ");
  Serial.print("从左到右的人流计数: ");  
  Serial.println(readCountFromEEPROM(leftToRightAddress));
  Serial.print("从右到左的人流计数: ");
  Serial.println(readCountFromEEPROM(rightToLeftAddress));
  Serial.print("统计时间秒: ");
  Serial.println(readCountFromEEPROM(countTimeAddress));

  countLeftToRight = readCountFromEEPROM(leftToRightAddress);
  countRightToLeft = readCountFromEEPROM(rightToLeftAddress);
  countTime = readCountFromEEPROM(countTimeAddress);

  // 设置看门狗
  MCUSR = 0;
  wdt_disable();

}


void loop() {
  // digitalWrite(ledPin, LOW);
  Serial.println();
  Serial.print("系统时间: ");
  Serial.println(millis());

  if (Serial2.available() > 0) {
    byte rec_buf[256] = "";
    int len = Serial2.readBytes(rec_buf, sizeof(rec_buf));

    int radar_flag = readRadarData(rec_buf, len, nowTargets, 3);
    Serial.println(radar_flag);

    if (radar_flag == 1) {
      digitalWrite(ledPin, HIGH);
      for (int i = 0; i < 3; i++) {  // 输出雷达数据读取
        Serial.print("目标 ");
        Serial.print(nowTargets[i].id);
        Serial.print(": X = ");
        Serial.print(nowTargets[i].x);
        Serial.print("mm, Y = ");
        Serial.print(nowTargets[i].y);
        Serial.print("mm, 速度 = ");
        Serial.print(nowTargets[i].speed);
        Serial.print("cm/s, 距离分辨率 = ");
        Serial.print(nowTargets[i].resolution);
        Serial.print("mm; \n");
      }

      // 统计从左到右和从右到左的人流
      for (int i = 0; i < 3; i++) { // 检查每个目标
        if (nowTargets[i].y != 0) { // 假设y坐标为0表示目标不存在
            // 检查目标是否从左到右移动
            if (abs(nowTargets[i].speed) >= v_th && abs(lastTargets[i].speed) >= v_th  && nowTargets[i].x < lastTargets[i].x) {
                countLeftToRight++;
                // countLeftToRight+=10;
            }
            // 检查目标是否从右到左移动
            else if (abs(nowTargets[i].speed) >= v_th && abs(lastTargets[i].speed) >= v_th  && nowTargets[i].x > lastTargets[i].x) {
                countRightToLeft++;
                // countRightToLeft+=10;
            }
        }
        lastTargets[i] = nowTargets[i]; // 更新上一帧的目标信息
      }    

      // 打印统计结果
      Serial.print("从左到右的人数: ");
      Serial.println(countLeftToRight);
      Serial.print("从右到左的人数: ");
      Serial.println(countRightToLeft);

      // 检查是否已经过去了3分钟
      Serial.print("间隔时间: ");
      Serial.println(millis() - lastUpdateTime);
      
      if ((millis() - lastUpdateTime) >= updateInterval) {
        countTime = countTime + (int)(millis() - lastUpdateTime)/1000;
        Serial.println(countTime);
        // 更新EEPROM中的数据
        Serial.println("更新EEPROM中的数据");
        if (countLeftToRight != readCountFromEEPROM(leftToRightAddress)) saveCountToEEPROM(leftToRightAddress, countLeftToRight);
        if (countRightToLeft != readCountFromEEPROM(rightToLeftAddress)) saveCountToEEPROM(rightToLeftAddress, countRightToLeft);
        if (countTime != readCountFromEEPROM(countTimeAddress)) saveCountToEEPROM(countTimeAddress, countTime);

        // 更新上次更新时间
        lastUpdateTime = millis();

        // 读取数据
        Serial.println("从EEPROM读取的数据: ");
        Serial.print("从左到右的人流计数: ");  
        Serial.println(readCountFromEEPROM(leftToRightAddress));
        Serial.print("从右到左的人流计数: ");
        Serial.println(readCountFromEEPROM(rightToLeftAddress));
        Serial.print("统计时间秒: ");
        Serial.println(readCountFromEEPROM(countTimeAddress));
      }
    }


  }
  else {
    digitalWrite(ledPin, LOW);
    delay(1);
  }

  if (Serial.available() > 0) { // 恢复出厂设置
        digitalWrite(ledPin, HIGH);
        // 如果串口有数据可读，读取字符串直到遇到换行符
        String command = Serial.readStringUntil('\n');

        // 检查收到的命令是否是"FAC"
        if (command == "FAC") {
            clearEEPROM();
            Serial.println();
            Serial.println("EEPROM已清除");
            Serial.println("重启。。。");
            softwareReset();  // 重启Arduino
        }
        else {
            Serial.println();
            Serial.println("无效命令");
            Serial.println("发送 FAC 恢复出厂设置");
            digitalWrite(ledPin, LOW);
        }
    }

    else digitalWrite(ledPin, LOW);


    // if (Serial1.available() > 0) {
    //     byte rec_buf[256] = "";
    //     int len = Serial1.readBytes(rec_buf, sizeof(rec_buf));

    //     for (int i = 0; i < len; i++) {
    //         if (rec_buf[i] == 0xAA && rec_buf[i + 1] == 0xFF && rec_buf[i + 2] == 0x03 && rec_buf[i + 3] == 0x00 && rec_buf[i + 28] == 0x55 && rec_buf[i + 29] == 0xCC) {
    //             String targetInfo = ""; // 存储目标信息的字符串
    //             int index = i + 4; // 跳过帧头和帧内数据长度字段

    //             for (int targetCounter = 0; targetCounter < 3; targetCounter++) {
    //                 if (index + 7 < len) {
    //                     RadarTarget target;
    //                     target.x = (int16_t)(rec_buf[index] | (rec_buf[index + 1] << 8));
    //                     target.y = (int16_t)(rec_buf[index + 2] | (rec_buf[index + 3] << 8));
    //                     target.speed = (int16_t)(rec_buf[index + 4] | (rec_buf[index + 5] << 8));
    //                     target.resolution = (uint16_t)(rec_buf[index + 6] | (rec_buf[index + 7] << 8));

    //                     // 检查x和y的最高位，调整符号
    //                     Serial.println(target.x);
    //                     Serial.println(target.y);
    //                     Serial.println(target.speed);
    //                     if (rec_buf[index + 1] & 0x80) target.x -= 0x8000;
    //                     else target.x = -target.x;
    //                     if (rec_buf[index + 3] & 0x80) target.y -= 0x8000;
    //                     else target.y = -target.y;
    //                     if (rec_buf[index + 5] & 0x80) target.speed -= 0x8000;
    //                     else target.speed = -target.speed;

    //                     // // 输出目标信息
    //                     // Serial.print("目标 ");
    //                     // Serial.print(++targetCounter); // 增加目标计数器并输出
    //                     // Serial.print(": X: ");
    //                     // Serial.print(target.x);
    //                     // Serial.print("mm, Y: ");
    //                     // Serial.print(target.y);
    //                     // Serial.print("mm, 速度: ");
    //                     // Serial.print(target.speed);
    //                     // Serial.print("cm/s, 距离分辨率: ");
    //                     // Serial.print(target.resolution);

    //                     // 添加目标信息到字符串
    //                     targetInfo += "目标 " + String(targetCounter + 1) + ": X: " + target.x + "mm, Y: " + target.y + "mm, 速度: " + target.speed + "cm/s, 距离分辨率: " + target.resolution + "mm; ";

    //                     index += 8; // 移动到下一个目标数据的开始位置
    //                 }
    //             }

    //             // 输出目标信息
    //             Serial.println(targetInfo);

    //             // 输出原始数据
    //             Serial.print("原始数据: ");
    //             for (int j = i; j < i + 30; j++) {
    //                 if (j < len) {
    //                     Serial.print(rec_buf[j], HEX);
    //                     Serial.print(" ");
    //                 }
    //             }
    //             Serial.println("\n"); // 换行，准备输出下一帧数据

    //             i = index; // 更新外部循环的索引
    //         }
    //     }
    // }


}
