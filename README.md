# arduino-lib_HLK-LD2450_Radar
arduino communication driver for Hilink HLK-LD2450 human body has inductive radar

通过hlk-2450人体轨迹雷达实现开放通道往来人流检测，开机从eeprom拿取上次数据继续叠加，hlk-2450雷达接在串口3，测得数据定时存入eeprom，串口0发送FAC恢复出厂设置

mega2560 + hlk-2450