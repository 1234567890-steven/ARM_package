#include "HumanoidCommunication.h"

HumanoidCommunication::HumanoidCommunication(int port) {
    //初始化
    ArmVel = 0;
    MotorEnable = false;
    _port = port;
    Serial.println("Humanoid Communication Initialized...");
    if (port == 0) {
        _serial = &Serial;
    }
    else if (port == 1) {
        _serial = &Serial1;
    } 
    else if (port == 2) {
        _serial = &Serial2;
    }
    else if (port == 3) {
        _serial = &Serial3;
        Serial3.transmitterEnable(13); 
    }
    else if (port == 4) {
        _serial = &Serial4;
    }
    else if (port == 5) {
        _serial = &Serial5;
        Serial5.transmitterEnable(2);
    }
    else if (port == 6) {
        _serial = &Serial6;
    }
    else {
        _serial = nullptr;
        Serial.println("Invalid Serial Port, Please Check Agian.");
    }
}

void HumanoidCommunication::SerialInit() {
    if (_serial != nullptr) {
        switch (_port) {
            case 0:
                Serial.begin(115200);
                Serial.println("Serial Port 0 Initialized...");
                break;
            case 1:
                Serial1.begin(115200);
                Serial.println("Serial Port 1 Initialized...");
                break;
            case 2:
                Serial2.begin(115200);
                Serial.println("Serial Port 2 Initialized...");
                break;
            case 3:
                Serial3.begin(115200);
                Serial.println("Serial Port 3 Initialized...");
                break;
            case 4:
                Serial4.begin(115200);
                Serial.println("Serial Port 4 Initialized...");
                break;
            case 5:
                Serial5.begin(115200);
                Serial.println("Serial Port 5 Initialized...");
                break;  
            case 6:
                Serial6.begin(115200);
                Serial.println("Serial Port 6 Initialized...");
                break;
            default:
                break;
        }
    }
}

void HumanoidCommunication::_Receive() {
    byte _rec[1];
    
    while (_serial -> available() > 0) {
        _serial -> readBytes(_rec, 1);
        Serial.println(_rec[0], HEX);
        if (_rec[0] == 0x3E && !Listening) {
            Listening = 1;
            cnt = 0;
            Serial.println("Header correct.");
        }
        if (Listening == 1)
        {   // Check the Target of this command
            if (cnt == 1) { 
                if (_rec[0] == 0x01) {
                    target = 1;
                    Serial.println("Arm");
                    
                } 
                else if (_rec[0] == 0x02) {
                    target = 2;
                    Serial.println("Hand");
                } 
                else if (_rec[0] == 0x03) {
                    target = 3;
                    Serial.println("Leg");
                }
                else {
                    target = 0;
                    Serial.println("Invalid Target. Packet Discarded...");
                    Serial.println("Target: ");
                    Serial.println(_rec[0]);
                    Listening = 0;
                }
            }
            // Check the Function of this command
            if (cnt == 2) {
                if (_rec[0] == 0x01) {
                    func = 1;
                } 
                else if (_rec[0] == 0x02) {
                    func = 2;
                }
                else if (_rec[0] == 0x03) {
                    func = 3;
                }
                else if (_rec[0] == 0x04) {
                    func = 4;
                }
                else if (_rec[0] == 0x05) {
                    func = 5;
                }
                else if (_rec[0] == 0x06) {
                    func = 6;
                }
                else {
                    func = 0;
                    Serial.println("Invalid Function. Packet Discarded...");
                    Listening = 0;
                }
            }
            // Check the _data Length of this command
            if (cnt == 3) {
                length = _rec[0];
                Serial.print("Length: ");
                Serial.println(length);
            }
            // Send the _data to a specific function
            // if (cnt == 4) {
            //     if (length == 0) {
            //         _UnpackData(0, 0, target, func);
            //         Listening = 0;
            //         Serial.println("Unpack Data...  State instruction");
            //     }
            //     else {
            //         for (int i = 0; i <= length; i++) {
            //             _data[i] = _rec[i + cnt]; //包含checksum
            //         }
            //         _UnpackData(_data, length, target, func);
            //         Serial.println("Unpack Data...  Command instruction");
            //         Listening = 0;
            //     }
            // }
            
            if (cnt > 3 && cnt <= 4 + length) {
                if (length == 0) {
                    _UnpackData(0, 0, target, func);
                    Listening = 0;
                    Serial.println("Unpack Data...  State instruction");
                }
                else {
                    _data[cnt - 4] = _rec[0];
                    if (cnt == 4 + length) {
                        _UnpackData(_data, length, target, func);
                        for (int i = 0; i < length+1; i++) {
                            Serial.print(_data[i]);
                            Serial.print(" ");
                        }
                        Serial.println("Unpack Data...  Command instruction");
                        Listening = 0;
                    }
                }
            }
            
            cnt++;   
        } 
    }
}

void HumanoidCommunication::_UnpackData(byte _rec[], int len, int _target, int _func) {
    // 狀態回覆或是姿態復位
    if (len == 0) {
        Serial.println("State instruction");
        if (_target == 1) { //手臂
            if (_func == 6) { // 馬達enable
                MotorEnable = true;
                Serial.println("Motor Enable...");
            }
        }
    }
    else if (len > 0) 
    {   
        // 先檢查checksum
        uint16_t crc = CheckSum(_rec, len - 1);
        Serial.print("Checksum: ");
        Serial.println(crc);
        uint16_t crc_receive = _rec[len];
        if (crc != crc_receive) {
            Serial.println("Checksum Error. Packet Discarded...");
            return;
        }
        // 以下均為命令指令
        if (_target == 1) { //手臂
            if (_func == 2 && len == 48) { //角度命令
                ArmAngle[0][0] = (_rec[3] << 24 | _rec[2] << 16 | _rec[1] << 8 | _rec[0]) / 100.0;
                ArmAngle[0][1] = (_rec[7] << 24 | _rec[6] << 16 | _rec[5] << 8 | _rec[4]) / 100.0;
                ArmAngle[0][2] = (_rec[11] << 24 | _rec[10] << 16 | _rec[9] << 8 | _rec[8]) / 100.0;
                ArmAngle[0][3] = (_rec[15] << 24 | _rec[14] << 16 | _rec[13] << 8 | _rec[12]) / 100.0;
                ArmAngle[0][4] = (_rec[19] << 24 | _rec[18] << 16 | _rec[17] << 8 | _rec[16]) / 100.0;
                ArmAngle[0][5] = (_rec[23] << 24 | _rec[22] << 16 | _rec[21] << 8 | _rec[20]) / 100.0;
                ArmAngle[1][0] = (_rec[27] << 24 | _rec[26] << 16 | _rec[25] << 8 | _rec[24]) / 100.0;
                ArmAngle[1][1] = (_rec[31] << 24 | _rec[30] << 16 | _rec[29] << 8 | _rec[28]) / 100.0;
                ArmAngle[1][2] = (_rec[35] << 24 | _rec[34] << 16 | _rec[33] << 8 | _rec[32]) / 100.0;
                ArmAngle[1][3] = (_rec[39] << 24 | _rec[38] << 16 | _rec[37] << 8 | _rec[36]) / 100.0;
                ArmAngle[1][4] = (_rec[43] << 24 | _rec[42] << 16 | _rec[41] << 8 | _rec[40]) / 100.0;
                ArmAngle[1][5] = (_rec[47] << 24 | _rec[46] << 16 | _rec[45] << 8 | _rec[44]) / 100.0;
                Serial.println("Arm Angle Command Set... Done");
                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 6; j++) {
                        Serial.print(ArmAngle[i][j]);
                        Serial.print(" ");
                    }
                    Serial.println();
                }
            }
            else if (_func == 5 && len == 2) {
                ArmVel = _rec[1] << 8 | _rec[0];
                Serial.println("Arm Velocity Set... Done");
            }
            else if (_func == 6 && len == 1) {
                if (_rec[0] == 0x00) {
                    RetuenPermission = false;
                    Serial.println("Return Permission Set False... Done");
                }
                else if (_rec[0] == 0x01) {
                    RetuenPermission = true;
                    Serial.println("Return Permission Set True... Done");
                }
                else {
                    Serial.println("Invalid Return Permission. Packet Discarded...");
                }
            }
        }
    }
    else {
        Serial.println("Invalid length. Packet Discarded...");
    }
}

uint16_t HumanoidCommunication::CheckSum(byte _rec[], int len) {
    uint16_t crc = 0x00;
    for (int i = 0; i < length; i++) {
      crc += _rec[i];
    }
    crc = crc & 0xFF;
    return crc;  // 傳回計算後的 CRC
}