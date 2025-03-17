#pragma once
#include <Arduino.h>
#include <vector>

#define Baudrate_init 115200

class HumanoidCommunication
{
    public:
        HumanoidCommunication(int port);
        void SerialInit();
        void _Receive(); //Unpack data from serial port
        std::vector<std::vector<double>> ArmAngle = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; //Right Arm, Left Arm, 紀錄左右手臂的角度命令
        int ArmVel = 0; //紀錄手臂馬達的速度命令        
        bool MotorEnable = false; //馬達Enable, 預設為false
        bool RetuenPermission = false; //回傳權限, 預設為false

    private:
        Stream* _serial = &Serial;
        int _port = 0;
        int length = 0;
        int target = 0;
        int func = 0;
        int Listening = 0;
        int cnt = 0;
        byte _data[80] = {0};
        void _UnpackData(byte _rec[], int len, int _target, int _func); //Unpack data from serial port
        uint16_t CheckSum(byte _rec[], int len); //Check the checksum of the received data

        
};