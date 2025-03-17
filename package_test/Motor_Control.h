#include "Arduino.h"
#include <FlexCAN_T4.h>
#include <vector>

class Motor_CT
{
public:
  Motor_CT();
  void Set_value(int _id,int _reduction_ratio,char _foot);
  void Serial_Init();
  void SendPosCommand(float theta); //傳送馬達運動指令
//  void SendSpeedCommand(float theta); //傳送馬達速度控制指令
  void SendStopCommand(); //傳送馬達運動指令
  void SetSpeedCommand(float _speed); //設定馬達速度指令
  void SaveSetting();
  
  static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_R;
  static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_L;

private:
  void SendCommand(CAN_message_t msg);
  int id = 0;
  int reduction_ratio = 0;
  int encoder_parm = 65535;
  char foot='R';
};

