#include "Arduino.h"
#include "Motor_Control.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Motor_CT::can_R;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Motor_CT::can_L;

Motor_CT::Motor_CT(){

}

void Motor_CT::Set_value(int _id,int _reduction_ratio,char _foot){
  id=_id;
  reduction_ratio=_reduction_ratio;
  foot=_foot;
}

void Motor_CT::Serial_Init(){
  can_R.begin();
  can_R.setBaudRate(1000000);
  can_R.setMaxMB(16);
  can_R.enableFIFO();
  can_R.enableFIFOInterrupt();

  can_L.begin();
  can_L.setBaudRate(1000000);
  can_L.setMaxMB(16);
  can_L.enableFIFO();
  can_L.enableFIFOInterrupt();
}

void Motor_CT::SendPosCommand(float theta) {
  int32_t pos=theta/360.0*reduction_ratio*encoder_parm;
  CAN_message_t msg;
  msg.id = (byte)id;
  msg.len = 5;
  msg.buf[0] = 0x1E;
  msg.buf[1] = (pos) & 0xFF;
  msg.buf[2] = (pos >> 8) & 0xFF;
  msg.buf[3] = (pos >> 16) & 0xFF;
  msg.buf[4] = (pos >> 24) & 0xFF;
  SendCommand(msg);
}


void Motor_CT::SendStopCommand(){
  CAN_message_t msg;
  msg.id = (byte)id;
  msg.len = 1;
  msg.buf[0] = 0x02;
  SendCommand(msg);
}

void Motor_CT::SetSpeedCommand(float _speed){
  int32_t M_speed=_speed/360.0*reduction_ratio*100;
  
  CAN_message_t msg;
  msg.id = (byte)id;
  msg.len = 5;
  msg.buf[0] = 0x24;
  msg.buf[1] = (M_speed) & 0xFF;
  msg.buf[2] = (M_speed >> 8) & 0xFF;
  msg.buf[3] = (M_speed >> 16) & 0xFF;
  msg.buf[4] = (M_speed >> 24) & 0xFF;
  SendCommand(msg);
  delayMicroseconds(200);

  M_speed = M_speed * (-1);
  msg.id = (byte)id;
  msg.len = 5;
  msg.buf[0] = 0x25;
  msg.buf[1] = (M_speed) & 0xFF;
  msg.buf[2] = (M_speed >> 8) & 0xFF;
  msg.buf[3] = (M_speed >> 16) & 0xFF;
  msg.buf[4] = (M_speed >> 24) & 0xFF;
  SendCommand(msg);
  delayMicroseconds(200);

}

void Motor_CT::SaveSetting(){
  CAN_message_t msg;
  msg.id = (byte)id;
  msg.len = 1;
  msg.buf[0] = 0x0E;
  SendCommand(msg);
}


void Motor_CT::SendCommand(CAN_message_t msg){
  if (foot=='R'){
    can_R.write(msg);
  }
  else if (foot=='L'){
    can_L.write(msg);
  }
}

