# include "SITO_motor_package.h"

SITO_Motor::SITO_Motor()
{
  ID = 0;
  Serial_port = 0;
  Reduction_ratio = 0;
  One_cycle = 0;
}

SITO_Motor::SITO_Motor(int id, int serial_port, int reduction_ratio, int one_cycle)
{
  ID = id;
  Serial_port = serial_port;
  Reduction_ratio = reduction_ratio;
  One_cycle = one_cycle;
}

// Set baudrate
void SITO_Motor::Motor_init()
{
  if (Serial_port == 1)
  {
    Motor_serial = &Serial1;
    Serial1.begin(Baudrate);
  }
  else if (Serial_port == 2) 
  {
    Motor_serial = &Serial2;
    Serial2.begin(Baudrate);
  }
  else if (Serial_port == 3) 
  {
    Motor_serial = &Serial3;
    Serial3.begin(Baudrate);
    Serial3.transmitterEnable(13);
  }
  else if (Serial_port == 4) 
  {
    Motor_serial = &Serial4;
    Serial4.begin(Baudrate);
  }
  else if (Serial_port == 5) 
  {
    Motor_serial = &Serial5;
    Serial5.begin(Baudrate);
    Serial5.transmitterEnable(2);
  }
  else 
  {
    Serial.println("----------- Serial Port Error -----------");
  }
}

// RS485 package (Motor on)
void SITO_Motor::Motor_on()
{
  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x10;
  RS485_send_buffer[4] = 0x00;
  RS485_send_buffer[5] = 0x00;
  RS485_send_buffer[6] = 0x00;
  RS485_send_buffer[7] = 0x01;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Motor off)
void SITO_Motor::Motor_off()
{
  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x10;
  RS485_send_buffer[4] = 0x00;
  RS485_send_buffer[5] = 0x00;
  RS485_send_buffer[6] = 0x00;
  RS485_send_buffer[7] = 0x00;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Return origin)
void SITO_Motor::Motor_return_origin()
{
  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x32;
  RS485_send_buffer[4] = 0x00;
  RS485_send_buffer[5] = 0x00;
  RS485_send_buffer[6] = 0x00;
  RS485_send_buffer[7] = 0x01;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Control motor angle)
void SITO_Motor::Motor_angle_control(float angle)
{
  // multiply with gear ratio
  int data = (angle / 360.0) * (One_cycle * Reduction_ratio);

  // byte operation
  byte data_4 = (data >> 24) & 0xFF;
  byte data_3 = (data >> 16) & 0xFF;
  byte data_2 = (data >> 8) & 0xFF;
  byte data_1 = (data >> 0) & 0xFF;

  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x82;
  RS485_send_buffer[4] = data_4;
  RS485_send_buffer[5] = data_3;
  RS485_send_buffer[6] = data_2;
  RS485_send_buffer[7] = data_1;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Control motor angle with no acceleration)
void SITO_Motor::Motor_angle_control_NA(float angle)
{
  // multiply with gear ratio
  int data = (angle / 360.0) * (One_cycle * Reduction_ratio);

  // byte operation
  byte data_4 = (data >> 24) & 0xFF;
  byte data_3 = (data >> 16) & 0xFF;
  byte data_2 = (data >> 8) & 0xFF;
  byte data_1 = (data >> 0) & 0xFF;

  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x81;
  RS485_send_buffer[4] = data_4;
  RS485_send_buffer[5] = data_3;
  RS485_send_buffer[6] = data_2;
  RS485_send_buffer[7] = data_1;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Set motor RPM)
void SITO_Motor::Motor_set_RPM(int _RPM)
{
  // byte operation
  byte data_4 = (_RPM >> 24) & 0xFF;
  byte data_3 = (_RPM >> 16) & 0xFF;
  byte data_2 = (_RPM >> 8) & 0xFF;
  byte data_1 = (_RPM >> 0) & 0xFF;

  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x2E;
  RS485_send_buffer[4] = data_4;
  RS485_send_buffer[5] = data_3;
  RS485_send_buffer[6] = data_2;
  RS485_send_buffer[7] = data_1;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Set motor ACC)
void SITO_Motor::Motor_set_ACC(int _ACC)
{
  // byte operation
  byte data_4 = (_ACC >> 24) & 0xFF;
  byte data_3 = (_ACC >> 16) & 0xFF;
  byte data_2 = (_ACC >> 8) & 0xFF;
  byte data_1 = (_ACC >> 0) & 0xFF;

  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x27;
  RS485_send_buffer[4] = data_4;
  RS485_send_buffer[5] = data_3;
  RS485_send_buffer[6] = data_2;
  RS485_send_buffer[7] = data_1;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

// RS485 package (Set motor DEC)
void SITO_Motor::Motor_set_DEC(int _DEC)
{
  // byte operation
  byte data_4 = (_DEC >> 24) & 0xFF;
  byte data_3 = (_DEC >> 16) & 0xFF;
  byte data_2 = (_DEC >> 8) & 0xFF;
  byte data_1 = (_DEC >> 0) & 0xFF;

  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x06;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x28;
  RS485_send_buffer[4] = data_4;
  RS485_send_buffer[5] = data_3;
  RS485_send_buffer[6] = data_2;
  RS485_send_buffer[7] = data_1;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

void SITO_Motor::Motor_read_angle_single()
{
  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x03;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x15;
  RS485_send_buffer[4] = 0x00;
  RS485_send_buffer[5] = 0x00;
  RS485_send_buffer[6] = 0x00;
  RS485_send_buffer[7] = 0x02;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}

void SITO_Motor::Motor_read_angle_multiple()
{
  RS485_send_buffer[0] = ID;
  RS485_send_buffer[1] = 0x03;
  RS485_send_buffer[2] = 0x00;
  RS485_send_buffer[3] = 0x13;
  RS485_send_buffer[4] = 0x00;
  RS485_send_buffer[5] = 0x00;
  RS485_send_buffer[6] = 0x00;
  RS485_send_buffer[7] = 0x02;
  uint16_t crc = CRC16_cal(RS485_send_buffer, 8);
  byte crc_HIGH = (crc >> 8) & 0xFF;
  byte crc_LOW = crc & 0xFF;
  RS485_send_buffer[8] = crc_HIGH;
  RS485_send_buffer[9] = crc_LOW;
  Motor_serial -> write(RS485_send_buffer, 10);
  delayMicroseconds(Send_delay);
}


//CRC16 calculation
uint16_t SITO_Motor::CRC16_cal(uint8_t *data, int len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}