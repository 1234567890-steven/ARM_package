# ifndef SITO_MOTOR_PACKAGE_H
# define SITO_MOTOR_PACKAGE_H

# include "Arduino.h"

# define SITO_Motor_BUADRATE 2250000
# define SEND_DELAY 500

class SITO_Motor
{
  private:
    int ID = 0;                                                                         // motor ID
    int Serial_port = 0;                                                                // motor serial port
    int Reduction_ratio = 0;                                                            // motor reduction ratio
    int One_cycle = 0;                                                                  // motor's encoder
    int Baudrate = SITO_Motor_BUADRATE;                                                 // Baudrate between teensy and motor
    int Send_delay = SEND_DELAY;                                                        // Delay time while sending
    Stream* Motor_serial = &Serial1;                                      
    byte RS485_send_buffer[100] = {0};
    uint16_t CRC16_cal(uint8_t *data, int len);                                         // CRC16 calculate method

  public:
    SITO_Motor();
    SITO_Motor(int id, int serial_port, int reduction_ratio, int one_cycle);
    void Motor_init();
    void Motor_on();
    void Motor_off();
    void Motor_return_origin();
    void Motor_angle_control(float angle);  //
    void Motor_angle_control_NA(float angle);
    void Motor_set_RPM(int _RPM);
    void Motor_set_ACC(int _ACC);
    void Motor_set_DEC(int _DEC);
    void Motor_read_angle_single();
    void Motor_read_angle_multiple();
};

# endif
