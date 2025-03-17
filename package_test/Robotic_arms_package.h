# ifndef ROBOTIC_ARMS_PACKAGE_H
# define ROBOTIC_ARMS_PACKAGE_H

# include "Arduino.h"
# include "SITO_motor_package.h"
# include "ieee754.h"
# include <vector>
# include <math.h>

# define MOTOR_NUM 14
# define RPM_CAL_DELTA_TIME 0.05        // 20Hz(for angle control with RPM)

class Robotic_arms
{
  private:
  
    // Object
    SITO_Motor MotorArr[MOTOR_NUM];
    IEEE754 ie754;

    bool RECIEVING = false;
    bool RECIEVING_END = false;
    int csv_data_num = -1;
    std::vector<std::vector<float>> right_arm_csv_data;
    std::vector<std::vector<float>> left_arm_csv_data;
    
    // For receiving data(USB) 
    byte USB_receive_buffer[100] = {0};
    byte USB_send_buffer[100] = {0};
    int USB_pkg_num = 0;
    int USB_data_num = 0;

    // For receiving data(RS485)
    byte RS485_receive_buffer[100] = {0};
    byte RS485_send_buffer[100] = {0};
    int RS485_pkg_num = 0;
    int RS485_data_num = 0;

    // function
    void reset_RS485_parameters();                              // reset receive parameter after receive one package
    void reset_USB_parameters();                                // reset receive parameter after receive one package
    uint16_t CRC16_cal(uint8_t *data, int len);                 // CRC16 calculation function

    // for filter
    double alpha = 3.0;                                         // compensate parameter
    int reduction_ratio = 101;                                  // motor reduction ratio
    int regular_threshold = 1500;                               // the threshold of regular interval
    int lower_threshold = 1000;                                 // the threshold of lower interval
    int regular_RPM = 1500;                                     // the offset of RPM in regular interval
    int lower_RPM = 1000;                                       // the offset of RPM in regular interval
    int window_size = 12;                                       // window size of smoothing filter
    std::vector<std::vector<double>> rad_angle = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};            // for data processing
    std::vector<std::vector<double>> previous_angle = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};       // for data processing
    std::vector<std::vector<double>> delta_angle = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};          // for data processing
    std::vector<std::vector<double>> angular_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};     // for data processing
    std::vector<std::vector<double>> target_RPM = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};           // for data processing
    std::vector<std::vector<std::vector<double>>> filter_data_list_RPM;       // collect all data calculated before

  public:
    Robotic_arms();
    void open_arms();
    void close_arms();
    void return_origin();
    void control_motor_angle(std::vector<std::vector<double>> angle);
    void control_motor_angle_NA(std::vector<std::vector<double>> angle);
    void control_motor_angle_with_RPM(std::vector<std::vector<double>> angle);
    void set_RPM(int _RPM);
    void set_ACC(int _ACC);
    void set_DEC(int _DEC);
    void read_angle_single();                         // read single circle encoder
    void read_angle_multiple();                       // read multiple circles encoder, it will return zero after turn off power
    std::vector<float> read_angle_data_record();      // read all angle(for recording motion data)
    std::vector<float> read_angle_current();          // read all angle(RECORDING REAL ANGLES OF ROBOTIC ARMS)
    void set_offset();                                // Get offset value by reading multiple circles encoder
    void initial_pose();                              // To initial pose
    void receive();                                   // receive data from Serial
    void receive_3();                                 // receive data from Serial3
    void receive_5();                                 // receive data from Serial5

    // motor angle(single) record
    float motor_angles_read_single[MOTOR_NUM] = {0};
    byte motor_angles_read_single_bytes[MOTOR_NUM * 4];

    // motor angle(multiple) record
    float motor_angles_read_multiple[MOTOR_NUM] = {0};
    byte motor_angles_read_multiple_bytes[MOTOR_NUM * 4];

    // offset record
    float motor_angle_offset[MOTOR_NUM] = {0};
};

# endif