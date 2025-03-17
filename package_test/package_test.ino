# include "Robotic_arms_package.h"
# include "HumanoidCommunication.h"
# include <Arduino.h>
#include <ArduinoEigenDense.h> 
#include "Motor_Control.h"
using namespace Eigen;

std::vector<std::vector<double>> test = {{0.0, 10.0, 0.0, 0.0, 0.0, 0.0}, {0.0, -10.0, 0.0, 0.0, 0.0, 0.0}};

const int ACC_and_DEC = 450;                // Motor initial ACC DEC setting
const int _RPM = 135;                       // Motor initial RPM setting
const int count = 200;                      // Data collect num
const int HZ = 20;                          // Data collect frequency

// Used in record and replay callback
int record_count = 0;
int replay_count = 0;
// std::vector<std::vector<float>> angle_single_data_record = {{10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
//                                                             {20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
std::vector<std::vector<float>> angle_single_data_record;

// Limit the data transmit scale
const float ANGLE_MIN_VALUE = -180.0;
const float ANGLE_MAX_VALUE = 540.0;
const float ANGLE_RANGE = ANGLE_MAX_VALUE - ANGLE_MIN_VALUE;
const int ANGLE_BIT_SIZE = 13;
const int ANGLE_MAX_SIZE = (1 << ANGLE_BIT_SIZE) - 1;

// Limit workspace
float workspaceR[3][2] = {{-10, 10}, {-83, 0}, {-100, -10}};

// set pamameter
float armR_length[3] = {30.4, 25.45, 14.26};
float current_angleR[6] = {0};
float current_angleL[6] = {0};
int init_state = 0;
float kp = 0.05; 

// 控制參數
const float error_threshold = 0.1; // 誤差閾值
const int freq = 50; // Hz
const int peroid = (1.0/freq)*1e6;
const int max_iterations = 100; // 最大迭代次數
int iteration_count = 0; // 當前迭代次數
bool control_running = false; // 運行標誌

// 角度和位置變數
float current_theta[6]; // 當前關節角度
float target_pos[3]; // 目標位置
float cur_pos[3];
float output_theta[6]; // 結果關節角度
float init_theta[6] = {-25 * M_PI / 180.0, -5 * M_PI / 180, -90 * M_PI / 180, 90 * M_PI / 180, 80 * M_PI / 180, 0 * M_PI / 180};
float pose[3] = {-85, 0, -25};
float initial_pos[6] = {40, -38, -10, pose[0], pose[1], pose[2]};
float armR_angles[6] = {0};
Vector3f current_pos; // 當前末端位置
Vector3f next_pos;
Vector3f error_vector; // 誤差向量
float error_magnitude; // 誤差大小
byte readin[100] = {0};
byte checksum;
int count_Rx = 0;
float arm_pos[3];
float des_pos[3];

// Object
Robotic_arms robotic;
HumanoidCommunication man(4);
Motor_CT waist;
IntervalTimer timer;
IntervalTimer jacobianTimer;

// Callback function for angle record
void angle_record_callback()
{
  // Set the limit of data num
  if (record_count > count)
    timer.end();

  // Timestamp
  long time = micros();

  // Get recorded angle data
  std::vector<float> angle = robotic.read_angle_data_record();

  // Save angle(single)
  angle_single_data_record.push_back(angle); 

  // Print timestamp and angle
  Serial.print(time);
  Serial.print("\t");
  for (int i = 0 ; i < angle.size() ; i ++)
  {
    Serial.print(angle[i]);
    Serial.print("\t");
  }
  Serial.println();

  // data counter plus one
  record_count += 1;
}

// Callback function for angle replay
void angle_replay_callback()
{
  // Set the limit of data num
  if (replay_count > count)
    timer.end();

  // Get recorded data and resize to 2d vector
  std::vector<std::vector<float>> angle_data_input;
  std::vector<std::vector<double>> angle_data_input_double;
  std::vector<float> new_row1;
  std::vector<float> new_row2;
  std::vector<float>& row = angle_single_data_record[replay_count];
  for (int i = 0 ; i < row.size() ; i ++)
  {
    Serial.print(row[i]);
    Serial.print("\t");
  }
  Serial.println();
  for (int j = 0 ; j <= 5 ; j ++)
  {
    new_row1.push_back(row[j]);
  }
  for (int j = 7 ; j <= 12 ; j ++)
  {
    new_row2.push_back(row[j]);
  }
  angle_data_input.push_back(new_row1);
  angle_data_input.push_back(new_row2);

  // transform float 2d vector to double 2d vector
  for (const auto& row: angle_data_input)
  {
    std::vector<double> new_row(row.begin(), row.end());
    angle_data_input_double.push_back(new_row);
  }

  // Send 2d vector into angle command 
  robotic.control_motor_angle_with_RPM(angle_data_input_double);

  // data counter plus one
  replay_count += 1;
}

// Transmit angle info to PC through UART
void answer_angle_callback()
{
  // If the PC ask for transmitting angle
  if (man.RetuenPermission == false)
    timer.end();

  // Create UART sending buffer
  byte buffer[100] = {0};

  // get all current angle
  std::vector<float> current_angle = robotic.read_angle_current();

  // print angle info
  for (int i = 0 ; i < current_angle.size() ; i ++)
  {
    Serial.print(current_angle[i]);
    Serial.print("\t");
  }
  Serial.println();

  // build sending buffer
  buffer[0] = 0x3E;
  buffer[1] = 0x01;
  buffer[2] = 0x02;
  buffer[3] = 0x30;
  buffer[4] = int(current_angle[0] * 100) & 0xFF;
  buffer[5] = (int(current_angle[0] * 100) >> 8) & 0xFF;
  buffer[6] = (int(current_angle[0] * 100) >> 16) & 0xFF;
  buffer[7] = (int(current_angle[0] * 100) >> 24) & 0xFF;
  buffer[8] = int(current_angle[1] * 100) & 0xFF;
  buffer[9] = (int(current_angle[1] * 100) >> 8) & 0xFF;
  buffer[10] = (int(current_angle[1] * 100) >> 16) & 0xFF;
  buffer[11] = (int(current_angle[1] * 100) >> 24) & 0xFF;
  buffer[12] = int(current_angle[2] * 100) & 0xFF;
  buffer[13] = (int(current_angle[2] * 100) >> 8) & 0xFF;
  buffer[14] = (int(current_angle[2] * 100) >> 16) & 0xFF;
  buffer[15] = (int(current_angle[2] * 100) >> 24) & 0xFF;
  buffer[16] = int(current_angle[3] * 100) & 0xFF;
  buffer[17] = (int(current_angle[3] * 100) >> 8) & 0xFF;
  buffer[18] = (int(current_angle[3] * 100) >> 16) & 0xFF;
  buffer[19] = (int(current_angle[3] * 100) >> 24) & 0xFF;
  buffer[20] = int(current_angle[4] * 100) & 0xFF;
  buffer[21] = (int(current_angle[4] * 100) >> 8) & 0xFF;
  buffer[22] = (int(current_angle[4] * 100) >> 16) & 0xFF;
  buffer[23] = (int(current_angle[4] * 100) >> 24) & 0xFF;
  buffer[24] = int(current_angle[5] * 100) & 0xFF;
  buffer[25] = (int(current_angle[5] * 100) >> 8) & 0xFF;
  buffer[26] = (int(current_angle[5] * 100) >> 16) & 0xFF;
  buffer[27] = (int(current_angle[5] * 100) >> 24) & 0xFF;
  buffer[28] = int(current_angle[7] * 100) & 0xFF;
  buffer[29] = (int(current_angle[7] * 100) >> 8) & 0xFF;
  buffer[30] = (int(current_angle[7] * 100) >> 16) & 0xFF;
  buffer[31] = (int(current_angle[7] * 100) >> 24) & 0xFF;
  buffer[32] = int(current_angle[8] * 100) & 0xFF;
  buffer[33] = (int(current_angle[8] * 100) >> 8) & 0xFF;
  buffer[34] = (int(current_angle[8] * 100) >> 16) & 0xFF;
  buffer[35] = (int(current_angle[8] * 100) >> 24) & 0xFF;
  buffer[36] = int(current_angle[9] * 100) & 0xFF;
  buffer[37] = (int(current_angle[9] * 100) >> 8) & 0xFF;
  buffer[38] = (int(current_angle[9] * 100) >> 16) & 0xFF;
  buffer[39] = (int(current_angle[9] * 100) >> 24) & 0xFF;
  buffer[40] = int(current_angle[10] * 100) & 0xFF;
  buffer[41] = (int(current_angle[10] * 100) >> 8) & 0xFF;
  buffer[42] = (int(current_angle[10] * 100) >> 16) & 0xFF;
  buffer[43] = (int(current_angle[10] * 100) >> 24) & 0xFF;
  buffer[44] = int(current_angle[11] * 100) & 0xFF;
  buffer[45] = (int(current_angle[11] * 100) >> 8) & 0xFF;
  buffer[46] = (int(current_angle[11] * 100) >> 16) & 0xFF;
  buffer[47] = (int(current_angle[11] * 100) >> 24) & 0xFF;
  buffer[48] = int(current_angle[12] * 100) & 0xFF;
  buffer[49] = (int(current_angle[12] * 100) >> 8) & 0xFF;
  buffer[50] = (int(current_angle[12] * 100) >> 16) & 0xFF;
  buffer[51] = (int(current_angle[12] * 100) >> 24) & 0xFF;

  // CRC claculate
  for (int i = 4 ; i < 52 ; i ++)
    buffer[52] += buffer[i];
  buffer[52] = buffer[52] & 0xFF;

  // Send package through UART
  Serial4.write(buffer, 53);
  delayMicroseconds(500);

  // Reset buffer
  for (int i = 0 ; i < sizeof(buffer) ; i ++)
    buffer[i] = 0;
}

// Input motor ID and angle, then it will output two byte data package
std::pair<uint8_t, uint8_t> pack_motor_data(uint8_t ID, float angle)
{
  if (angle < ANGLE_MIN_VALUE)
    angle = ANGLE_MIN_VALUE;
  else if (angle > ANGLE_MAX_VALUE)
    angle = ANGLE_MAX_VALUE;
  uint16_t angle_bits = (((angle - ANGLE_MIN_VALUE) / ANGLE_RANGE) * ANGLE_MAX_SIZE);
  uint16_t data_bits = (ID << ANGLE_BIT_SIZE) | angle_bits;
  uint8_t data_bits_HIGH = (data_bits >> 8) & 0xFF;
  uint8_t data_bits_LOW = (data_bits) & 0xFF;
  return {data_bits_HIGH, data_bits_LOW};
}

// Send data package to python to save data as csv file
void write_recorded_angle(std::vector<std::vector<float>> angle_single_data_record)
{
  std::vector<uint8_t> R_package;
  std::vector<uint8_t> L_package;
  uint8_t HEAD_1 = 0xFF;              // first header
  uint8_t HEAD_2 = 0xFF;              // second header
  uint8_t FUNC_1 = 0x01;                // function code(0x01: right)
  uint8_t FUNC_2 = 0x02;                // function code(0x02: left)
  uint8_t END_1 = 0xFF;               // first end code
  uint8_t END_2 = 0xFE;               // second end code
  R_package.push_back(HEAD_1);
  R_package.push_back(HEAD_2);
  R_package.push_back(FUNC_1);
  L_package.push_back(HEAD_1);
  L_package.push_back(HEAD_2);
  L_package.push_back(FUNC_2);
  for (int i = 0 ; i < angle_single_data_record.size() ; i ++)
  {
    for (int j = 0 ; j < 6 ; j ++)
    {
      std::pair<uint8_t, uint8_t> data_R;
      std::pair<uint8_t, uint8_t> data_L;
      data_R = pack_motor_data(j + 1, angle_single_data_record[i][j]);
      data_L = pack_motor_data(j + 1, angle_single_data_record[i][j]);
      R_package.push_back(data_R.first);
      R_package.push_back(data_R.second);
      L_package.push_back(data_L.first);
      L_package.push_back(data_L.second);
    }
  }
  R_package.push_back(END_1);
  R_package.push_back(END_2);
  L_package.push_back(END_1);
  L_package.push_back(END_2);
  
  Serial.write(R_package.data(), R_package.size());
  delay(2000);
  Serial.write(L_package.data(), L_package.size());
  delay(2000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 正向運動學函數
Vector3f fk(float* theta_input, bool degree) {

  float J1 = theta_input[0];
  float J2 = theta_input[1];
  float J3 = theta_input[2];
  float J4 = theta_input[3];
  float J5 = theta_input[4];
  float J6 = theta_input[5];

  if (degree)
  {
    J1 = J1/180.0*M_PI;
    J2 = J2/180.0*M_PI;
    J3 = J3/180.0*M_PI;
    J4 = J4/180.0*M_PI;
    J5 = J5/180.0*M_PI;
    J6 = J6/180.0*M_PI;
  }

  
  float d3 = armR_length[0];
  float d5 = armR_length[1];
  float a6 = armR_length[2];
  
  // 計算 x
  float x = (d3 * cos(J2) * sin(J1) +
         d5 * (cos(J2) * cos(J4) * sin(J1) - cos(J1) * sin(J3) * sin(J4) - cos(J3) * sin(J1) * sin(J2) * sin(J4)) +
         a6 * (cos(J2) * cos(J4) * cos(J6) * sin(J1) - cos(J1) * cos(J6) * sin(J3) * sin(J4) -
               cos(J1) * cos(J3) * sin(J5) * sin(J6) - cos(J1) * cos(J4) * cos(J5) * sin(J3) * sin(J6) -
               cos(J3) * cos(J6) * sin(J1) * sin(J2) * sin(J4) - cos(J2) * cos(J5) * sin(J1) * sin(J4) * sin(J6) +
               sin(J1) * sin(J2) * sin(J3) * sin(J5) * sin(J6) - cos(J3) * cos(J4) * cos(J5) * sin(J1) * sin(J2) * sin(J6)));

  // 計算 y
  float y = d5 * cos(J1) * cos(J3) * sin(J2) * sin(J4) - d5 * cos(J1) * cos(J2) * cos(J4) - d5 * sin(J1) * sin(J3) * sin(J4) - 
            a6 * cos(J1) * cos(J2) * cos(J4) * cos(J6) - d3 * cos(J1) * cos(J2) - a6 * cos(J6) * sin(J1) * sin(J3) * sin(J4) - 
            a6 * cos(J3) * sin(J1) * sin(J5) * sin(J6) + a6 * cos(J1) * cos(J3) * cos(J6) * sin(J2) * sin(J4) + 
            a6 * cos(J1) * cos(J2) * cos(J5) * sin(J4) * sin(J6) - a6 * cos(J4) * cos(J5) * sin(J1) * sin(J3) * sin(J6) - 
            a6 * cos(J1) * sin(J2) * sin(J3) * sin(J5) * sin(J6) + a6 * cos(J1) * cos(J3) * cos(J4) * cos(J5) * sin(J2) * sin(J6);

  // 計算 z
  float z = (-d3 * sin(J2) +
         d5 * (-cos(J4) * sin(J2) - cos(J2) * cos(J3) * sin(J4)) +
         a6 * (cos(J2) * sin(J3) * sin(J5) * sin(J6) - cos(J4) * cos(J6) * sin(J2) -
               cos(J2) * cos(J3) * cos(J6) * sin(J4) + cos(J5) * sin(J2) * sin(J4) * sin(J6) -
               cos(J2) * cos(J3) * cos(J4) * cos(J5) * sin(J6)));
               
  Vector3f result;
  result << x, y, z;
  
  return result;
}

// 計算Jacobian矩陣
MatrixXf calJacobianMatrix(float* theta_input, float eps = 1e-6) {
  MatrixXf J(3, 6);
  float joint_angles[6];
  
  // 複製值
  for (int i = 0; i < 6; i++) {
    joint_angles[i] = theta_input[i];
  }
  
  for (int i = 0; i < 6; i++) {
    float angles_plus[6];
    float angles_minus[6];
    
    // 複製值並加/減 epsilon
    for (int j = 0; j < 6; j++) {
      angles_plus[j] = joint_angles[j];
      angles_minus[j] = joint_angles[j];
    }
    
    angles_plus[i] += eps;
    angles_minus[i] -= eps;
    
    // 計算Jacobian的第i列
    Vector3f fk_plus = fk(angles_plus, false);
    Vector3f fk_minus = fk(angles_minus, false);
    
    Vector3f col = (fk_plus - fk_minus) / (2 * eps);
    J.col(i) = col;
  }
  
  return J;
}

// 使用SVD計算Jacobian的偽逆
MatrixXf calJPinv(float* theta_input) {
  MatrixXf J = calJacobianMatrix(theta_input);
  
  // 使用SVD進行偽逆計算
  JacobiSVD<MatrixXf> svd(J, ComputeFullU | ComputeFullV);
  
  // 獲取奇異值
  VectorXf singular_values = svd.singularValues();
  MatrixXf S_inv = MatrixXf::Zero(6, 3);
  
  // 計算奇異值的倒數，帶閾值
  double epsilon = 1e-6; // 奇異值閾值
  for (int i = 0; i < singular_values.size(); i++) {
    if (singular_values(i) > epsilon) {
      S_inv(i, i) = 1.0 / singular_values(i);
    }
  }
  
  // 計算偽逆: V * S+ * U^T
  MatrixXf J_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();
  
  return J_pinv;
}

// Timer中斷函數 - 執行一次Jacobian迭代
void jacobianIteration() {
  if (!control_running) return;
 
  // 檢查是否達到最大迭代次數
  if (iteration_count >= max_iterations) {
    control_running = false;
    jacobianTimer.end(); // 停止timer
    
    // Serial.println("Max iterations reached!");
    // printResults();
    return;
  }
  
  // 計算當前位置
  // for (int i = 0; i < 6; i++)
  // {
  //   current_theta[i] = current_angleR[i]*M_PI/180.0;
  // }
  // get all current angle
  // std::vector<float> current_angle = robotic.read_angle_current();
  // current_pos = fk(current_theta, false);

  std::vector<float> current_angle = robotic.read_angle_current();
  // 複製初始關節角度
  for (int i = 0; i < 6; i++) {
    current_theta[i] = current_angle[i];
  }
  
  // 計算誤差
  Vector3f target_vector(target_pos[0], target_pos[1], target_pos[2]);
  Vector3f cur_vector(cur_pos[0], cur_pos[1], cur_pos[2]);
  error_vector = target_vector - cur_vector;
  error_magnitude = error_vector.norm();

  // Serial.print("X: ");Serial.print(current_pos(0), 5);Serial.print("  ");
  // Serial.print("Y: ");Serial.print(current_pos(1), 5);Serial.print("  ");
  // Serial.print("Z: ");Serial.println(current_pos(2), 5);

  
  // 檢查是否達到誤差閾值
  if (error_magnitude < error_threshold) {
    control_running = false;
    jacobianTimer.end(); // 停止timer
    
    // Serial.println("Convergence achieved!");
    // printResults();
    return;
  }
  
  // 計算Jacobian偽逆
  MatrixXf J_pinv = calJPinv(current_theta);
  
  // 計算關節角度增量
  Vector3f scaled_delta_p = kp * error_vector;
  VectorXf delta_q = J_pinv * scaled_delta_p;
  
  // 更新關節角度
  for (int i = 0; i < 6; i++) {
    current_theta[i] += delta_q(i);
    output_theta[i] = current_theta[i]*180/M_PI;
  }
  // Serial.println(String("") + String(current_theta[0]*180/M_PI, 3) + ",  " + String(current_theta[1]*180/M_PI, 3) + ",  " + String(current_theta[2]*180/M_PI, 3) + ",  " + String(current_theta[3]*180/M_PI, 3) + ",  " + String(current_theta[4]*180/M_PI, 3) + ",  " + String(current_theta[5]*180/M_PI, 3));
  // Serial.println(String("") + String(output_theta[0], 3) + ",  " + String(output_theta[1], 3) + ",  " + String(output_theta[2], 3) + ",  " + String(output_theta[3], 3) + ",  " + String(output_theta[4], 3) + ",  " + String(output_theta[5], 3));
  
  // Check the workspace
  next_pos = fk(current_theta, false);
  if (next_pos(1) > workspaceR[0][0] && next_pos(1) < workspaceR[0][1] && next_pos(2) > workspaceR[1][0] && next_pos(2) < workspaceR[1][1] && next_pos(3) > workspaceR[2][0] && next_pos(3) < workspaceR[2][1])
  {
    // Serial.println("Out of workspace!");
    control_running = false;
    jacobianTimer.end(); // 停止timer
    return;
  }
  
  // std::vector<float> current_angle = robotic.read_angle_current();
  std::vector<std::vector<double>> angles_output = {{-output_theta[0], -output_theta[1], output_theta[2], -output_theta[3], output_theta[4], -output_theta[5]}, {current_angle[7], current_angle[8], current_angle[9], current_angle[10], current_angle[11], current_angle[12]}};
  
  // Serial.println("####### output angles #######");
  // for (size_t i = 0; i < angles_output.size(); i++)
  // {
  //   for (size_t j = 0; j < angles_output[i].size(); j++)
  //   {
  //     Serial.print(angles_output[i][j], 5);Serial.print(", ");
  //   }
  //   Serial.println();
  // }
  // Serial.println("#############################");
  // robotic.control_motor_angle_NA(angles_output);
  // Serial5.write(iteration_count);
  // 增加迭代計數
  iteration_count++;
  
  // 每10次迭代輸出一次狀態
  if (iteration_count % 10 == 0) {
    // Serial.print("Iteration ");
    // Serial.print(iteration_count);
    // Serial.print(", Error: ");
    // Serial.println(error_magnitude, 5);
  }
}

// 開始差分運動學控制
void startDiffKinematic(float* current_position, float* target_position) {
  // get all current angle
  // std::vector<float> current_angle = robotic.read_angle_current();
  // // 複製初始關節角度
  // for (int i = 0; i < 6; i++) {
  //   current_theta[i] = current_angle[i];
  // }
  // for (int i = 0; i < 6; i++) {
  //   Serial.print(current_theta[i], 5);Serial.print(", ");
  // }
  // Serial.println();


  // 複製現在位置
  for (int i = 0; i < 3; i++) {
    cur_pos[i] = current_position[i];
  }

  
  // 複製目標位置
  for (int i = 0; i < 3; i++) {
    target_pos[i] = target_position[i];
  }
  
  // 重置迭代計數器
  iteration_count = 0;
  
  // 開始控制過程
  control_running = true;

  jacobianTimer.begin(jacobianIteration,  peroid); 
  
  // Serial.println("Differential kinematics control started");
}

// 打印結果
void printResults() {
  Serial.println("Final joint angles :");
  for (int i = 0; i < 6; i++) {
    Serial.print(current_theta[i]*180/M_PI);
    Serial.print(", ");
  }
  Serial.println();
  
  Vector3f final_pos = fk(current_theta, false);
  Serial.println("Final position:");
  Serial.print(final_pos(0), 5); Serial.print(", ");
  Serial.print(final_pos(1), 5); Serial.print(", ");
  Serial.println(final_pos(2), 5);
  
  // Serial.print("Total iterations: ");
  // Serial.println(iteration_count);
  
  Serial.print("Final error: ");
  Serial.println(error_magnitude, 5);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
void setup() 
{
  man.SerialInit();
  Serial.begin(115200);
  waist.Set_value(14, 81, 'R');
  waist.Serial_Init();
//  Serial3.begin(2250000);
//  Serial5.begin(2250000);
  robotic.open_arms();                    // open arms
  delay(1000);
//  robotic.set_RPM(15);                  // set RPM
//  delay(1000);
//  robotic.set_ACC(ACC_and_DEC);           // set ACC
//  delay(1000);
//  robotic.set_DEC(ACC_and_DEC);           // set DEC
//  delay(1000);

  // read offset(MUST DO)
  robotic.set_offset();

  // initial pose
  waist.SetSpeedCommand(5);
  delay(1);
  waist.SendPosCommand(0);
  robotic.set_RPM(135);                  // set RPM
  delay(1000);
  robotic.initial_pose();
  delay(1000);
  robotic.set_RPM(_RPM);                  // set RPM
  /////////////////////////////////////////////  Motion record and replay  /////////////////////////////////////////////
//  delay(20000);
//  robotic.close_arms();
//  delay(1000);
//
//  // Set timer(angle record)
//  timer.begin(angle_record_callback, 50000);
//  delay((count / HZ) * 1000 + 5000);
//
//  // // Open arms ready to replay
//  // robotic.open_arms();
//  // delay(10000);
//
//  // // Set timer(angle replay)
//  // timer.begin(angle_replay_callback, 50000);
//  // delay((count / HZ) * 1000 + 5000);
//
//  // Save csv file 
//  write_recorded_angle(angle_single_data_record);
//
//  /////////////////////////////////////////////  UART test  /////////////////////////////////////////////
//  // man.RetuenPermission = true;
//  // serialEvent4();
//  // delay(10000);
//  // man.RetuenPermission = false;

}

void loop() 
{
  // if (Serial.available() > 0)
  // {
  //   int n = Serial.parseInt();
  //   switch (n)
  //   {
  //     case 0:
  //     {
  //       robotic.close_arms();
  //     }break;

  //     case 1:
  //     {
  //       startDiffKinematic(init_theta, initial_pos);
  //     }break;
  //   }
  // }
}

void reset_params()
{
  for (int i = 0; i < 100; i++)
  {
    readin[i] = 0;
  }
  count_Rx = 0;
  checksum = 0;
}

////////////////////////////////////////////////////////////////////  Serial event  ////////////////////////////////////////////////////////////////////

// Triggered while received data from Computer(Serial)
void serialEvent()
{
  byte temp[1];
  // robotic.receive();
  while (Serial.available())
  {
    Serial.readBytes(temp, 1);
    if (count_Rx == 0 && temp[0] == 0xF6)
    {
      readin[0] = temp[0];
      count_Rx = 1;
      // Serial5.write(temp, 1);
    }
    else if (count_Rx == 1 && temp[0] == 0x00)
    {
      readin[1] = temp[0];
      count_Rx = 2;
    }
    else if (count_Rx > 2 && count_Rx == readin[2]+3)
    {
      readin[count_Rx] = temp[0];
      // Serial5.write(temp, 1);
      checksum = 0;

      for(int i = 3; i < count_Rx; i++)
      {
        checksum += readin[i];
        // Serial.println(readin[i], HEX);
        // Serial5.write(checksum);
      }
      checksum &= 0xFF;
      // Serial5.write(checksum);
      if (checksum != readin[count_Rx])
      {
        // Serial.println("Checksum error");
        // Serial5.write(90);
        reset_params();
      }
      else
      {
        int des_pos_int[3];
        int arm_pos_int[3];
        for (int i = 0; i < 3; i++)
        {
          des_pos_int[i] = (readin[3 + i * 4] & 0xFF) |
                           ((readin[4 + i * 4] & 0xFF) << 8) |
                           ((readin[5 + i * 4] & 0xFF) << 16) |
                           ((readin[6 + i * 4] & 0xFF) << 24);
        }
        for (int i = 0; i < 3; i++)
        {
          arm_pos_int[i] = (readin[15 + i * 4] & 0xFF) |
                           ((readin[16 + i * 4] & 0xFF) << 8) |
                           ((readin[17 + i * 4] & 0xFF) << 16) |
                           ((readin[18 + i * 4] & 0xFF) << 24);
        }
        for (int i = 0; i < 3; i++) {
            des_pos[i] = des_pos_int[i] / 100.0;
            arm_pos[i] = arm_pos_int[i] / 100.0;
        }
        // Serial5.write(readin, 28);
        // for (int i = 0; i < 3; i++)
        // {
        //   Serial.println(des_pos[i]);
        // }
        // for (int i = 0; i < 3; i++)
        // {
        //   Serial.println(arm_pos[i]);
        // }
        reset_params();
        startDiffKinematic(arm_pos, des_pos);

      }
    }
    else
    {
      readin[count_Rx] = temp[0];
      // Serial5.write(count_Rx);
      // Serial5.write(temp, 1);
      count_Rx++;
    }
  }
}

// Triggered while received data from RS485(Serial 3)
void serialEvent3()
{
  robotic.receive_3();
}

// Triggered while received data from UART(Serial 4)
void serialEvent4()
{
  man._Receive();
  if (man.RetuenPermission == true)
    timer.begin(answer_angle_callback, 50000);
  // robotic.control_motor_angle_with_RPM(man.ArmAngle);
}

// Triggered while received data from RS485(Serial 5)
void serialEvent5()
{
  robotic.receive_5();
}
