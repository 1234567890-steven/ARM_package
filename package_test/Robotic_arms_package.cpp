# include "Robotic_arms_package.h"

Robotic_arms::Robotic_arms()
{
  // 創建馬達物件, 初始化馬達Serial port
  for (int i = 0 ; i < 7 ; i ++)
  {
    MotorArr[i] = SITO_Motor(i + 1, 3, 101, 32767);
    MotorArr[i].Motor_init();
  }

  // 創建馬達物件, 初始化馬達Serial port
  for (int i = 7 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i] = SITO_Motor(i + 1, 5, 101, 32767);
    MotorArr[i].Motor_init();
  }
  
}

void Robotic_arms::open_arms()
{
  // Send to motor
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_on();
  }
  Serial.println("ARMS OPEN");
}

void Robotic_arms::close_arms()
{
  // Send to motor
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_off();
  }
  Serial.println("ARMS CLOSE");
}

void Robotic_arms::return_origin()
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_return_origin();
  }
  
}

// Send angle control command to dual arms(12 motors), NEED COMPENSATE, send M = S - offset 
void Robotic_arms::control_motor_angle(std::vector<std::vector<double>> angle)
{
  for (int i = 0 ; i < angle.size() ; i ++)
  {
    for (int j = 0 ; j < angle[i].size() ; j ++)
    {
      angle[i][j] -= motor_angle_offset[i * 7 + j];
      if (angle[i][j] < -180.0)
      {
        angle[i][j] += 360.0;
      }
      else if (angle[i][j] > 180.0)
      {
        angle[i][j] -= 360.0;
      }
      MotorArr[i * 7 + j].Motor_angle_control(float(angle[i][j]));
    }
  }
  
  // // Single motor test
  // MotorArr[0].Motor_angle_control(float(angle[0][0]));
  // MotorArr[1].Motor_angle_control(float(angle[0][1]));
  // MotorArr[2].Motor_angle_control(float(angle[0][2]));
  // MotorArr[3].Motor_angle_control(float(angle[0][3]));
  // MotorArr[4].Motor_angle_control(float(angle[0][4]));
  // MotorArr[5].Motor_angle_control(float(angle[0][5]));
  // MotorArr[7].Motor_angle_control(float(angle[1][0]));
  // MotorArr[8].Motor_angle_control(float(angle[1][1]));
  // MotorArr[9].Motor_angle_control(float(angle[1][2]));
  // MotorArr[10].Motor_angle_control(float(angle[1][3]));
  // MotorArr[11].Motor_angle_control(float(angle[1][4]));
  // MotorArr[12].Motor_angle_control(float(angle[1][5]));
}

// Send angle control command to dual arms(12 motors) with no ACC, NEED COMPENSATE, send M = S - offset 
void Robotic_arms::control_motor_angle_NA(std::vector<std::vector<double>> angle)
{
  for (int i = 0 ; i < angle.size() ; i ++)
  {
    for (int j = 0 ; j < angle[i].size() ; j ++)
    {
      angle[i][j] -= motor_angle_offset[i * 7 + j];
      if (angle[i][j] < -180.0)
      {
        angle[i][j] += 360.0;
      }
      else if (angle[i][j] > 180.0)
      {
        angle[i][j] -= 360.0;
      }
      MotorArr[i * 7 + j].Motor_angle_control_NA(float(angle[i][j]));
    }
  }

  // Single motor test
  // MotorArr[0].Motor_angle_control_NA(float(angle[0][0]));
  // MotorArr[1].Motor_angle_control_NA(float(angle[0][1]));
  // MotorArr[2].Motor_angle_control_NA(float(angle[0][2]));
  // MotorArr[3].Motor_angle_control_NA(float(angle[0][3]));
  // MotorArr[4].Motor_angle_control_NA(float(angle[0][4]));
  // MotorArr[5].Motor_angle_control_NA(float(angle[0][5]));
  // MotorArr[7].Motor_angle_control_NA(float(angle[1][0]));
  // MotorArr[8].Motor_angle_control_NA(float(angle[1][1]));
  // MotorArr[9].Motor_angle_control_NA(float(angle[1][2]));
  // MotorArr[10].Motor_angle_control_NA(float(angle[1][3]));
  // MotorArr[11].Motor_angle_control_NA(float(angle[1][4]));
  // MotorArr[12].Motor_angle_control_NA(float(angle[1][5]));
  
}

// Send angle control command and RPM command to dual arms(12 motors), NEED COMPENSATE, send M = S - offset 
void Robotic_arms::control_motor_angle_with_RPM(std::vector<std::vector<double>> angle)
{
  // Get all RPM
  for (int i = 0; i < 2 ; i ++)
  {
    for (int j = 0 ; j < 6 ; j ++)
    {
      rad_angle[i][j] = angle[i][j] * (M_PI/180.0);
      delta_angle[i][j] = rad_angle[i][j] - previous_angle[i][j];
      angular_velocity[i][j] = delta_angle[i][j] / RPM_CAL_DELTA_TIME;
      target_RPM[i][j] = angular_velocity[i][j] * alpha * (60 / (2 * M_PI)) * reduction_ratio;
      if (target_RPM[i][j] < regular_threshold && target_RPM[i][j] >= lower_threshold)
      {
        target_RPM[i][j] = regular_RPM;
      }
      else if (target_RPM[i][j] < lower_threshold)
      {
        target_RPM[i][j] = lower_RPM;
      }
      else if (target_RPM[i][j] > 2999)
      {
        target_RPM[i][j] = 2999;
      }
      previous_angle[i][j] = rad_angle[i][j];
    }
  }

  std::vector<std::vector<double>> RPM_after_filter = {{0, 0, 0, 0, 0, 0}, {0, 0 ,0 ,0 ,0 ,0}};
  if (filter_data_list_RPM.empty()) 
  {
      filter_data_list_RPM.resize(1, std::vector<std::vector<double>>(2, std::vector<double>(6, 0.0)));
  }
  if (filter_data_list_RPM.size() > window_size)
    {
      filter_data_list_RPM.erase(filter_data_list_RPM.begin()); // 移除最舊的數據
    } 
  filter_data_list_RPM.push_back(target_RPM);
  
  // Smooth filter
  for (int i = 0 ; i < 2 ; i ++)
  {
    std::vector<double> RPM_sum(target_RPM[i].size(), 0.0);
    int data_size = filter_data_list_RPM.size();
    
    if (data_size <= window_size)
    {
      for (int j = 0 ; j < data_size ; j ++)
      {
        for (int k = 0 ; k < target_RPM[i].size() ; k ++)
        {
          RPM_sum[k] += filter_data_list_RPM[j][i][k];
        }
      }
      for (int j = 0 ; j < target_RPM[i].size() ; j ++)
      {
        RPM_after_filter[i][j] = RPM_sum[j] / data_size;
      }
    }
    else
    {
      for (int j = data_size - window_size ; j < data_size ; j ++)
      {
        for (int k = 0 ; k < target_RPM[i].size() ; k ++)
        {
          RPM_sum[k] += filter_data_list_RPM[j][i][k];
        }
      }
      for (int j = 0 ; j < target_RPM[i].size() ; j ++)
      {
        RPM_after_filter[i][j] = RPM_sum[j] / window_size;
      }
    }
  }

  // Send RPM package
  MotorArr[0].Motor_set_RPM(int(RPM_after_filter[0][0]));
  MotorArr[1].Motor_set_RPM(int(RPM_after_filter[0][1]));
  MotorArr[2].Motor_set_RPM(int(RPM_after_filter[0][2]));
  MotorArr[3].Motor_set_RPM(int(RPM_after_filter[0][3]));
  MotorArr[4].Motor_set_RPM(int(RPM_after_filter[0][4]));
  MotorArr[5].Motor_set_RPM(int(RPM_after_filter[0][5]));
  MotorArr[7].Motor_set_RPM(int(RPM_after_filter[1][0]));
  MotorArr[8].Motor_set_RPM(int(RPM_after_filter[1][1]));
  MotorArr[9].Motor_set_RPM(int(RPM_after_filter[1][2]));
  MotorArr[10].Motor_set_RPM(int(RPM_after_filter[1][3]));
  MotorArr[11].Motor_set_RPM(int(RPM_after_filter[1][4]));
  MotorArr[12].Motor_set_RPM(int(RPM_after_filter[1][5]));

  // Send angle package
  for (int i = 0 ; i < angle.size() ; i ++)
  {
    for (int j = 0 ; j < angle[i].size() ; j ++)
    {
      angle[i][j] -= motor_angle_offset[i * 7 + j];
      if (angle[i][j] < -180.0)
      {
        angle[i][j] += 360.0;
      }
      else if (angle[i][j] > 180.0)
      {
        angle[i][j] -= 360.0;
      }
      MotorArr[i * 7 + j].Motor_angle_control(float(angle[i][j]));
    }
  }
}

void Robotic_arms::set_RPM(int _RPM)
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_set_RPM(_RPM);
  } 
  Serial.println("RPM SET");
}

void Robotic_arms::set_ACC(int _ACC)
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_set_ACC(_ACC);
  }
  Serial.println("ACC SET");   
}

void Robotic_arms::set_DEC(int _DEC)
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_set_ACC(_DEC);
  } 
  Serial.println("DEC SET");
}

void Robotic_arms::read_angle_single()
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_read_angle_single();
  } 
  
}

void Robotic_arms::read_angle_multiple()
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    MotorArr[i].Motor_read_angle_multiple();
  } 
  
}

std::vector<float> Robotic_arms::read_angle_data_record()
{
  // Read angle(multiple)
  read_angle_multiple();

  // Transform angle from multiple to single (S = M + offset)
  int vector_size = sizeof(motor_angles_read_multiple) / sizeof(motor_angles_read_multiple[0]);
  std::vector<float> angle(motor_angles_read_multiple, motor_angles_read_multiple + vector_size);
  for (int i = 0 ; i < vector_size ; i ++)
  {
    angle[i] += motor_angle_offset[i];
    angle[i] = round(angle[i] * 10) / 10;        
  }

  return angle;
}

std::vector<float> Robotic_arms::read_angle_current()
{
  // Read angle(multiple)
  read_angle_multiple();

  // Transform angle from multiple to single (S = M + offset)
  int vector_size = sizeof(motor_angles_read_multiple) / sizeof(motor_angles_read_multiple[0]);
  std::vector<float> current_angle(MOTOR_NUM, 0.0);
  std::vector<float> angle(motor_angles_read_multiple, motor_angles_read_multiple + vector_size);
  for (int i = 0 ; i < vector_size ; i ++)
  {
    angle[i] += motor_angle_offset[i];
    angle[i] = round(angle[i] * 10) / 10;        
  }

  // Resize angle
  for (int i = 0 ; i < vector_size ; i ++)
  {
    if (angle[i] > 180)
    {
      current_angle[i] = angle[i] - 360;
    }
    else if (angle[i] < -180)
    {
      current_angle[i] = angle[i] + 360;
    }
    else
    {
      current_angle[i] = angle[i];
    }
  }
  
  return current_angle;
}

void Robotic_arms::set_offset()
{
  read_angle_single();
  receive_3();
  receive_5();
  Serial.println();
  Serial.println("Single encoder:");
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    Serial.print(motor_angles_read_single[i]);
    Serial.print("\t");
  }
  Serial.println("");
  
  
  read_angle_multiple();
  receive_3();
  receive_5();
  Serial.println("Multiple encoder:");
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    Serial.print(motor_angles_read_multiple[i]);
    Serial.print("\t");
  }
  Serial.println("");

  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    motor_angle_offset[i] = motor_angles_read_single[i] - motor_angles_read_multiple[i];
  }
  Serial.println("Offset: ");
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    Serial.print(motor_angle_offset[i]);
    Serial.print("\t");
  }
  Serial.println("");
  
}

void Robotic_arms::initial_pose()
{
   MotorArr[6].Motor_angle_control((float(0.0 - motor_angle_offset[6]) < -180.0) ? (float(0.0 - motor_angle_offset[6]) + 360.0) : (float(0.0 - motor_angle_offset[6])));
   MotorArr[13].Motor_angle_control((float(0.0 - motor_angle_offset[13]) < -180.0) ? (float(0.0 - motor_angle_offset[13]) + 360.0) : (float(0.0 - motor_angle_offset[13])));

   MotorArr[1].Motor_angle_control((float(10.0 - motor_angle_offset[1]) < -180.0) ? (float(10.0 - motor_angle_offset[1]) + 360.0) : (float(10.0 - motor_angle_offset[1])));
   MotorArr[8].Motor_angle_control((float(-10.0 - motor_angle_offset[8]) < -180.0) ? (float(-10.0 - motor_angle_offset[8]) + 360.0) : (float(-10.0 - motor_angle_offset[8])));

//   delay(1000);

   MotorArr[2].Motor_angle_control((float(-90.0 - motor_angle_offset[2]) < -180.0) ? (float(-90.0 - motor_angle_offset[2]) + 360.0) : (float(-90.0 - motor_angle_offset[2])));
   MotorArr[9].Motor_angle_control((float(90.0 - motor_angle_offset[9]) < -180.0) ? (float(90.0 - motor_angle_offset[9]) + 360.0) : (float(90.0 - motor_angle_offset[9])));

  delay(15000);

   MotorArr[3].Motor_angle_control((float(-90.0 - motor_angle_offset[3]) < -180.0) ? (float(-90.0 - motor_angle_offset[3]) + 360.0) : (float(-90.0 - motor_angle_offset[3])));
   MotorArr[10].Motor_angle_control((float(90.0 - motor_angle_offset[10]) < -180.0) ? (float(90.0 - motor_angle_offset[10]) + 360.0) : (float(90.0 - motor_angle_offset[10])));

//   delay(1000);

   MotorArr[4].Motor_angle_control((float(80.0 - motor_angle_offset[4]) < -180.0) ? (float(80.0 - motor_angle_offset[4]) + 360.0) : (float(80.0 - motor_angle_offset[4])));
   MotorArr[11].Motor_angle_control((float(-125.0 - motor_angle_offset[11]) < -180.0) ? (float(-125.0 - motor_angle_offset[11]) + 360.0) : (float(-125.0 - motor_angle_offset[11])));

   MotorArr[0].Motor_angle_control((float(25.0 - motor_angle_offset[0]) < -180.0) ? (float(25.0 - motor_angle_offset[0]) + 360.0) : (float(25.0 - motor_angle_offset[0])));
   MotorArr[7].Motor_angle_control((float(-25.0 - motor_angle_offset[7]) < -180.0) ? (float(-25.0 - motor_angle_offset[7]) + 360.0) : (float(-25.0 - motor_angle_offset[7])));

   delay(10000);

   MotorArr[5].Motor_angle_control((float(45.0 - motor_angle_offset[5]) < -180.0) ? (float(45.0 - motor_angle_offset[5]) + 360.0) : (float(45.0 - motor_angle_offset[5])));
   MotorArr[12].Motor_angle_control((float(-45.0 - motor_angle_offset[12]) < -180.0) ? (float(-45.0 - motor_angle_offset[12]) + 360.0) : (float(-45.0 - motor_angle_offset[12])));

//   delay(1000);
}

void Robotic_arms::reset_RS485_parameters()
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    RS485_receive_buffer[i] = 0; 
    RS485_send_buffer[i] = 0; 
  }
  RS485_pkg_num = 0;
  RS485_data_num = 0;
}

void Robotic_arms::reset_USB_parameters()
{
  for (int i = 0 ; i < MOTOR_NUM ; i ++)
  {
    USB_receive_buffer[i] = 0; 
    USB_send_buffer[i] = 0; 
  }
  USB_pkg_num = 0;
  USB_data_num = 0;
  RECIEVING = false;
  RECIEVING_END = false;
}

// NOT FINISH
void Robotic_arms::receive()
{
  while(Serial.available())
  {
    byte USB_tmp = Serial.read();

    if (USB_tmp == 0xFF && USB_pkg_num == 0)                            // Confirm first header(Get angle csv data)
    {
      USB_receive_buffer[USB_pkg_num] = USB_tmp;
      RECIEVING = true;
      USB_pkg_num ++;
    }

    while (Serial.available() && RECIEVING)
    {
      byte USB_tmp = Serial.read();
      USB_receive_buffer[USB_pkg_num] = USB_tmp;
      if (USB_tmp == 0xFF && USB_pkg_num == 1)                          // Confirm second header(Get angle csv data)
      {
        USB_receive_buffer[USB_pkg_num ++] = USB_tmp;
      }
      else if (USB_pkg_num == 2)                                        // Function code
      {
        USB_receive_buffer[USB_pkg_num ++] = USB_tmp;
      }
      else if (USB_pkg_num > 2)                                         // data
      {
        USB_receive_buffer[USB_pkg_num] = USB_tmp;
        
        if (USB_receive_buffer[USB_pkg_num] == 0xFF)                    // First end code
        {
          RECIEVING_END = true;
          USB_pkg_num ++;
        }

        if (RECIEVING_END)
        {
          byte USB_tmp = Serial.read();
          USB_receive_buffer[USB_pkg_num] = USB_tmp;    
          if (USB_receive_buffer[USB_pkg_num] == 0xFE)                  // Second end code
          {
            if (USB_receive_buffer[2] == 0x01)                          // Right arm
            {
              for (int i = 0 ; i <= USB_data_num ; i += 2)
              {
                byte first_byte = USB_receive_buffer[3 + i];
                byte second_byte = USB_receive_buffer[4 + i];
                uint16_t data = (first_byte << 8) | second_byte;
                uint8_t id = (data >> 13) & 0xFF;
                uint16_t angle_data = data & 0x1FFF;
                int ID = id;
                float angle = ((angle_data * 720.0) / 8191.0) - 180.0;
                if (ID == 1)
                {
                  right_arm_csv_data.push_back(std::vector<float>(6, 0.0));
                  csv_data_num ++;
                }
                right_arm_csv_data[csv_data_num].push_back(angle);

                // // FOR TESING
                // byte data_1 = (angle_data >> 24) & 0xFF;
                // byte data_2 = (angle_data >> 16) & 0xFF;
                // byte data_3 = (angle_data >> 8) & 0xFF;
                // byte data_4 = angle_data & 0xFF;
                // Serial.write(ID);
                // Serial.write(data_1);
                // Serial.write(data_2);
                // Serial.write(data_3);
                // Serial.write(data_4);
              }
              csv_data_num = -1;
              reset_USB_parameters();
            }
            else if (USB_receive_buffer[2] == 0x02)                       // Left arm
            {
              for (int i = 0 ; i <= USB_data_num ; i += 2)
              {
                byte first_byte = USB_receive_buffer[3 + i];
                byte second_byte = USB_receive_buffer[4 + i];
                uint16_t data = (first_byte << 8) | second_byte;
                uint8_t id = (data >> 13) & 0xFF;
                uint16_t angle_data = data & 0x1FFF;
                int ID = id;
                float angle = ((angle_data * 720.0) / 8191.0) - 180.0;
                if (ID == 1)
                {
                  left_arm_csv_data.push_back(std::vector<float>(6, 0.0));
                  csv_data_num ++;
                }
                left_arm_csv_data[csv_data_num].push_back(angle);

                // // FOR TESING
                // byte data_1 = (angle_data >> 24) & 0xFF;
                // byte data_2 = (angle_data >> 16) & 0xFF;
                // byte data_3 = (angle_data >> 8) & 0xFF;
                // byte data_4 = angle_data & 0xFF;
                // Serial.write(ID);
                // Serial.write(data_1);
                // Serial.write(data_2);
                // Serial.write(data_3);
                // Serial.write(data_4);
              }
            }
            else   
            {
              reset_USB_parameters();
            }              
          }
          RECIEVING_END = false;
        }
        USB_data_num ++;
        USB_pkg_num ++;
      }
      else
      {
        RECIEVING = false;
      }
    }
    reset_USB_parameters();
  }
  reset_USB_parameters();
}

void Robotic_arms::receive_3()
{
  while(Serial3.available())
  {
    // Serial.println("Trigger SerialEvent3 loop once");
    byte RS485_tmp = Serial3.read(); 
    if (RS485_pkg_num == 0)                                       // Motor ID      
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 1)                                  // I/O 
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 2)                                  // Function code (1)
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 3)                                  // Function code (2)
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num > 3)                                   // Data
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
      RS485_data_num ++;

      if (byte(RS485_data_num) == 4)                              // While receiving specific length data
      {
        // Read CRC16
        byte CRC_HIGH = Serial3.read();
        byte CRC_LOW = Serial3.read();
        
        // Calculate CRC16
        uint16_t crc = CRC16_cal(RS485_receive_buffer, 8);
        byte crc_HIGH = (crc >> 8) & 0xFF;
        byte crc_LOW = crc & 0xFF;

        // Check sum
        if (crc_LOW == CRC_LOW && crc_HIGH == CRC_HIGH)
        {
          // Function code
          switch (RS485_receive_buffer[3])                             
          {
            // Open arms and close arms
            case 0x10:
            {
              reset_RS485_parameters();
              break;
            }

            // Return origin
            case 0x32:
            {
              reset_RS485_parameters();
              break;
            }

            // Read angle (multiple)
            case 0x13:
            {
              // Read angle and save
              float angle = RS485_receive_buffer[7] + (RS485_receive_buffer[6] << 8) + (RS485_receive_buffer[5] << 16) + (RS485_receive_buffer[4] << 24);
              angle = angle * 360.0 / (32767.0 * 101.0);
              motor_angles_read_multiple[RS485_receive_buffer[0] - 1] = angle;

              // Transform angle(float) to 4byte and save
              for (int i = 0 ; i < MOTOR_NUM ; i ++)
              {
                byte angle_hex[4];
                ie754.float_to_hex(float(motor_angles_read_multiple[i]), angle_hex);
                for (int j = 0 ; j < 4 ; j++)
                {
                  motor_angles_read_multiple_bytes[i * 4 + j] = angle_hex[j];
                }
              }
              reset_RS485_parameters();
              break;
            }

            // Read angle (Single)
            case 0x15:
            { 
              // Read angle and save
              float angle = RS485_receive_buffer[7] + (RS485_receive_buffer[6] << 8) + (RS485_receive_buffer[5] << 16) + (RS485_receive_buffer[4] << 24);
              angle = angle * 360.0 / 32767.0;
              motor_angles_read_single[RS485_receive_buffer[0] - 1] = angle;

              // Transform angle(float) to 4byte and save
              for (int i = 0 ; i < MOTOR_NUM ; i ++)
              {
                byte angle_hex[4];
                ie754.float_to_hex(float(motor_angles_read_single[i]), angle_hex);
                for (int j = 0 ; j < 4 ; j++)
                {
                  motor_angles_read_single_bytes[i * 4 + j] = angle_hex[j];
                }
              }
              reset_RS485_parameters();
              break;
            }

            // Control six motors angle
            case 0x82:
            {
             reset_RS485_parameters();
             break; 
            }

            case 0x2E:
            {
              reset_RS485_parameters();
              break;
            }

            default:
            {
              reset_RS485_parameters();
              break;
            }
          }
        }
        // When check sum is not pass
        else
        {
          reset_RS485_parameters();
        }
      }
    }
  }
  reset_RS485_parameters();
}

void Robotic_arms::receive_5()
{
  while(Serial5.available())
  {
    // Serial.println("Trigger SerialEvent5 loop once");
    byte RS485_tmp = Serial5.read(); 
    if (RS485_pkg_num == 0)                                       // Motor ID      
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 1)                                  // I/O 
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 2)                                  // Function code (1)
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num == 3)                                  // Function code (2)
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
    }
    else if (RS485_pkg_num > 3)                                   // Data
    {
      RS485_receive_buffer[RS485_pkg_num ++] = RS485_tmp;
      RS485_data_num ++;

      if (byte(RS485_data_num) == 4)                              // While receiving specific length data
      {
        // Read CRC16
        byte CRC_HIGH = Serial5.read();
        byte CRC_LOW = Serial5.read();
        
        // Calculate CRC16
        uint16_t crc = CRC16_cal(RS485_receive_buffer, 8);
        byte crc_HIGH = (crc >> 8) & 0xFF;
        byte crc_LOW = crc & 0xFF;

        // Check sum
        if (crc_LOW == CRC_LOW && crc_HIGH == CRC_HIGH)
        {
          // Function code
          switch (RS485_receive_buffer[3])                             
          {
            // Open arms and close arms
            case 0x10:
            {
              reset_RS485_parameters();
              break;
            }

            // Return origin
            case 0x32:
            {
              reset_RS485_parameters();
              break;
            }

            // Read angle (multiple)
            case 0x13:
            {
              // Read angle and save
              float angle = RS485_receive_buffer[7] + (RS485_receive_buffer[6] << 8) + (RS485_receive_buffer[5] << 16) + (RS485_receive_buffer[4] << 24);
              angle = angle * 360.0 / (32767.0 * 101.0);
              motor_angles_read_multiple[RS485_receive_buffer[0] - 1] = angle;

              // Transform angle(float) to 4byte and save
              for (int i = 0 ; i < MOTOR_NUM ; i ++)
              {
                byte angle_hex[4];
                ie754.float_to_hex(float(motor_angles_read_multiple[i]), angle_hex);
                for (int j = 0 ; j < 4 ; j++)
                {
                  motor_angles_read_multiple_bytes[i * 4 + j] = angle_hex[j];
                }
              }
              reset_RS485_parameters();
              break;
            }

            // Read angle
            case 0x15:
            { 
              // Read angle and save
              float angle = RS485_receive_buffer[7] + (RS485_receive_buffer[6] << 8) + (RS485_receive_buffer[5] << 16) + (RS485_receive_buffer[4] << 24);
              angle = angle * 360.0 / 32767.0;
              motor_angles_read_single[RS485_receive_buffer[0] - 1] = angle;

              // Transform angle(float) to 4byte and save
              for (int i = 0 ; i < MOTOR_NUM ; i ++)
              {
                byte angle_hex[4];
                ie754.float_to_hex(float(motor_angles_read_single[i]), angle_hex);
                for (int j = 0 ; j < 4 ; j++)
                {
                  motor_angles_read_single_bytes[i * 4 + j] = angle_hex[j];
                }
              }
              reset_RS485_parameters();
              break;
            }

            // Control six motors angle
            case 0x82:
            {
             reset_RS485_parameters();
             break; 
            }

            case 0x2E:
            {
              reset_RS485_parameters();
              break;
            }

            default:
            {
              reset_RS485_parameters();
              break;
            }
          }
        }
        // When check sum is not pass
        else
        {
          reset_RS485_parameters();
        }
      }
    }
  }
  reset_RS485_parameters();
}

//CRC16 calculation
uint16_t Robotic_arms::CRC16_cal(uint8_t *data, int len)
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
