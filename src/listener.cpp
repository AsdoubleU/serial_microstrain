
#include "listener.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "microstrain_inertial_listener");
    ros::NodeHandle node_obj;

    const ros::Duration control_period_(control_period);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_control_time = ros::Time::now();

    // Serial setup
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);

    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    tcsetattr(serial_port, TCSANOW, &tty);

    std::vector<uint8_t> buffer;

    p_imu_roll = node_obj.advertise<std_msgs::Float64>("/imu_roll/",1);
    p_imu_pitch = node_obj.advertise<std_msgs::Float64>("/imu_pitch/",1);
    p_imu_yaw = node_obj.advertise<std_msgs::Float64>("/imu_yaw/",1);

    p_imu_ang_vel_x = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_x/",1);
    p_imu_ang_vel_y = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_y/",1);
    p_imu_ang_vel_z = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_z/",1);

    p_imu_lin_acc_x = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_x/",1);
    p_imu_lin_acc_y = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_y/",1);
    p_imu_lin_acc_z = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_z/",1);

    p_imu_magneto_x = node_obj.advertise<std_msgs::Float64>("/imu_magneto_x/",1);
    p_imu_magneto_y = node_obj.advertise<std_msgs::Float64>("/imu_magneto_y/",1);
    p_imu_magneto_z = node_obj.advertise<std_msgs::Float64>("/imu_magneto_z/",1);

    p_imu_magneto_yaw = node_obj.advertise<std_msgs::Float64>("/imu_magneto_yaw/",1);

    while (ros::ok()) {

        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_control_time;

        last_control_time = current_time;
        control_time += elapsed_time.toSec();

        // read from serial
        char temp_buf[256];
        int num_bytes = read(serial_port, &temp_buf, sizeof(temp_buf));
        if (num_bytes > 0) { buffer.insert(buffer.end(), temp_buf, temp_buf + num_bytes); }

        // packet parsing loop
        while (buffer.size() >= 4) {

            if (buffer[0] != 0x75 || buffer[1] != 0x65) {
                buffer.erase(buffer.begin());
                continue;
            }

            if (buffer.size() < 4) break;

            uint8_t payload_len = buffer[3];
            size_t full_packet_size = 4 + payload_len + 2;

            if (buffer.size() < full_packet_size) break;

            std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + full_packet_size);

            // Check checksum
            uint16_t calculated_checksum = FletcherChecksum(packet.data(), full_packet_size);
            uint16_t received_checksum = ((uint16_t)packet[full_packet_size - 2] << 8) | packet[full_packet_size - 1];

            if (calculated_checksum != received_checksum) {
                buffer.erase(buffer.begin(), buffer.begin() + full_packet_size);
                continue;
            }

            if (packet[2] == 0x80) {

                magnetometer(X) = ReadBigEndian<float>(&packet[6]);
                magnetometer(Y) = - ReadBigEndian<float>(&packet[10]);
                magnetometer(Z) = - ReadBigEndian<float>(&packet[14]);
                    
                angular_velocity_(X) = ReadBigEndian<float>(&packet[20]);
                angular_velocity_(Y) = - ReadBigEndian<float>(&packet[24]);
                angular_velocity_(Z) = - ReadBigEndian<float>(&packet[28]);

                count1++;
            }

            if (packet[2] == 0x82) {

                orientation_(X) = ReadBigEndian<float>(&packet[6]);
                orientation_(Y) = - ReadBigEndian<float>(&packet[10]);
                orientation_(Z) = - ReadBigEndian<float>(&packet[14]);

                linear_acceleration_(X) = ReadBigEndian<float>(&packet[62]);
                linear_acceleration_(Y) = - ReadBigEndian<float>(&packet[66]);
                linear_acceleration_(Z) = - ReadBigEndian<float>(&packet[70]);

                rotation_matrix_(0, 0) = ReadBigEndian<float>(&packet[22]);
                rotation_matrix_(0, 1) = ReadBigEndian<float>(&packet[26]);
                rotation_matrix_(0, 2) = ReadBigEndian<float>(&packet[30]);
                rotation_matrix_(1, 0) = ReadBigEndian<float>(&packet[34]);
                rotation_matrix_(1, 1) = ReadBigEndian<float>(&packet[38]);
                rotation_matrix_(1, 2) = ReadBigEndian<float>(&packet[42]);
                rotation_matrix_(2, 0) = ReadBigEndian<float>(&packet[46]);
                rotation_matrix_(2, 1) = ReadBigEndian<float>(&packet[50]);
                rotation_matrix_(2, 2) = ReadBigEndian<float>(&packet[54]);

                // for(size_t i=0;i<payload_len + 6;i++){
                //     printf("%02X ",packet[i]);
                // }
                // printf("\n");
                // std::cout<<std::endl;

                count2++;
            }

            float mag_x = magnetometer(X)*cos(orientation_(Y)) 
                          + magnetometer(Y)*sin(orientation_(X))*sin(orientation_(Y)) + magnetometer(Z)*cos(orientation_(X))*sin(orientation_(Y));
            float mag_y = magnetometer(Y)*cos(orientation_(X)) - magnetometer(Z)*sin(orientation_(X));
            float mag_z = atan2(-mag_y, mag_x);
            m_imu_magneto_yaw.data = mag_z;

            std::cout<<"=========== Rotation Matrix ==========="<<std::endl;
            std::cout<<rotation_matrix_<<std::endl;
            std::cout<<"======================================="<<std::endl;
            std::cout<<"Control frequency of IMU raw data :      "<<displat_count1<<" Hz"<<std::endl;
            std::cout<<"Control frequency of Filtered IMU data : "<<displat_count2<<" Hz"<<std::endl;
            std::cout<<std::endl;

            buffer.erase(buffer.begin(), buffer.begin() + full_packet_size);
        }

        m_imu_roll.data = orientation_(X); m_imu_pitch.data = orientation_(Y); m_imu_yaw.data = orientation_(Z);
        p_imu_roll.publish(m_imu_roll); p_imu_pitch.publish(m_imu_pitch); p_imu_yaw.publish(m_imu_yaw);

        m_imu_ang_vel_x.data = angular_velocity_(X); m_imu_ang_vel_y.data = angular_velocity_(Y); m_imu_ang_vel_z.data = angular_velocity_(Z);
        p_imu_ang_vel_x.publish(m_imu_ang_vel_x); p_imu_ang_vel_y.publish(m_imu_ang_vel_y); p_imu_ang_vel_z.publish(m_imu_ang_vel_z);
            
        m_imu_lin_acc_x.data = linear_acceleration_(X);  m_imu_lin_acc_y.data = linear_acceleration_(Y);  m_imu_lin_acc_z.data = linear_acceleration_(Z);
        p_imu_lin_acc_x.publish(m_imu_lin_acc_x); p_imu_lin_acc_y.publish(m_imu_lin_acc_y); p_imu_lin_acc_z.publish(m_imu_lin_acc_z);

        m_imu_magneto_x.data = magnetometer(X); m_imu_magneto_y.data = magnetometer(Y); m_imu_magneto_z.data = magnetometer(Z);
        p_imu_magneto_x.publish(m_imu_magneto_x); p_imu_magneto_y.publish(m_imu_magneto_y); p_imu_magneto_z.publish(m_imu_magneto_z);

        p_imu_magneto_yaw.publish(m_imu_magneto_yaw);

        if(control_time >= 1.){ displat_count1 = count1; displat_count2 = count2; count1 = 0.; count2 = 0.; control_time = 0.; }

        ros::spinOnce();
    }

    close(serial_port);
    return 0;
}
