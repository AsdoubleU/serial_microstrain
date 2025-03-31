#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "Eigen/Dense"

#define RAD2DEG 57.2957914
#define DEG2RAD 0.01745329

double control_period = 0.01;
double control_time = 0.0;

Eigen::Quaterniond quaternion_;
Eigen::Vector3d orientation_, angular_velocity_, linear_acceleration_;
Eigen::Matrix3d rotation_matrix_;

ros::Publisher p_imu_roll;
ros::Publisher p_imu_pitch;
ros::Publisher p_imu_yaw;

ros::Publisher p_imu_ang_vel_x;
ros::Publisher p_imu_ang_vel_y;
ros::Publisher p_imu_ang_vel_z;

ros::Publisher p_imu_lin_acc_x;
ros::Publisher p_imu_lin_acc_y;
ros::Publisher p_imu_lin_acc_z;

std_msgs::Float64 m_imu_roll;
std_msgs::Float64 m_imu_pitch;
std_msgs::Float64 m_imu_yaw;

std_msgs::Float64 m_imu_ang_vel_x;
std_msgs::Float64 m_imu_ang_vel_y;
std_msgs::Float64 m_imu_ang_vel_z;

std_msgs::Float64 m_imu_lin_acc_x;
std_msgs::Float64 m_imu_lin_acc_y;
std_msgs::Float64 m_imu_lin_acc_z;

enum Pos
{
    X,
    Y,
    Z,
    RP,
    ROLL = 3,
    PITCH = 4,
    YAW = 5,
    Pos = 6,
    XYZ = 3
};

// Fletcher checksum function
uint16_t FletcherChecksum(const uint8_t* packet, int packet_length) {
    const int checksum_length = packet_length - 2;
    uint8_t checksum_MSB = 0;
    uint8_t checksum_LSB = 0;
    for(int i = 0; i < checksum_length; i++) {
        checksum_MSB += packet[i];
        checksum_LSB += checksum_MSB;
    }
    return ((uint16_t)checksum_MSB << 8) | (uint16_t)checksum_LSB;
}

template <typename T>
T ReadBigEndian(const uint8_t* data) {
    constexpr size_t size = sizeof(T);
    uint8_t reversed[size];

    for (size_t i = 0; i < size; ++i) {
        reversed[i] = data[size - 1 - i];  // big endian → little endian
    }

    T value;
    memcpy(&value, reversed, size);
    return value;
}