#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <geometry_msgs/Wrench.h>

/*
 * buff[0] = 0x20
 * buff[1] = 0x4E
 * buff[2] = Fx * 100 (Least Significant Bit)
 * buff[3] = Fx * 100 (Most Significant Bit)
 * buff[4] = Fy * 100
 * buff[5] = Fy * 100
 * buff[6] = Fz * 100
 * buff[7] = Fz * 100
 * buff[8] = Mx * 1000
 * buff[9] = Mx * 1000
 * buff[10] = My * 1000
 * buff[11] = My * 1000
 * buff[12] = Mz * 1000
 * buff[13] = Mz * 1000
 * buff[14] = LSB crc
 * buff[15] = MSB crc
 *
 */

// Compute the MODBUS RTU CRC
unsigned short crc_calculator (char buff[], int len) {
    unsigned short crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned short) buff[pos];          // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {    // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else                            // Else LSB is not set
                crc >>= 1;                    // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "driver");
    ros::NodeHandle node;

    int socketfd = socket(AF_INET, SOCK_STREAM, 0);
    if(socketfd == -1) {
        ROS_INFO("Socket Failed");
        return -1;
    }

    sockaddr_in robot_address;
    robot_address.sin_family = AF_INET;
    robot_address.sin_port = htons(63351);
    robot_address.sin_addr.s_addr = htonl(192 << 24 | 168 << 16 | 53 << 8 | 3); // 192.168.53.3

    int is_connected = connect(socketfd, (sockaddr*) &robot_address, sizeof(sockaddr_in));
    if (is_connected == -1) {
        ROS_INFO("Connection Failed");
        return -1;
    }

    // is necessary write 0xff 50 times to the sensor?

    ros::Rate rate(100); // ft300 frequency
    char buff [17]; // 16 (size of ft300 message) + 1 (size of \0 character)
    int r;
    ros::Publisher publisher = node.advertise<geometry_msgs::Wrench>("sensor_topic", 10);
    geometry_msgs::Vector3 force;
    geometry_msgs::Vector3 torque;
    geometry_msgs::Wrench message;
    while(ros::ok() && (r = read(socketfd, buff, 16)) != -1) {
        buff[r] = 0; // c string terminator char
        unsigned short crc = crc_calculator(buff, 14);
        unsigned char* lsb = (unsigned char*) &crc;
        unsigned char* msb = lsb + 1;
        if (*lsb == buff[14] && *msb == buff[15]) { // crc check
            ROS_INFO("No Transmission Errors");
            // parsing sensor values
            force.x = buff[3] << 2 | buff[2];
            force.y = buff[5] << 2 | buff[4];
            force.z = buff[7] << 2 | buff[6];

            torque.x = buff[9] << 2 | buff[8];
            torque.y = buff[11] << 2 | buff[10];
            torque.z = buff[13] << 2 | buff[12];

            message.force = force;
            message.torque = torque;
            publisher.publish(message);
        }
        rate.sleep();
    }
    return 0;
}