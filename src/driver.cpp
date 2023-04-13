#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string.h>

void parse(char* buff, char** res) {
    char delim [] = "() ,";
    char* ptr = strtok(buff, delim);
    int i = 0;
    while(ptr != NULL) {
        res[i++] = ptr;
        ptr = strtok(NULL, delim);
    }
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

    ros::Rate rate(100); // ft300 frequency

    char buff [72]; // 71 (size of ft300 message) + 1 (size of \0 character)
    int r;

    ros::Publisher publisher = node.advertise<geometry_msgs::WrenchStamped>("sensor_topic", 10);
    geometry_msgs::Vector3 force;
    geometry_msgs::Vector3 torque;
    geometry_msgs::WrenchStamped message;

    while(ros::ok() && (r = read(socketfd, buff, 71)) != -1) {
        buff[r] = 0; // c string terminator char

        char* buff_parsed [6];

        parse(buff, buff_parsed);

        // parsing sensor values
        force.x = atof(buff_parsed[0]);
        force.y = atof(buff_parsed[1]);
        force.z = atof(buff_parsed[2]);

        torque.x = atof(buff_parsed[3]);
        torque.y = atof(buff_parsed[4]);
        torque.z = atof(buff_parsed[5]);

        message.wrench.force = force;
        message.wrench.torque = torque;
        message.header.frame_id = "robotiq_ft_frame_id";
        message.header.stamp = ros::Time::now();
        publisher.publish(message);

        rate.sleep();
    }

    return 0;
}