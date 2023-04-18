import socket

import rospy
from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Header

REMOTE_IP = "192.168.53.3"
REMOTE_PORT  = 63351
REMOTE_ADDR = (REMOTE_IP, REMOTE_PORT)

def main():
    # Create socket and connect to remote address
    local_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    local_socket.connect(REMOTE_ADDR)

    # Create a publisher and a new node in ROS
    publisher = rospy.Publisher('sensor_topic', WrenchStamped, queue_size=1)
    rospy.init_node("robotiq_ft_sensor_publisher")
    # Set rate to run (100hz for robotiq)
    rate = rospy.Rate(100)

    # Create message parts
    wrench_message = WrenchStamped()
    header = Header()
    wrench = Wrench()
    header.frame_id = "robotiq_ft_frame_id"

    # Create a varaible to hold incomplete packets
    last_message = ""
    while not rospy.is_shutdown():

        # Read bytes from tcp stream and convert to ascii string
        buffer, addr = local_socket.recvfrom(100)
        message = buffer.decode("ascii")

        # We want to split the message in packets, but we need to track the peackets incomplete '(...' or '...)'
        message = message.replace(")", ")|")    # So we add another delimiter between packets
        reading_list = message.split("|")       # Now we split using our delimiter, which will be removed

        # We can now inspect the first packet
        if (reading_list[0].count("(") == 0):
            # if the first packet doesn't have the opening braket, we compose it with the last packet of the previus reading from socket
            reading_list[0] = last_message + reading_list[0]
            last_message = "" #last message has been used, we clean it
        # Now we inspect the last packet
        if (reading_list[-1].count(")") == 0):
            # if the last packet doesn't have the closing bracket, we save it for later use
            last_message = reading_list[-1]
            reading_list.remove(reading_list[-1]) # and we remove the incomplete packet from the reading list. It will be used in the next reading.

        # Now we scan each packet
        for reading in reading_list:
            
            # Make sure each reading is now fully composed
            if reading.count("(") == 0 or reading.count(")") == 0:
                rospy.loginfo(f"MALFORMED packet is missing brackets, printing buffer, list and readings:\n" +
                              f"BUFFER:{message}\n\n" +
                              f"Reading list:{reading_list}\n\n" +
                              f"Reading:{reading}")
                # We don't send this reading
                continue

            # We can finally remove brakets
            reading = reading.replace("(", "").replace(")", "")
            # Now we can split safely, and convert each value to float value
            values = list(map(float, reading.split(" , ")))
            
            # We make sure each packet has 6 values
            if len(values) != 6 :
                rospy.loginfo(f"MALFORMED packet is missing values, printing buffer, list and readings:\n" +
                              f"BUFFER:{message}\n\n" +
                              f"Reading list:{reading_list}\n\n" +
                              f"Reading:{reading}\n" +
                              f"has been mapped to\n" +
                              f"{values}")
                # We don't send this reading
                continue

            # Now we need to construct the message 
            # Construct force vector
            wrench.force.x = values[0]
            wrench.force.y = values[1]
            wrench.force.z = values[2]
            #Construct torque vector
            wrench.torque.x = values[3]
            wrench.torque.y = values[4]
            wrench.torque.z = values[5]
            # Construct header's stamp
            header.stamp = rospy.Time.now()

            # Set message header and wrench
            wrench_message.header = header
            wrench_message.wrench = wrench

            # Publish WrenchStamped message to topic and sleep
            publisher.publish(wrench_message)

        # We sent packets received, we can read again at given ros frequency
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

