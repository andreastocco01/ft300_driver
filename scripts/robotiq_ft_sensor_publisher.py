import rospy
import socket
from geometry_msgs.msg import WrenchStamped, Wrench
from std_msgs.msg import Header

REMOTE_IP = "192.168.53.3"
REMOTE_PORT  = 63351
REMOTE_ADDR = (REMOTE_IP, REMOTE_PORT)

def main():
    # Create socket and connect to remote address
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(REMOTE_ADDR)

    # Create a publisher and a new node in ROS
    publisher = rospy.Publisher('sensor_topic', WrenchStamped, queue_size=1)
    rospy.init_node("robotiq_ft_sensor_publisher")
    # Set rate to run (100hz for robotiq)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        # Read 71 bytes from tcp stream
        buffer, addr = sock.recvfrom(71)
        
        # Remove '(' ')' from buffer string and split each value
        values = str(buffer, "ascii").strip("()").split(",")             

        # Convert values to float numbers
        fx = float(values[0])
        fy = float(values[1])
        fz = float(values[2])
        mx = float(values[3])
        my = float(values[4])
        mz = float(values[5])

        # DEBUG: print float values
        print(f"{{ {fx}, {fy}, {fz}, {mx}, {my}, {mz} }}")
        print("\n\n") 

        # Create message parts
        message = WrenchStamped()
        header = Header()
        wrench = Wrench()
        
        # Construct header by setting frame_id and stamp
        header.frame_id = "robotiq_ft_frame_id"
        header.stamp = rospy.Time.now()

        # Construct force vector
        wrench.force.x = fx
        wrench.force.y = fy
        wrench.force.z = fz
        # Construct torque vector
        wrench.torque.x = mx
        wrench.torque.y = my
        wrench.torque.z = mz

        # Set message header and wrench
        message.header = header
        message.wrench = wrench

        # DEBUG: log values
        rospy.loginfo(values)
        # Publish WrenchStamped message to topic and sleep
        publisher.publish(message)
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

