import rospy
from std_msgs.msg import String,Int16MultiArray
import sys
import select
import termios
import tty


velocity = 0
steering = 195

Min_Steering = 130
Max_steeromg = 250

MAX_Velocity = 255


pub = rospy.Publisher('/control_data', Int16MultiArray, queue_size=1)


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    settings = termios.tcgetattr(sys.stdin)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop():
    global velocity,steering

    rospy.init_node('keyboard_publisher')
    rate = rospy.Rate(10)

    status = 0

    while not rospy.is_shutdown():
        pubmsg = Int16MultiArray()
        key = getKey()

        if key == 'w':
            velocity = velocity + 5
            steering = 0 
            status = status + 1

            
        elif key == 's':
            velocity = 0
            #steering = 0
            status = status + 1
        elif key == 'a':
            steering = steering - 2
            status = status + 1
        elif key == 'd':
            steering = steering + 2
            status = status + 1
        elif key == 'x':
            velocity = velocity - 5
            steering = 0
            status = status + 1
        else:
            if (key == '\x03'):
                break
        pubmsg.data = [velocity,steering]
        
        pub.publish(pubmsg)

        print('cmd : ' + str(velocity) + ','+ str(steering))
    rospy.spin()


if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass


    
    
