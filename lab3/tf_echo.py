import rospy
import tf2_ros
import sys


def echo(first_frame, second_frame):
    r = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)


    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(first_frame, second_frame, rospy.Time())
            # Process trans to get your state error
            # Generate a control command to send to the robot
            print(trans.transform)
    


            #################################### end your code ###############
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        # Use our rate object to sleep until it is time to publish again
        r.sleep()

if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    echo(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
    pass