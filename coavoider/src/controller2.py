#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
import math
from listener import Listener
from tf import transformations


garbage_1=[[0,2],[0,4],[5,4],[5,1],[5,4],[0,4],[-4.66,4.11],[-4.66,7]]  #paths from starting point 
garbage_2=[[0,2],[0,4],[5,4],[11,4],[11,1],[11,4],[5,4],[0,4],[-4.66,4.11],[-4.66,7]]
garbage_3=[[0,2],[0,4],[5,4],[5,7],[5,4],[0,4],[-4.66,4.11],[-4.66,7]]
garbage_4=[[0,2],[0,4],[5,4],[11,4],[11,7],[11,4],[5,4],[0,4],[-4.66,4.11],[-4.66,7]]

medical_1=[[0,2],[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[5,1]]
medical_2=[[0,2],[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[11,4],[11,1]]
medical_3=[[0,2],[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[5,7]]
medical_4=[[0,2],[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[11,4],[11,7]]
current_path =[]
path_counter = 0 #step of the path

waypoint_x =0 #initial waypoint
waypoint_y =0

temp_room =''
temp_job =''
state = "Listening"#initial state

if state=="Listening":#take voice input from the user
	listener = Listener()
	listener.start()
q_prev = 0
if __name__ == '__main__':
    global state
    rospy.init_node("safe_driver")
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    tf_listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
        	(translation,orientation) = tf_listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        #print("Robot is believed to have orientation (theta): (",robot_theta,")\n")
	
	motor_command=Twist()

	temp_room = listener.room #voice mistake corrections
	temp_job=listener.job
	if temp_room == 'wall':
	    temp_room = 'one'
	if temp_room == 'to':
	    temp_room = 'two'
	if temp_room == 'for':
	    temp_room = 'four'
	if temp_room == 'tree':
	    temp_room = 'three'
	if temp_room == '1':
	    temp_room = 'one'
	if temp_room == '2':
	    temp_room = 'two'
	if temp_room == '3':
	    temp_room = 'three'
	if temp_room == '4':
	   temp_room = 'four'
	print("Temp room:",temp_room," temp job",temp_job)
	if (temp_room=="one" or temp_room =="two" or temp_room =="three" or temp_room =="four") and (temp_job=="garbage" or temp_job=="medical") and (state =="Listening"):
		print("Going to room-",temp_room,"for",temp_job) #select path from voice input
		if(temp_job=="garbage" and temp_room =="one"):
			current_path = garbage_1
		elif(temp_job=="garbage" and temp_room =="two"):
			current_path = garbage_2
		elif(temp_job=="garbage" and temp_room =="three"):
			current_path = garbage_3
		elif(temp_job=="garbage" and temp_room =="four"):
			current_path = garbage_4
		elif(temp_job=="medical" and temp_room =="one"):
			current_path = medical_1
		elif(temp_job=="medical" and temp_room =="two"):
			current_path = medical_2
		elif(temp_job=="medical" and temp_room =="three"):
			current_path = medical_3
		elif(temp_job=="medical" and temp_room =="four"):
			current_path = medical_4

		waypoint_x = current_path[path_counter][0] #assign waypoint
		waypoint_y = current_path[path_counter][1]

		listener.stop = True
		state = "Moving"


	dx = waypoint_x - translation[0]
	dy = waypoint_y - translation[1]

	xr = math.cos(robot_theta) * dx + math.sin(robot_theta) * dy
	yr = math.cos(robot_theta) * dy - math.sin(robot_theta) * dx
		
	p = math.hypot(xr, yr)
	q = math.atan2(yr, xr)
	if p<0.1 and state == "Moving": ## arrived to waypoint
		path_counter = path_counter + 1 #select next waypoint from path
		waypoint_x = current_path[path_counter][0]
		waypoint_y = current_path[path_counter][1]
	if(state =="Moving"):
		print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")
		print("Robot is going to (x,y): (",waypoint_x,",",waypoint_y,")")
		if p>1: #max dist for linear speed
			p = 1
		motor_command.linear.x=0.3*p*math.cos(abs(q))
		motor_command.angular.z=0.9*q + 0.5*(q-q_prev)
		print("linear speed x is:",motor_command.linear.x)
		print("angular speed z is:",motor_command.angular.z)
		q_prev = q
		motor_command_publisher.publish(motor_command)
	else:
		motor_command.linear.x=0
		motor_command.angular.z=0
		print("linear speed x is:",motor_command.linear.x)
		print("angular speed z is:",motor_command.angular.z)
		motor_command_publisher.publish(motor_command)
	


        #I don't know what to do because nobody has programmed me with any smartness,
        #so I'll do what everybody does, and drive very fast straight forwards.
        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
