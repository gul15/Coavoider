#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from geometry_msgs.msg import *
import tf
import math
from gazebo_msgs.srv import DeleteModel,SpawnModel
from listener import Listener
from tf import transformations
def delete_model(model):
	delete_model_prox = rospy.ServiceProxy('gazebo/delete_model',DeleteModel)
	delete_model_prox(model)
q_prev = 0
room_1=[5,1] #room center coordinates [x,y]
room_2=[11,1]
room_3=[5,7]
room_4=[11,7]
garbage_room = [-4.66,7]
medical_room = [-5,1]

garbage_1=[[0,4],[5,4],[5,1],[5,4],[-4.66,4.11],[-4.66,7],[-4.66,4.11],[0,4],[0,0]]  			#paths from starting point
garbage_2=[[0,4],[11,4],[11,1],[11,4],[0,4],[-4.66,4.11],[-4.66,7],[-4.66,4.11],[0,4],[0,0]]
garbage_3=[[0,4],[5,4],[5,7],[5,4],[-4.66,4.11],[-4.66,7],[-4.66,4.11],[0,4],[0,0]]
garbage_4=[[0,4],[11,4],[11,7],[11,4],[0,4],[-4.66,4.11],[-4.66,7],[-4.66,4.11],[0,4],[0,0]]

medical_1=[[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[5,1],[5,4],[0,4],[0,0]]
medical_2=[[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[11,4],[11,1],[11,4],[0,4],[0,0]]
medical_3=[[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[5,4],[5,7],[5,4],[0,4],[0,0]]
medical_4=[[0,4],[-4.66,4.11],[-5,1],[-4.66,4.11],[0,4],[11,4],[11,7],[11,4],[0,4],[0,0]]
current_path =[]
path_counter = 0 #step of the path

waypoint_x =0 #initial waypoint
waypoint_y =0

temp_room =''
temp_job =''
state = "Listening"#initial state
if __name__ == '__main__':
    global state
    if state=="Listening":#take voice input from the user
	print('Give me a job and a room number.(Ex:Garbage three or Medical one')
	listener = Listener()
	listener.start()
    rospy.init_node("safe_driver")
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    tf_listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
        	(translation,orientation) = tf_listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        #print("Robot is believed to have orientation (theta): (",robot_theta,")\n")
	
	motor_command=Twist()

	temp_room = listener.room #voice mistake corrections
	temp_job=listener.job
	if temp_room == '1' or temp_room == 'wall':
	    temp_room = 'one'
	if temp_room == '2' or temp_room == 'to':
	    temp_room = 'two'
	if temp_room == '3'or temp_room == 'tree':
	    temp_room = 'three'
	if temp_room == '4'or temp_room == 'for':
	   temp_room = 'four'
	#print("Temp room:",temp_room," temp job",temp_job)
	if (temp_room=="one" or temp_room =="two" or temp_room =="three" or temp_room =="four") and (temp_job=="garbage" or temp_job=="medical") and (state =="Listening"):
		listener.stop = True
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
		if temp_job == "garbage":
			print('Taking the garbage of room ',temp_room)
		elif temp_job == "medical":
			print('Bringing medical supply to room ',temp_room)
		job= temp_job
		room= temp_room
		waypoint_x = current_path[path_counter][0] #assign waypoint
		waypoint_y = current_path[path_counter][1]
		state = "Moving"

        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis

	diff_x = waypoint_x - translation[0]
	diff_y = waypoint_y - translation[1]

	new_x = math.cos(robot_theta) * diff_x + math.sin(robot_theta) * diff_y #new plane found with rotation matrix
	new_y = math.cos(robot_theta) * diff_y - math.sin(robot_theta) * diff_x

	p = math.hypot(new_x, new_y) #distance between robot and waypoint
	q = math.atan2(new_y, new_x) #angle between robot and waypoint

	if p<0.1 and state == "Moving": ## arrived to waypoint
		currentroom = [waypoint_x,waypoint_y]
		print("Reached waypoint:",path_counter)
		print("Robot currently at",translation[0],",",translation[1])
		if(waypoint_x==garbage_room[0] and waypoint_y==garbage_room[1]):
			print('Garbage cleared.')
		if(waypoint_x==medical_room[0] and waypoint_y==medical_room[1]):
			if room == "one": 
				model_to_delete = 'medical_1'
			if room == "two": 
				model_to_delete = 'medical_2'
			if room == "three": 
				model_to_delete = 'medical_3'
			if room == "four": 
				model_to_delete = 'medical_4'
			delete_model(model_to_delete)
			print('Medicine taken.')
		if((currentroom == room_1 or currentroom == room_2 or currentroom == room_3 or currentroom == room_4)and job == "medical"):
			print('Medicine delivered.')
		if((currentroom == room_1 or currentroom == room_2 or currentroom == room_3 or currentroom == room_4) and job == "garbage"):
			if currentroom == room_1: 
				model_to_delete = 'garbage_1'
			if currentroom == room_2: 
				model_to_delete = 'garbage_2'
			if currentroom == room_3: 
				model_to_delete = 'garbage_3'
			if currentroom == room_4: 
				model_to_delete = 'garbage_4'
			delete_model(model_to_delete)
			print('Garbage taken.')
		path_counter = path_counter + 1 #select next waypoint from path
		if path_counter == len(current_path):
			print('Finished. Listening for new instructions.')
			listener.stop = False
			temp_room =''
			temp_job =''
			path_counter = 0
			state = "Listening"
		waypoint_x = current_path[path_counter][0]
		waypoint_y = current_path[path_counter][1]
	if(state =="Moving"):
		#print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")
		#print("Robot is going to (x,y): (",waypoint_x,",",waypoint_y,")")
		if p>1: #max dist for linear speed
			p = 1
		motor_command.linear.x=0.4*p*math.cos(abs(q))
		motor_command.angular.z=0.9*q + 0.5*(q-q_prev)
		q_prev = q
		#print("linear speed x is:",motor_command.linear.x)
		#print("angular speed z is:",motor_command.angular.z)
		motor_command_publisher.publish(motor_command)
	else:
		motor_command.linear.x=0
		motor_command.angular.z=0
		#print("linear speed x is:",motor_command.linear.x)
		#print("angular speed z is:",motor_command.angular.z)
		motor_command_publisher.publish(motor_command)


    
        #delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
