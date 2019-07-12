# -*- coding:utf-8 -*-
from pymavlink import mavutil
import time
import dronetools
import math
import thread

#status = 'p'
drone1 = dronetools.drone('/dev/ttyAMA0', 921600, 1)


#def get_status_from_keyboard(self):
#    #global self.status
#    while True:
#        key_board_input = raw_input()
#        if key_board_input == 'c' or key_board_input == 'p' or key_board_input == 'l' or key_board_input == 's' or key_board_input == 'L':
#            if self.status != 'p' and key_board_input != 'p' and key_board_input != 'l':
#                print('You have change to point status first!!')
#                continue
#            else:
#                self.status = key_board_input
#        else:
#            print('input error!!')

def send_drone_position_under_optitrack(drone):
    while True:
        drone1.get_position_from_optitrack_UDP()
        drone1.send_position_data_to_PX4()






#while True:
#    drone1.the_connection.wait_heartbeat()
#    if drone1.the_connection.target_system == 1:
#        break
print("connection established!!")
print("Heartbeat from system (system %u component %u)" % (drone1.the_connection.target_system, drone1.the_connection.target_component))


drone1.connect_to_optitrack_UDP('192.168.50.3',31500)
#drone1.connect_to_optitrack_UDP('172.18.84.212',31500)
drone1.get_command_from_keyboard()
drone1.get_apriltag_data()
#drone1.connect_to_optitrack_UDP('172.18.91.167',31500)


#for i in xrange(0,100):
#thread.start_new_thread(get_status_from_keyboard,(drone1, ))

thread.start_new_thread(send_drone_position_under_optitrack,(drone1,))
for i in xrange(0,100):
    drone1.set_target_position_under_optitrack_UDP(0.5 , 0.5 , -1)
#drone1.arm()
while True:
    if drone1.status == 'c':
        timeBegin = time.time()
        while True:
            if drone1.status != 'c':
                break
            if drone1.AprilTag['num'] > 0:
                drone1.status = 'l'
            timeNow = time.time() 
            timeFromBegin = timeNow - timeBegin

            drone1.set_target_position_under_optitrack_UDP(0.5 * math.cos(math.pi * timeFromBegin / 5), 0.5 * math.sin(math.pi * timeFromBegin / 5), -1.5)
            #drone1.set_target_position_under_optitrack_UDP(1 , 0.5 * math.cos(math.pi * timeFromBegin / 10), -1.5 + 0.5 * math.sin(math.pi * timeFromBegin / 10))
            print('circle')
    # if status == 'square':
        # timeBegin = time.time()
        # while True:
            # if(status != 'square'):
                # break
            # print('square')
    if drone1.status == 'L':
        drone1.from_point_to_point_under_optitrack_UDP(0.5, 0.5, -1, 0.5, -1, -1, 0.5)
        drone1.from_point_to_point_under_optitrack_UDP(0.5, -1, -1, 0.5, 0.5, -1, 0.5)
        if drone1.status != 'l':
            drone1.status = 'p'

    if drone1.status == 's':
        drone1.status = 'L'
        drone1.from_point_to_point_under_optitrack_UDP(0.5, 0.5, -1, 0.5, -1, -1, 0.5)
        drone1.from_point_to_point_under_optitrack_UDP(0.5, -1, -1, 0.5, -1, -2, 0.5)
        drone1.from_point_to_point_under_optitrack_UDP(0.5, -1, -2, 0.5, 0.5, -2, 0.5)
        drone1.from_point_to_point_under_optitrack_UDP(0.5, 0.5, -2, 0.5, 0.5, -1, 0.5)
        if drone1.status != 'l':
            drone1.status = 'p'



    if drone1.status == 'd':
        if drone1.AprilTag['num'] > 0:
            drone1.status = 'l'
        else:
            drone1.status = 'p'
    if drone1.status == 'p':
        timeBegin = time.time()
        while True:
            if drone1.status != 'p':
                break
            drone1.set_target_position_under_optitrack_UDP(1 , 0.5 , -1.5)
            #drone1.set_target_position_under_optitrack_UDP(0.5 , 0 , -1.5)
            print('point')
    if drone1.status == 'l':
        timeBegin = time.time()
        landing_position_now = drone1.position
        while True:
            timeNow = time.time() 
            timeFromBegin = timeNow - timeBegin
            altitude_now = drone1.position[2]
            drone1.set_target_position_under_optitrack_UDP(landing_position_now[0] , landing_position_now[1] , altitude_now + 0.1 * timeFromBegin)
            if(drone1.position[2] > -0.01):
                break
        break
               

#drone1.set_mode_to()

drone1.disconnect_from_optitrack_UDP()

drone1.disarm()
