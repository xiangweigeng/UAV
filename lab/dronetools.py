# -*- coding:utf-8 -*-
from pymavlink import mavutil
import time
import socket
import math
import thread

class drone:
    def __init__(self, port, baudrate, system_id):
        self.port = port
        self.baudrate = baudrate
        self.system_id = system_id
        self.position = [0,0,0,0,0,0]
        self.status = 'p'
        self.threadLock_keyboard = thread.allocate_lock()
        self.threadLock_position_write = thread.allocate_lock()
        self.AprilTag = {'num' : 0} 
        while True:
            self.the_connection = mavutil.mavlink_connection(port, baudrate)
            self.the_connection.wait_heartbeat()
            if self.the_connection.target_system == system_id:
                print("connection establised! system_id: %u component_id %u") % (self.the_connection.target_system, self.the_connection.target_component)
                break
        
    def get_status_from_keyboard(self):
        #global self.status
        while True:
            key_board_input = raw_input()
            if key_board_input == 'c' or key_board_input == 'p' or key_board_input == 'l' or key_board_input == 's' or key_board_input == 'L':
                if self.status != 'p' and key_board_input != 'p' and key_board_input != 'l':
                    print('You have change to point status first!!')
                    continue
                else:
                    self.threadLock_keyboard.acquire()
                    self.status = key_board_input
                    self.threadLock_keyboard.release()
            else:
                print('input error!!')

    def get_command_from_keyboard(self):
        thread.start_new_thread(self.get_status_from_keyboard,() )
    
    
    def connect_to_optitrack_UDP(self, ip, port):
        #setup UDP
        self.optitrack_UDP_address = (ip, port)
        #address = ('172.18.91.167', 31500)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(self.optitrack_UDP_address)

    def disconnect_from_optitrack_UDP(self):
        self.s.close()

    def get_position_from_optitrack_UDP(self):
        self.UDP_data_from_optitrack, self.UDP_data_from_addr = self.s.recvfrom(2048)
        if not self.UDP_data_from_optitrack:
            print "no data!"
        position_temp = self.UDP_data_from_optitrack.split(',')
        for i in xrange(0,6):
            position_temp[i] = float(position_temp[i])
        self.threadLock_position_write.acquire()
        self.position = position_temp
        self.threadLock_position_write.release()
 
        print "received:", self.UDP_data_from_optitrack, "from", self.UDP_data_from_addr

    #def get_position_from_optitrack_UDP(self):

    #    self.UDP_data_from_optitrack, self.UDP_data_from_addr = self.s.recvfrom(2048)
    #    if not self.UDP_data_from_optitrack:
    #        print "no data!"
    #    self.position = self.UDP_data_from_optitrack.split(',')
    #    for i in xrange(0,6):
    #        self.position[i] = float(self.position[i])
 
    #    print "received:", self.UDP_data_from_optitrack, "from", self.UDP_data_from_addr

    def send_position_data_to_PX4(self):

        self.the_connection.mav.vision_position_estimate_send(self.system_id,   self.position[0] ,   self.position[1] ,   self.position[2] ,   self.position[3] ,   self.position[4] ,   self.position[5]  )
        print ("send optitrack position : ", self.position)

        
    
    def arm(self):
        self.the_connection.mav.command_long_send(
        self.the_connection.target_system, #1# autopilot system id
        self.the_connection.target_component, #1# autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        1, # arm!
        0,
        0.0,0.0,0.0,0.0,0.0, # unused parameters for this command,
        force_mavlink1=True)
        time.sleep(0.5)
        print("armed!!")

    def disarm(self):
        self.the_connection.mav.command_long_send(
        self.the_connection.target_system, #1# autopilot system id
        self.the_connection.target_component, #1# autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        0, # arm!
        0,
        0.0,0.0,0.0,0.0,0.0, # unused parameters for this command,
        force_mavlink1=True)
        print("disarmed!!")

    def set_target_position_under_optitrack_UDP(self, x, y, z):
        self.the_connection.mav.set_position_target_local_ned_send(0,self.the_connection.target_system,self.the_connection.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b110111111000,x,y,z,0,0,0,0,0,0,0,0,force_mavlink1=True)

    def set_target_position_with_velocity_under_optitrack_UDP(self, x, y, z, v_x, v_y, v_z):
        self.the_connection.mav.set_position_target_local_ned_send(0,self.the_connection.target_system,self.the_connection.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b110111000000,x,y,z,v_x,v_y,v_z,0,0,0,0,0,force_mavlink1=True)



#    def set_mode_to(self):
#        self.the_connection.mav.command_long_send(
#        self.the_connection.target_system, #1# autopilot system id
#        self.the_connection.target_component, #1# autopilot component id
#        176, #MAV_CMD_DO_SET_MODE (176 ) 
#        208,
#        0, 
#        0,
#        0.0,0.0,0.0,0.0,0.0, # unused parameters for this command,
#        force_mavlink1=True)

    def set_mode_to_offboard(self):

        self.the_connection.mav.command_long_send(

            self.the_connection.target_system,  # 1# autopilot system id

            self.the_connection.target_component,  # 1# autopilot component id

            176,  # MAV_CMD_DO_SET_MODE (176 )

            0,  # confirmation

            129,

            6,

            0,

            0.0, 0.0, 0.0, 0.0,  # unused parameters for this command,

            force_mavlink1=True)




    def from_point_to_point_under_optitrack_UDP(self ,start_x, start_y, start_z, target_x, target_y, target_z, velocity): # start point, target_point, trajectory_velocity(m/s)
        difference = [target_x - start_x, target_y - start_y, target_z - start_z]
        if difference[0] == 0 and difference[1] == 0 and difference[2] == 0:
            print("There is no distance!!")
            self.status = 'p'
            return 0
        while True:
            if self.status != 'L':
                break
            if ((self.position[0] - start_x) ** 2 +  (self.position[1] - start_y) ** 2 + (self.position[2] - start_z) ** 2) < 0.0049:
                print("drone is on the start point ", [start_x, start_y, start_z])
                break
            print("Going to start point", [start_x, start_y, start_z])
            self.set_target_position_under_optitrack_UDP(start_x, start_y, start_z)

        time_point_to_point_begin = time.time() 
        velocity_xy = float(math.sqrt(difference[0] ** 2 + difference[1] ** 2)) / math.sqrt(difference[0] ** 2 + difference[1] ** 2 + difference[2] ** 2) * velocity
        velocity_z =  float(difference[2]) / math.sqrt(difference[0] ** 2 + difference[1] ** 2 + difference[2] ** 2) * velocity
        if difference[0] == 0:
            velocity_x = 0
        else:
            velocity_x  = float(difference[0]) / math.sqrt(difference[0] ** 2 + difference[1] ** 2) * velocity
        if difference[1] == 0:
            velocity_y = 0
        else:
            velocity_y  = float(difference[1]) / math.sqrt(difference[0] ** 2 + difference[1] ** 2) * velocity
        while True:
            if self.status != 'L':
                break
            time_point_to_point_now = time.time()
            if ((self.position[0] - target_x) ** 2 +  (self.position[1] - target_y) ** 2 + (self.position[2] - target_z) ** 2) < 0.0049:
                print("drone is on the terminal point %s", [target_x, target_y, target_z])
                break
            print("Going to target point", [target_x, target_y, target_z])
            time_point_to_point_difference = time_point_to_point_now - time_point_to_point_begin
            if difference[0] * (difference[0] - velocity_x * time_point_to_point_difference) < 0 or  difference[1] * (difference[1] - velocity_y * time_point_to_point_difference) < 0 or difference[2] * (difference[2] - velocity_z * time_point_to_point_difference) < 0 :
                self.set_target_position_under_optitrack_UDP(target_x, target_y, target_z)
            else:
                self.set_target_position_under_optitrack_UDP(start_x + velocity_x * time_point_to_point_difference, start_y + velocity_y * time_point_to_point_difference, start_z + velocity_z * time_point_to_point_difference)

    #def from_point_to_point_under_optitrack_UDP(self ,start_x, start_y, start_z, target_x, target_y, target_z):
    #   while True:
    #       if (self.position[0] - start_x) ** 2 +  (self.position[1] - start_y) ** 2 + (self.position[2] - start_z) ** 2 < 0.0025:
    #           print("drone is on the start point ", [start_x, start_y, start_z])
    #           break
    #       self.set_target_position_under_optitrack_UDP(start_x, start_y, start_z)

    #   while True:
    #       if (self.position[0] - start_x) ** 2 +  (self.position[1] - start_y) ** 2 + (self.position[2] - start_z) ** 2 < 0.0025:
    #           print("drone is on the terminal point %s", [target_x, target_y, target_z])
    #           break
    #       self.set_target_position_under_optitrack_UDP(target_x,target_y.target_z)
   
 
