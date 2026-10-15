#Bryce Lindsey 14 October 2026
import numpy as np
import random as rand
#MQTT imports
import paho.mqtt.client as mqtt
import json
#Robot imports
import time
import sys
sys.path.append('fanuc_ethernet_ip_drivers/src')
from robot_controller import robot 

#Robot vars
robot_ip = '10.2.1.6' #beaker
sleep_delay = 0.2
slave_busy = 0
transformation_matrix = [
[1, 0, 0, -15.15],
[0, 1, 0, 2196.51],
[0, 0, 1, 25.04],
[0, 0, 0, 1]]

#Mqtt vars
master_ip = '10.2.1.140'
master_port = 1885
pos_t = "beaker/command/position"
grip_t = "beaker/command/gripper"
stat_t ="beaker/status"

#Master robot movement
def move_cartesian(crx10, position: list[float], log: bool = 0):
    crx10.write_cartesian_position(position)
    if (log):
        print(f"Arm moving to:\nx={position[0]}, y={position[1]}, z={position[2]},\nw={position[3]}, p={position[4]}, r={position[5]}")
    time.sleep(sleep_delay) #prevent blocking
    return 

def gripper(crx10, width: float, force: float, log: bool = 0):
    crx10.onRobot_gripper(width, force)
    if (log):
        print(f"Gripper moving to {width}mm with force of {force}N")
    time.sleep(sleep_delay) #prevent blocking
    return

#Slave robot movement
def move_slave(client, position):
    client.publish(pos_t, '{\n"timestamp": '+str(time.time())+',\n"position": '+str(position)+'\n}')
    global slave_busy
    slave_busy = 1
    while (slave_busy):
        client.loop_start()

def grip_slave(client, grip):
    client.publish(grip_t, '{\n"timestamp": '+str(time.time())+',\n"open": '+str(grip)+'\n}')
    global slave_busy
    slave_busy = 1
    while (slave_busy):
        client.loop_start()

#Point vector calculations
def transform(matrix, point: float[3]):
    point_aug = np.append(point, 1)
    point_aug_trans = np.matmul(matrix, point_aug)
    return point_aug_trans[0:3].tolist()

def random_point(point: float[3], offest: float[3]):
    random_vector = [rand.uniform(-1,1), rand.uniform(-1,1), rand.uniform(-1,1)]
    return np.add(point, np.multiply(random_vector, offest)).tolist()

#MQTT functions
def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(stat_t)
    print("Connected to: " + master_ip)

def on_message(client, userdata, msg):
    print(msg.payload)
    if msg.topic == stat_t:
        data = json.loads(msg.payload)
        status = data["busy"]
        global slave_busy
        slave_busy = int(status)
    else: 
        print("Off topic message recived.")

#Movement subroutines
def pickup_block(crx10, position: list[float], vertical_offset: float, let_go: int):
    p_up = [position[0], position[1], position[2]+vertical_offset, position[3], position[4], position[5]]
    p_down = position
    move_cartesian(crx10, p_up)
    if(not let_go):
        gripper(crx10, 120, 80)
    move_cartesian(crx10, p_down)
    if (let_go):
        gripper(crx10, 120, 80)
    else:
        gripper(crx10, 80, 80)
    move_cartesian(crx10, p_up)

def wave_bye(client, position: list[float], position2: list[float], times: int):
    i = 0
    while (i < times):
        move_slave(client, position)
        move_slave(client, position2)
        i = i+1    
    move_slave(client, position)

def random_move(crx10, client, p_center, p_range, offset_y, p_rot_master, p_rot_slave, held_by_master):
    p_random = random_point(p_center, p_range)
    p_random_offset = [p_random[0], p_random[1]+offset_y, p_random[2]]
    p_random_s = transform(transformation_matrix, p_random)
    p_random_offest_s = [p_random_s[0], p_random_s[1]-offset_y, p_random_s[2]]
    move_cartesian(crx10, p_random_offset + p_rot_master)
    move_slave(client, p_random_offest_s + p_rot_slave)
    move_cartesian(crx10, p_random + p_rot_master)
    move_slave(client, p_random_s + p_rot_slave)
    if(held_by_master):
        grip_slave(client, 0)
        gripper(crx10, 120, 80)
    else:
        gripper(crx10, 80, 80)
        grip_slave(client, 1)
    move_cartesian(crx10, p_random_offset + p_rot_master)
    move_slave(client, p_random_offest_s + p_rot_slave)
    
def control_loop(crx10, client):
    p_block = [377.1, 286.0, -110.0, 179.9, 0, 0]
    offset_y = 150
    p_center = [270.5, -1006.0, 300.8]
    p_rot_master = [119.5, 90, 32.3]
    p_rot_slave = [90, -60, 179.9]
    p_range = [100,50,100]
    pickup_block(crx10, p_block, 100, 0)
    for i in range(3):
        random_move(crx10, client, p_center, p_range, offset_y, p_rot_master, p_rot_slave, 1)
        random_move(crx10, client, p_center, p_range, offset_y, p_rot_master, p_rot_slave, 0)
    wave_bye(client, [270.5, 1006.0-150, 300.8, 90, -60, 179.9], [270.5, 1006.0-150, 300.8, 90, -30, 179.9], 1)
    move_cartesian(crx10, [385.1, 285.0, -110.0+100, 179.9, 0, 0])
    move_cartesian(crx10, [385.1, 285.0, -110.0+100, 179.9, 0, 90])
    pickup_block(crx10, [385.1, 285.0, -110.0, 179.9, 0, 90], 100, 1)
    crx10.write_joint_pose([0,0,0,0,-90,0])
    move_slave(client, [540, -150, 305, 179.9, 0, 0])    

time.sleep(5)
crx10 = robot(robot_ip)
crx10.set_speed(300) #Max 300
crx10.write_joint_pose([0,0,0,0,-90,0])
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(master_ip, master_port)
client.on_connect = on_connect
client.on_message = on_message
control_loop(crx10, client)
client.loop_forever()