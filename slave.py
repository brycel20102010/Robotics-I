#Bryce Lindsey 14 Oct 2026
#MQTT imports
import paho.mqtt.client as mqtt
import json

#Robot imports
import time
import sys
sys.path.append('fanuc_ethernet_ip_drivers/src')
from robot_controller import robot 

#Robot vars
robot_ip = '10.2.1.16' #beaker
sleep_delay = 0.2

#Mqtt vars
master_ip = '10.2.1.140'
master_port = 1885
pos_t = "beaker/command/position"
grip_t = "beaker/command/gripper"
stat_t ="beaker/status"

def move_cartesian(crx10, position: list[float], log: bool = 0):
    client.publish(stat_t, '{\n"time": '+str(time.time())+',\n"busy": 1\n}') #Beaker is now busy
    crx10.write_cartesian_position(position)
    if (log):
        print(f"Arm moving to:\nx={position[0]}, y={position[1]}, z={position[2]},\nw={position[3]}, p={position[4]}, r={position[5]}")
    time.sleep(sleep_delay) #prevent blocking
    client.publish(stat_t, '{\n"time": '+str(time.time())+',\n"busy": 0\n}') #Beaker is not busy

def gripper(crx10, open: bool, log: bool = 0):
    client.publish(stat_t, '{\n"time": '+str(time.time())+',\n"busy": 1\n}') #Beaker is now busy
    if (int(open) == 1):
        crx10.schunk_gripper("open")
    elif (int(open) == 0):
        crx10.schunk_gripper("close")
    else:
        print("Schunk not closed")
    time.sleep(sleep_delay) #prevent blocking
    client.publish(stat_t, '{\n"timestamp": '+str(time.time())+',\n"busy": 0\n}')

def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(pos_t)
    client.subscribe(grip_t)
    print("Connected to: " + master_ip)

def on_message(client, userdata, msg):
    print(msg.payload)
    if msg.topic == pos_t:
        data = json.loads(msg.payload)
        position = data["position"]
        move_cartesian(crx10, position)
    elif msg.topic == grip_t:
        data = json.loads(msg.payload)
        grip_state = data["open"]
        gripper(crx10, grip_state)
    else: 
        print("Off topic message recived.")

crx10 = robot(robot_ip)
crx10.set_speed(300) #Max 300
crx10.write_joint_pose([0,0,0,0,-90,0])
crx10.schunk_gripper("open")
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(master_ip, master_port)
client.on_connect = on_connect
client.on_message = on_message
client.loop_forever()