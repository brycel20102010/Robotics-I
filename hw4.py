import random as rand
import numpy as np
import time
import sys
sys.path.insert(0, "/home/me/Desktop/FANUC-Ethernet_IP_Drivers/src")          #change to correct path
from robot_controller import robot
import paho.mqtt.client as mqtt
import json

#drive_path = '10.2.1.16' # Beaker
drive_path = '10.2.1.6' # Bunsen

beaker_busy = bool(0)
transformation_matrix = [
[1, 0, 0, 70.15],
[0, 1, 0, -2236.51],
[0, 0, 1, -39.04],
[0, 0, 0, 1]]
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  #shorthand for other calls


#connects robots together
def on_connect(client, userdata, flags, reason_code, properties):
    print("Connected")
    client.subscribe("beaker/status")

#reaction for when a message is published by beaker, bunsen only cares about whether beaker is busy or not
def on_message(client, userdata, msg):
    print(msg.payload)
    data = json.loads(msg.payload)
    global beaker_busy 
    beaker_busy = data["busy"]

#???
def transform(matrix, point):
    point_aug = np.append(point, 1)
    point_aug_trans = np.matmul(matrix, point_aug)
    return point_aug_trans[0:3]

#????
def random_point(point, offset):
    random_vector = [rand.uniform(-1,1), rand.uniform(-1,1), rand.uniform(-1,1)]
    return np.add(point, np.multiply(random_vector, offset))

#simple function to position beaker
def move_beak(coords, detail):    #move_beak([1, 2, 3, 4, 5, 6], "hi")
    global beaker_busy
    beaker_busy = 1
    print(detail)
    client.publish("beaker/command/position", '{"position": ' + str(coords) + '}')
    while(beaker_busy == 1):
        time.sleep(0.1)

#simple function to manipulate beaker's grabber
def open_beak(state):   #open_beak(1)
    global beaker_busy
    beaker_busy = 1
    if state:
        print("\nOpening Beaker\n")
    else:
        print("\nClosing Beaker\n")
    client.publish("beaker/command/gripper", '{"open": ' + str(state) + '}')

'''
#looping the handshaking action, has logic for whether we want to wave or not
def random_grab_loop(waving):    #random_grab_loop(0) would do this without waving
    yoff = [0, ###, 0, 0, 0, 0] #mm offset for pre-approach


    bun_coords[] = ### #random coords for bun
    beak_coords[] = ### #translate these coords for beak

    move_beak(beak_coords, "Beaker Moves to Random")

    crx10.write_cartesian_position(bun_coords + yoff)    #bun matches beaks position (this will be +y something for approach)
    crx10.write_cartesian_position(bun_coords)  #move to actually meet

    open_beak(0)   #grabs dice

    crx10.onRobot_gripper(120,40)      #bun lets go
    time.sleep(0.5)
    crx10.write_cartesian_position(bun_coords + yoff)    #bun moves back)
    if waving:
        ### #this is bunsens waving meneuver
    
    bun_coords[] = ### #random coords for bun
    beak_coords[] = ### #translate these coords for beak

    crx10.write_cartesian_position(bun_coords)  #bun moves to wait

    move_beak(beak_coords - yoff, "Beaker Moves to Match Bunsen")
    move_beak(beak_coords, "Beaker Moves to Bunsen")

    crx10.onRobot_gripper(80,40)    #bun grabs

    open_beak(1)   #beaker lets go
    move_beak(beak_coords, - yoff, "Beaker Backs Off")
    if waving:
        ### #this is beakers waving meneuver
'''



def main():
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("10.2.1.141", 1883)   #ip of broker and the port the broker is on
    client.loop_start()
    crx10 = robot(drive_path)   # create new robot object
    crx10.set_speed(200)    #set robot speed
    global beaker_busy
    


    #1 bunsen wiggle manueaver
    crx10.write_cartesian_position([449, -14, 85, 179, 3, -2])      #bunson home
    crx10.write_cartesian_position([636, 22, 74, 176, 0, 0])

    #2 tell beaker to do stuff
    #beaker_busy = 1
    #client.publish("beaker/command/position", '{"position": [460, -29, 115, -177, 5, 28]}')
    #print("\nfirst wiggle\n")
    #while(beaker_busy == 1):
    #    time.sleep(0.1)
    #open_beak(1)
    #move_beak([460,-29, 115, -177, 5, 28], "wiggling")
    open_beak(0)

    '''
    for i in range(3):          # repeats grab back-and-forth 3 times
        random_grab_loop(0)
    '''




'''
print(crx10.read_current_cartesian_pose())

crx10.write_cartesian_position([, , , , , ])

crx10.onRobot_gripper(120,40)      #open for bun
time.sleep(0.5)
crx10.onRobot_gripper(80,40)       #close for bun
time.sleep(0.5)

crx10.schunk_gripper('open')    #beak
time.sleep(0.5)
crx10.schunk_gripper('close')    #beak
time.sleep(0.5)

crx10.conveyor("reverse")
try:
    while not crx10.conveyor_proximity_sensor("right"):
        time.sleep(0.1)
except KeyboardInterrupt:
    crx10.conveyor("stop")
    crx10.onRobot_gripper(120,40)
finally:
    crx10.conveyor("stop")

client.publish("robots/arm2/command", "MOVE_TO_POSITION")


crx10.write_cartesian_position([460, -29, 115, -177, 5, 28])    #beaker home
crx10.write_cartesian_position([449, -14, 85, 179, 3, -2])      #bunson home
'''


if __name__=="__main__":        #this makes sure it doesn't run immidietely
    main()