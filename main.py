"""
Bismillahirahmanirahim
this program use LeapMotion to control drone and Flex Sensor for giving control in thrust
Hardware :
PX4
Leap Motion
WeMos
Flex Sensor
To Do :
- SITL (done)
- take off (done)
- Move on NED Frame (done)
- receive flex degree from MQTT (done)
- process degree to ground speed (done)
- Yaw (done)
- Web Server using cherrypy( not yet)
"""

import sys
sys.path.insert(0, "lib/x64") # initialize the library

import Leap, sys, thread, time , math , operator , dronekit
from Leap import  SwipeGesture
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import os , os.path

# mqtt part
import paho.mqtt.client as mqtt
# define IP and Port MQTT Broker
brokerHost = "192.168.1.2"
port = 1883

# webserver part
import cherrypy
from ws4py.server.cherrypyserver import WebSocketPlugin, WebSocketTool
from ws4py.websocket import EchoWebSocket
from ws4py.client.threadedclient import WebSocketClient

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))
    # subscribe the topic "ekg/device1/signal"
    # subTopic = "ekg/+/signal"  # + is wildcard for all string to that level
    subTopic = "flex/degree"
    print "Subscribe topic ", subTopic
    clientMQTT.subscribe(subTopic)

def on_message(client, userdata, message):
    global gnd_speed
    print "message topic=", message.topic , " - qos=", message.qos , " - flag=", message.retain
    receivedMessage = str(message.payload.decode("utf-8"))
    print "received message = " , receivedMessage

    degree = float(receivedMessage)
    gnd_speed = setGroundSpeed(degree) # convert degree to scaled ground speed
    print(gnd_speed)

# create client object
clientMQTT = mqtt.Client("client-Server")


## MAVLink Part
##-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551') # sitl mode
#vehicle = connect("COM23",baud=57600)
# -- Setup the commanded flying speed
gnd_speed = 0.5  # [m/s]


# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print "waited motor armed."
        vehicle.armed = True
        time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


# -- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def condition_yaw(heading,  direction, relative=False ):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle

    if direction == 'right':
        # to move clock wise
        dir_move = 1
    else:
        # to move counter clock wise
        dir_move = 0

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        dir_move,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


# Leap Motion Part
class SampleListener(Leap.Listener):
    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Motion Sensor Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # check just one hand
        if (len(frame.hands) == 1):
            for hand in frame.hands:
                direction = hand.direction
                # print(direction)

                normal = hand.palm_normal
                pitch = direction.pitch * Leap.RAD_TO_DEG
                roll = normal.roll * Leap.RAD_TO_DEG
                yaw = direction.yaw * Leap.RAD_TO_DEG * 0.9

                ypr = (yaw , pitch , roll)
                analyzeDirection(ypr)
                #print ("Pitch : " + str(pitch) + " - Roll : " + str(roll) + " - Yaw : " + str(yaw) )

            for gesture in frame.gestures():
                #print(gesture)

                if gesture.type == Leap.Gesture.TYPE_SWIPE:
                    swipe = SwipeGesture(gesture)
                    swipeDir = swipe.direction

                    if(swipeDir.y > 0 and math.fabs(swipeDir.x) < math.fabs(swipeDir.y)):
                        print("Take off")
                        arm_and_takeoff(1)
                    elif(swipeDir.y < 0 and math.fabs(swipeDir.x) < math.fabs(swipeDir.y)):
                        print("Landing")
                        vehicle.mode = VehicleMode("RTL")


# analyze Dircetion
def analyzeDirection(ypr):
    if(ypr[1] > 0 and (math.fabs(ypr[1]) > (math.fabs(ypr[0]) and math.fabs(ypr[2])))):
        # ke depan
        print("pitch up")
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    elif(ypr[1] < 0 and (math.fabs(ypr[1]) > (math.fabs(ypr[0]) and math.fabs(ypr[2])))):
        # ke belakang
        print("pitch down")
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif(ypr[2] > 0 and (math.fabs(ypr[2]) > (math.fabs(ypr[1]) and math.fabs(ypr[0])))):
        # ke kiri
        print("roll left")
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif (ypr[0] > 0 and (math.fabs(ypr[0]) > (math.fabs(ypr[1]) and math.fabs(ypr[2])))):
        print("Yaw Right")
        condition_yaw(gnd_speed*0.5,'right',True)
    elif (ypr[0] < 0 and (math.fabs(ypr[0]) > (math.fabs(ypr[1]) and math.fabs(ypr[2])))):
        print("Yaw Left")
        condition_yaw(gnd_speed*0.5,'left',True)
    elif (ypr[2] < 0 and (math.fabs(ypr[2]) > (math.fabs(ypr[1]) and math.fabs(ypr[0])))):
        # ke kanan
        print("roll right")
        set_velocity_body(vehicle, 0, gnd_speed, 0)

# convert degree to ground speed
def setGroundSpeed(degree):
    """
    :param degree:
    :return: ground speed
    x:input value;
    a,b:input range (0 - 90)
    c,d:output range (0 - 5)
    y:return value ( between 0 - 5)
    """
    a = 0
    b = 90
    c = 0
    d = 5
    y = (degree - a) / (b - a) * (d - c) + c

    return y

class DummyClient(WebSocketClient):
    def opened(self):
        print "Connect ws ..."

    def closed(self, code, reason=None):
        print "Closed down", code, reason

    def received_message(self, m):
        print('Received Message : ')
        print m
        #if len(m) == 175:
        #    self.close(reason='Bye bye')


def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # set callback
    clientMQTT.on_message = on_message
    clientMQTT.on_connect = on_connect
    # connection established
    print "connecting to broker", brokerHost
    clientMQTT.connect(brokerHost, port)  # connect to broker

    # webserver
    cherrypy.config.update({'server.socket_port': 8080}) # socket in web 9000
    #WebSocketPlugin(cherrypy.engine).subscribe()
    #cherrypy.tools.websocket = WebSocketTool()
    cherrypy.quickstart(webserver(), '/' , setup_cherry())  # start the webserver

    print("Ready for take off, swipe up your hand")
    #arm_and_takeoff(10) # take off
    clientMQTT.loop_forever()  # loop forever

    # connect to websocket
    """
    ws = DummyClient('ws://127.0.0.1:6437/v6.json', protocols=['http-only'])
    ws.connect()
    ws.loop_forever()
    """

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        ws.close()
        controller.remove_listener(listener)

def setup_cherry():
    conf = {
        '/': {
            'tools.sessions.on': True,
            'tools.staticdir.root': os.path.abspath(os.getcwd())
        },
        '/static' : {
            'tools.staticdir.on': True,
            'tools.staticdir.dir': './www'
        }
    }
    return conf

# webserver

class webserver(object):
    @cherrypy.expose()
    def index(self):
        return open('www/index.html')

    @cherrypy.expose()
    def home(self):
        return open('www/home.html')



if __name__ == "__main__":
    main() # start the leap and mqtt