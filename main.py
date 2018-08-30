import sys
sys.path.insert(0, "lib/x64") # initialize the library

import Leap, sys, thread, time , math , operator , dronekit
from Leap import  SwipeGesture
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

##-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')
# -- Setup the commanded flying speed
gnd_speed = 5  # [m/s]

def analyzeDirection(ypr):
    if(ypr[1] > 0 and (math.fabs(ypr[1]) > (math.fabs(ypr[0]) and math.fabs(ypr[2])))):
        print("pitch up")
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif(ypr[1] < 0 and (math.fabs(ypr[1]) > (math.fabs(ypr[0]) and math.fabs(ypr[2])))):
        print("pitch down")
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    elif(ypr[2] > 0 and (math.fabs(ypr[2]) > (math.fabs(ypr[1]) and math.fabs(ypr[0])))):
        print("roll left")
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif(ypr[2] < 0 and (math.fabs(ypr[2]) > (math.fabs(ypr[1]) and math.fabs(ypr[0])))):
        print("roll right")
        set_velocity_body(vehicle, 0, gnd_speed, 0)
    if (ypr[0] > 0 and (math.fabs(ypr[0]) > (math.fabs(ypr[1]) and math.fabs(ypr[2])))):
        print("Yaw Right")
    elif (ypr[0] < 0 and (math.fabs(ypr[0]) > (math.fabs(ypr[1]) and math.fabs(ypr[2])))):
        print("Yaw Left")


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


        for hand in frame.hands:
            direction = hand.direction
            # print(direction)

            normal = hand.palm_normal
            pitch = direction.pitch * Leap.RAD_TO_DEG
            roll = normal.roll * Leap.RAD_TO_DEG
            yaw = direction.yaw * Leap.RAD_TO_DEG * 0.9

            ypr = (yaw , pitch , roll)
            analyzeDirection(ypr)
            print ("Pitch : " + str(pitch) + " - Roll : " + str(roll) + " - Yaw : " + str(yaw) )

        for gesture in frame.gestures():
            #print(gesture)

            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                swipeDir = swipe.direction

                if(swipeDir.y > 0 and math.fabs(swipeDir.x) < math.fabs(swipeDir.y)):
                    print("Take off")
                    arm_and_takeoff(10)
                elif(swipeDir.y < 0 and math.fabs(swipeDir.x) < math.fabs(swipeDir.y)):
                    print("Landing")
                    vehicle.mode = VehicleMode("RTL")


# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

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

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)


    print("Ready for take off, swipe up your hand")
    #arm_and_takeoff(10) # take off

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
