# Run SITL
dronekit-sitl copter --home=-6.9767975,107.6302553,90,0


mavproxy --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551


mavproxy --master com23 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551


# Run Mosquitto
cd C:\Program Files (x86)\mosquitto


mosquitto -v -c mosquitto.conf

# Run LDS-01
roscore
roslaunch hls_lfcd_lds_driver hlds_laser.launch
roslaunch hector_slam_launch tutorial.launch