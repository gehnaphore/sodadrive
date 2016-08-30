# Robot PenguIn

## How-To-Run

* Power-Up AND Log-In
 
* Start wheel_controller_node and drive_controller_node
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rpi drive_controller.launch
```

* Login another terminal

* Start teleop_keyboard_node 
```
cd ~/catkin_ws
source devel/setup.bash
rosrun rpi teleop_keyboard_node
```

## Nodes

### wheel_controller_node

This node is responsible for controlling the individual speed of the two wheels of the differential robot drive system.

#### Subscribed Topics
* **`/rpi/speed_1`** ([std_msgs/Float64])

Set speed of the left motor in m/s.

* **`/rpi/speed_2`** ([std_msgs/Float64])

Set speed of the right motor in m/s.

#### Published Topics
* **`/rpi/actual_speed_1`** ([std_msgs/Float64])

Actual speed of the left motor in m/s.

* **`/rpi/actual_speed_2`** ([std_msgs/Float64])

Actual speed of the right motor in m/s.

#### Services
*None*

#### Parameters
*None*

### drive_controller_node

This node provides target wheel speeds for the wheel_controller_node based on the desired linear and angular speed.

#### Subscribed Topics
* **`/rpi/cmd_vel`** ([geometry_msgs/Twist])

Linear (m/s) and angular (DEG/s) speed of the robot.

* **`/rpi/actual_speed_1`** ([std_msgs/Float64])

Actual speed of the left motor in m/s.

* **`/rpi/actual_speed_2`** ([std_msgs/Float64])

Actual speed of the right motor in m/s.

#### Published Topics
* **`/rpi/speed_1`** ([std_msgs/Float64])

Set speed of the left motor in m/s.

* **`/rpi/speed_2`** ([std_msgs/Float64])

Set speed of the right motor in m/s.

#### Services
*None*

#### Parameters
*None*

### teleop_keyboard

This node allows to control the robot via keyboard by sending [geometry_msgs/Twist] Messages to the **`/rpi/cmd_vel`** topic upon the reception of certain keyboard inputs.

#### Subscribed Topics
*None*

#### Published Topics
* **`/rpi/cmd_vel`** ([geometry_msgs/Twist])

Linear (m/s) and angular (DEG/s) speed of the robot.

#### Services
*None*

#### Parameters
*None*

### wheel_controller_test

This node allows for sending the desired target speed for the left and the right motor to the wheel_controller_node. **This node is for testing only**.

#### Subscribed Topics
*None*

#### Published Topics
* **`/rpi/speed_1`** ([std_msgs/Float64])

Set speed of the left motor in m/s.

* **`/rpi/speed_2`** ([std_msgs/Float64])

Set speed of the right motor in m/s.

#### Services
*None*

#### Parameters
*None*

## Configuration

### Login

**User:** pi
**Pass:** raspberry

### Setup WiFi/WLAN

* Scan for available WiFi networks
```
sudo iwlist wlan0 scan
```

* Edit /etc/wpa\_supplicant/wpa\_supplicant.conf
```
sudo vim /etc/wpa_supplicant/wpa_supplicant.conf
```

* Go to the bottom of the file and add the following:
```
network={
    ssid="The_ESSID_from_earlier"
    psk="Your_wifi_password"
}
```

* Restart the WiFi adapter
```
sudo ifdown wlan0
sudo ifup wlan0
```

* Verify that a connection was erstablished
```
ifconfig wlan0
```

[std_msgs/Float64]: http://docs.ros.org/indigo/api/std_msgs/html/msg/Float64.html
[geometry_msgs/Twist]: http://docs.ros.org/indigo/api/geometry_msgs/html/msg/Twist.html
