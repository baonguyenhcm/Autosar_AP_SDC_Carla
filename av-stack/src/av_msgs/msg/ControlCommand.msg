# Output of the control module to the vehicle interface.
std_msgs/Header header
float64 throttle   # [0..1]
float64 brake      # [0..1]
float64 steer      # [rad] front-wheel angle, +left
