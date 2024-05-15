## R2 WORKSPACE
This project contains several directories, each serving a specific purpose in the overall system. Here's a brief overview of each directory:


### ball_tracking
This directory contains code for tracking a ball in a video stream. It may use image processing techniques to identify and track the position of the ball over time.

### line_following
This directory contains code for a line-following robot. The robot uses sensors to detect a line on the ground and follows it.

### luna_controller
This directory contains the controller code for the Luna robot. It may include code for handling user input, controlling the robot's movements, and managing the robot's state.

### Microros_pub_sub
This directory contains code for a micro-ROS publisher and subscriber. micro-ROS is a version of ROS (Robot Operating System) designed for microcontrollers. The publisher sends messages on a topic, and the subscriber receives those messages.

### r2_bringup
This directory contains launch files for starting up the R2 robot. Launch files in ROS are used to start one or more nodes, such as a driver node for a sensor and a processing node for sensor data.

### r2_description
This directory contains the URDF (Unified Robot Description Format) files for the R2 robot. URDF files describe the robot's physical configuration, including its size, shape, and joint movements.

### r2_interfaces
This directory contains the definition of custom message and service types used by the R2 robot. In ROS, nodes communicate with each other by sending messages. Custom message types allow for complex data structures to be sent between nodes.

### r2_py
This directory contains Python code for the R2 robot. It may include scripts for controlling the robot, processing sensor data, and implementing robot behaviors.

### silo_tracking
This directory contains code for tracking silos. It may use image processing techniques to identify and track the position of silos in a video stream.

Each directory contains its own set of files and further subdirectories, which are organized according to their purpose in the system. For more information about a specific directory, please refer to the README file in that directory (if available).
