# DIY Ackermann Robot - ROS2 Humble Simulation Package

A hands-on ROS2 package for learning and building an Ackermann steering robot (car-like steering) in Gazebo with a **parametrized URDF** for educational purposes.

![Ackermann Robot](https://img.shields.io/badge/ROS2-Humble-blue) ![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange) ![License](https://img.shields.io/badge/license-MIT-green)

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Understanding the Parametrized URDF](#understanding-the-parametrized-urdf)
- [Usage](#usage)
  - [Visualize in RViz](#1-visualize-in-rviz)
  - [Run Simulation](#2-run-gazebo-simulation)
  - [Control the Robot](#3-control-the-robot)
- [Customization Guide](#customization-guide)
- [Topics and Services](#topics-and-services)
- [Troubleshooting](#troubleshooting)
- [Educational Resources](#educational-resources)
- [License](#license)

---

## 🎯 Overview

This package provides a complete simulation environment for an **Ackermann steering robot** - a four-wheeled vehicle with front-wheel steering, similar to a car. The robot features:

- **Parametrized URDF/Xacro**: All robot dimensions, masses, and properties are defined as parameters for easy modification
- **Realistic Ackermann steering**: Front wheels steer independently using proper kinematic constraints
- **Integrated sensors**: 2D LiDAR for environment perception
- **Gazebo integration**: Full physics simulation with Gazebo Fortress
- **ROS2 Humble**: Native ROS2 support with proper message types

Perfect for:
- Learning robotics fundamentals
- Teaching URDF/Xacro concepts
- Developing autonomous navigation algorithms
- Testing control strategies

---

## ✨ Features

### Robot Features
- ✅ **Ackermann steering mechanism** with configurable steering angle
- ✅ **Four wheels**: Two fixed rear wheels, two steerable front wheels
- ✅ **2D LiDAR sensor** for obstacle detection
- ✅ **Odometry publishing** for localization
- ✅ **Joint state publishing** for visualization

### Technical Features
- ✅ **Fully parametrized URDF**: Modify robot dimensions without touching complex XML
- ✅ **Xacro macros**: Reusable components for wheels and sensors
- ✅ **Gazebo plugins**: Ackermann drive, sensors, and joint states
- ✅ **RViz configuration**: Pre-configured visualization setup
- ✅ **Launch files**: Easy-to-use launch files with arguments
- ✅ **Well-documented code**: Comments explaining every section

---

## 📦 Prerequisites

### Required Software

1. **ROS2 Humble** (Desktop or Base install)
   ```bash
   # If not installed, follow: https://docs.ros.org/en/humble/Installation.html
   ```

2. **Gazebo Fortress** (not Gazebo Classic)
   ```bash
   sudo apt update
   sudo apt install gz-fortress
   ```

3. **ROS2-Gazebo Bridge**
   ```bash
   sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
   ```

4. **Additional ROS2 Packages**
   ```bash
   sudo apt install ros-humble-xacro \
                    ros-humble-joint-state-publisher \
                    ros-humble-joint-state-publisher-gui \
                    ros-humble-robot-state-publisher \
                    ros-humble-rviz2 \
                    ros-humble-teleop-twist-keyboard
   ```

### System Requirements
- Ubuntu 22.04 LTS
- 4GB RAM minimum (8GB recommended)
- Dedicated GPU recommended for Gazebo

---

## 🚀 Installation

1. **Create or navigate to your ROS2 workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone or copy this package:**
   ```bash
   # If you have the package directory:
   cp -r /path/to/diy_ackermann .
   
   # Or if in a git repository:
   # git clone <repository-url>
   ```

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select diy_ackermann --symlink-install
   ```
   
   > **Note**: `--symlink-install` allows you to modify Python files and launch files without rebuilding

4. **Source the workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
   
   > **Tip**: Add this to your `~/.bashrc` for automatic sourcing:
   > ```bash
   > echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   > ```

---

## 📁 Package Structure

```
diy_ackermann/
├── launch/
│   ├── rviz.launch.py          # Launch RViz visualization only
│   └── simulation.launch.py    # Launch full Gazebo simulation
├── urdf/
│   └── diy_ackermann.urdf.xacro  # Parametrized robot description
├── worlds/
│   └── ackermann_world.sdf     # Gazebo world with obstacles
├── rviz/
│   └── diy_ackermann.rviz    # RViz configuration
├── config/                      # Configuration files (future use)
├── meshes/                      # 3D meshes (if needed)
├── diy_ackermann/            # Python package (future nodes)
├── package.xml                  # Package manifest
├── setup.py                     # Python setup file
└── README.md                    # This file
```

---

## 🎓 Understanding the Parametrized URDF

One of the main educational features of this package is the **parametrized URDF**. Let's break down how it works:

### What is Xacro?

Xacro (XML Macros) extends URDF with:
- **Properties**: Variables you can define once and reuse
- **Macros**: Reusable templates (like functions)
- **Math expressions**: Calculate values dynamically

### Key Parameters in Our Robot

Open `urdf/diy_ackermann.urdf.xacro` and look at the top section:

```xml
<!-- Robot dimensions -->
<xacro:property name="chassis_length" value="1.0"/>
<xacro:property name="chassis_width" value="0.5"/>
<xacro:property name="chassis_height" value="0.2"/>
<xacro:property name="chassis_mass" value="20.0"/>

<!-- Wheel properties -->
<xacro:property name="wheel_radius" value="0.15"/>
<xacro:property name="wheel_width" value="0.08"/>
<xacro:property name="wheel_mass" value="1.5"/>

<!-- Axle dimensions -->
<xacro:property name="wheelbase" value="0.7"/>
<xacro:property name="track_width" value="0.5"/>

<!-- Steering properties -->
<xacro:property name="max_steering_angle" value="0.5236"/>  <!-- 30 degrees -->
```

**To modify your robot:**
1. Change any of these values
2. Rebuild: `colcon build --packages-select diy_ackermann`
3. Re-launch to see changes

### Reusable Macros

The URDF uses macros to avoid repetition. For example, the `wheel` macro:

```xml
<xacro:macro name="wheel" params="prefix suffix reflect_y *origin">
  <!-- Creates a complete wheel with visual, collision, and inertia -->
</xacro:macro>

<!-- Usage: -->
<xacro:wheel prefix="rear" suffix="left" reflect_y="1">
  <origin xyz="0 ${track_width/2} 0" rpy="0 0 0"/>
</xacro:wheel>
```

This creates consistent wheels without copying code!

### Ackermann Steering Explained

The robot has:
- **Rear axle**: Fixed, wheels only rotate
- **Front axle**: Steerable, each wheel can turn independently
- **Steering joints**: Revolute joints limited by `max_steering_angle`

```
     [Front Steering]
         /        \
   Left Wheel    Right Wheel
        |             |
    Steering      Steering
     Joint         Joint
        |             |
   =================== Front Axle
         Chassis
   =================== Rear Axle
        |             |
   Left Wheel    Right Wheel
```

---

## 🎮 Usage

### 1. Visualize in RViz

**Purpose**: View the robot model and manually control joints

```bash
ros2 launch diy_ackermann rviz.launch.py
```

**What you'll see:**
- Robot model in 3D
- TF frames showing coordinate systems
- Joint State Publisher GUI to manually move steering and wheels

**Try this:**
- Use the sliders in the Joint State Publisher GUI to:
  - Turn the front wheels (steering joints)
  - Rotate all wheels

---

### 2. Run Gazebo Simulation

**Purpose**: Full physics simulation with sensors and control

```bash
ros2 launch diy_ackermann simulation.launch.py
```

**What happens:**
1. Gazebo Fortress opens with a world containing obstacles
2. The Ackermann robot spawns at the origin
3. RViz opens showing the robot and LiDAR data
4. ROS2 bridges connect Gazebo to ROS topics

**Optional arguments:**
```bash
# Spawn at different position
ros2 launch diy_ackermann simulation.launch.py robot_x:=2.0 robot_y:=1.0 robot_yaw:=1.57

# Use different world
ros2 launch diy_ackermann simulation.launch.py world:=my_world.sdf

# Launch without RViz
ros2 launch diy_ackermann simulation.launch.py use_rviz:=false
```

---

### 3. Control the Robot

Once the simulation is running, control the robot using keyboard teleoperation:

**Terminal 1** (if not already running):
```bash
ros2 launch diy_ackermann simulation.launch.py
```

**Terminal 2**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `,` - Move backward  
- `j` - Turn left (steering)
- `l` - Turn right (steering)
- `k` - Stop
- `q/z` - Increase/decrease speed
- `w/x` - Increase/decrease turn rate

**Watch the topics:**
```bash
# In another terminal, monitor the velocity commands
ros2 topic echo /cmd_vel

# Monitor odometry
ros2 topic echo /odom

# Monitor LiDAR scans
ros2 topic echo /scan
```

---

## 🛠️ Customization Guide

### Modify Robot Dimensions

**Example: Make a bigger robot**

1. Edit `urdf/diy_ackermann.urdf.xacro`:
   ```xml
   <xacro:property name="chassis_length" value="2.0"/>  <!-- was 1.0 -->
   <xacro:property name="chassis_width" value="1.0"/>   <!-- was 0.5 -->
   <xacro:property name="wheelbase" value="1.4"/>       <!-- was 0.7 -->
   ```

2. Rebuild:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select diy_ackermann
   source install/setup.bash
   ```

3. Launch and see the changes:
   ```bash
   ros2 launch diy_ackermann rviz.launch.py
   ```

### Change Steering Angle

**Example: Allow sharper turns**

```xml
<xacro:property name="max_steering_angle" value="0.7854"/>  <!-- 45 degrees instead of 30 -->
```

### Add More Sensors

**Example: Add a camera**

1. In the URDF, add a camera link and joint:
   ```xml
   <link name="camera_link">
     <visual>
       <geometry>
         <box size="0.05 0.05 0.05"/>
       </geometry>
       <material name="red"/>
     </visual>
     <collision>
       <geometry>
         <box size="0.05 0.05 0.05"/>
       </geometry>
     </collision>
     <xacro:box_inertia mass="0.1" length="0.05" width="0.05" height="0.05"/>
   </link>
   
   <joint name="camera_joint" type="fixed">
     <origin xyz="${chassis_length/2} 0 ${chassis_height/2 + 0.1}" rpy="0 0 0"/>
     <parent link="chassis"/>
     <child link="camera_link"/>
   </joint>
   ```

2. Add Gazebo camera plugin:
   ```xml
   <gazebo reference="camera_link">
     <sensor name="camera" type="camera">
       <camera>
         <horizontal_fov>1.047</horizontal_fov>
         <image>
           <width>640</width>
           <height>480</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
       </camera>
       <always_on>1</always_on>
       <update_rate>30</update_rate>
       <visualize>true</visualize>
       <topic>camera</topic>
     </sensor>
   </gazebo>
   ```

### Modify World

Edit `worlds/ackermann_world.sdf` to add obstacles:

```xml
<model name="my_obstacle">
  <static>true</static>
  <pose>5 5 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </visual>
  </link>
</model>
```

---

## 📡 Topics and Services

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_description` | `std_msgs/String` | Robot URDF/Xacro description |
| `/joint_states` | `sensor_msgs/JointState` | Current state of all joints |
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position, velocity) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |

### Useful Commands

```bash
# List all topics
ros2 topic list

# See topic details
ros2 topic info /cmd_vel

# Echo a topic
ros2 topic echo /scan

# Publish manually (move forward)
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"

# View TF tree
ros2 run tf2_tools view_frames
```

---

## 🔧 Troubleshooting

### Issue: "Package not found"

**Solution:**
```bash
cd ~/ros2_ws
source install/setup.bash
```

### Issue: Gazebo doesn't open or crashes

**Check Gazebo installation:**
```bash
gz sim --version
```

**If not installed:**
```bash
sudo apt install gz-fortress
```

### Issue: Robot falls through ground in Gazebo

**Cause**: Physics not loaded properly

**Solution**: Make sure the world file has the physics plugin:
```xml
<plugin filename="libignition-gazebo-physics-system.so"
        name="ignition::gazebo::systems::Physics">
</plugin>
```

### Issue: No LiDAR data in RViz

**Check:**
1. Is the bridge running? `ros2 topic list | grep scan`
2. In RViz, check the LaserScan topic is set to `/scan`
3. Ensure the simulation is running (not paused)

### Issue: Robot doesn't move with keyboard commands

**Check:**
1. Is teleop running? Look for "Reading from keyboard" message
2. Is the terminal with teleop focused?
3. Check if cmd_vel is being published: `ros2 topic hz /cmd_vel`

### Issue: "xacro: error: No such file or directory"

**Make sure xacro is installed:**
```bash
sudo apt install ros-humble-xacro
```

---

## 📚 Educational Resources

### Learn More About:

**URDF & Xacro:**
- [URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Tutorial](https://wiki.ros.org/xacro)

**Ackermann Steering:**
- [Ackermann Steering Geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)

**Gazebo:**
- [Gazebo Tutorials](https://gazebosim.org/docs/fortress/tutorials)
- [SDF Format](http://sdformat.org/)

**ROS2:**
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

### Teaching Ideas

1. **Experiment with parameters**: Change wheel sizes, steering angles
2. **Add sensors**: Camera, IMU, GPS
3. **Modify steering**: Implement different steering models
4. **Create scenarios**: Build custom worlds for testing
5. **Implement control**: Write a simple controller node

---

## 📝 License

This project is licensed under the MIT License.

---

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest features
- Submit pull requests
- Improve documentation

---

## 📧 Contact

**Maintainer**: zetans  
**Email**: janssv.200604@gmail.com

---

## 🙏 Acknowledgments

- Inspired by the [gazebo_project](https://github.com/gulerburak/gazebo_project) repository
- ROS2 and Gazebo communities for excellent documentation
- Contributors and users of this package

---

**Happy Learning and Robot Building! 🤖**

If you find this package helpful for teaching or learning, please consider giving it a star ⭐

