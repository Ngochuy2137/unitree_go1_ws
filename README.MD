# Unitree_go1_ws
**This repo is for unitree go1 robot bringup**

**Objective:** Packages to simulate Unitree Go1 in Gazebo and control the robot with keyboard based on ROS

## **1. Prerequisite**

- Ubuntu 20.04 with ROS noetic or Ubuntu 18.04 with ROS melodic

## 2. Installation

### 2.1 Clone repo including necessary packages

- cd to a directory you like to install
    
    ```bash
    cd <your favorite directory>
    ```
    
- Clone this repo with submodule mode
    
    ```bash
    git clone --recurse-submodules https://github.com/Ngochuy2137/unitree_go1_ws.git
    ```
    
- Update all sub repos
    
    ```bash
    cd unitree_go1_ws
    git submodule update --init --recursive
    ```
    
- In the unitree_ros repo, branch master, all packages are setup to check out to proper branches for go1 robot. Therefore, you don’t need to checkout or do anything complicatedly.

### 2.2 Install dependencies

- Install ROS dependencies with rosdep
    
    ```bash
    cd <your favorite directory>/unitree_go1_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
    
- Install lcm library if you don’t have
    
    ```jsx
    sudo apt install liblcm-dev
    ```
    

### 2.3 Build unitree_legged_sdk manually

- unitree_ros_to_real isn’t a ROS package, so we need to build it directly by Cmake instead of catkin build or catkin_make
- cd to the package
    
    ```bash
    cd src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk
    ```
    
- If the following steps don’t work, Build the unitree_legged_sdk package by following the instruction in README file
    
    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```
    

### 2.4 Build workspace

```jsx
cd <your favorite directory>/unitree_go1_ws
catkin build
```

## 3. Simulation Running
Reference: [Link](https://github.com/unitreerobotics/unitree_guide/blob/main/README.md)
### Note:

Before running any following command in any terminal, you need to source your workspace before.

```bash
source cd <unitree_go1_ws directory>/devel/setup.bash
```

### 3.1 Launch gazebo simulation

After installing all requirements [Link](https://www.notion.so/2-Installation-b89198feccc54934a9b540d4b7c1f5cc?pvs=21), let ‘s run gazebo simulation

- Firstly, access file normal.launch in unitree_ros/unitree_gazebo/launch/normal.launch
- Modify your robot (Ex: go1) in argument `rname`

```jsx
roslaunch unitree_guide gazeboSim.launch
```

### 3.2 Control the robot with keyboard

```bash
./devel/lib/unitree_guide/junior_ctrl
# or 
# rosrun unitree_guide junior_ctrl
```

> After starting the controller, the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from **Passive**(initial state) to **FixedStand**, then press the '4' key to switch the FSM from **FixedStand** to **Trotting**, now you can press the 'w' 'a' 's' 'd' key to control the translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot will stop and stand on the ground . (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the previous operation)
> 

- When run `rosrun unitree_guide junior_ctrl` , a node named `unitree_gazebo_servo` is generated and it will calculate and publish commands to the following topics, corresponding to each leg joint, which are subscribed by the robot in Gazebo or in reality:
    
    <aside>
    💡
    
    - ***Front Left leg:***
        - /go1_gazebo/FL_calf_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/FL_hip_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/FL_thigh_controller/command [unitree_legged_msgs/MotorCmd]
    - ***Front Right leg:***
        - /go1_gazebo/FR_calf_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/FR_hip_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/FR_thigh_controller/command [unitree_legged_msgs/MotorCmd]
    - ***Rear Left leg:***
        - /go1_gazebo/RL_calf_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/RL_hip_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/RL_thigh_controller/command [unitree_legged_msgs/MotorCmd]
    - ***Rear Right leg:***
        - /go1_gazebo/RR_calf_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/RR_hip_controller/command [unitree_legged_msgs/MotorCmd]
        - /go1_gazebo/RR_thigh_controller/command [unitree_legged_msgs/MotorCmd]
    </aside>
    
- The message type of these topics is unitree_legged_msgs/MotorCmd:
    
    <aside>
    💡
    
    ***unitree_legged_msgs/MotorCmd***
    
    - uint8 mode
    - float32 q
    - float32 dq
    - float32 tau
    - float32 Kp
    - float32 Kd
    - uint32[3] reserve
    </aside>