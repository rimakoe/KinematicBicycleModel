# Kinematic Bicycle Model

![codeql.yml](https://github.com/winstxnhdw/KinematicBicycleModel/actions/workflows/codeql.yml/badge.svg)

<div align="center">
    <img src="resources/animation_wide.gif" />
</div>

## Abstract

A python library for the Kinematic Bicycle model. The model can be defined with the following state-space representation,

$$
\frac{d}{dt}
\begin{pmatrix}
x \\
y \\
\theta \\
v
\end{pmatrix} =
\begin{pmatrix}
v\cos{\theta} \\
v\sin{\theta} \\
\frac{v\tan{\delta}}{L} \\
a
\end{pmatrix}
$$

where $v$ is the vehicle's velocity in the x-axis, $\theta$ is the vehicle's yaw, $\delta$ is the steering angle, $L$ is the vehicle's wheelbase, $a$ is the acceleration/throttle, $f$ is friction in the x-axis.

```yaml
At initialisation
:param wheelbase:           (float) vehicle's wheelbase [m]
:param max_steer:           (float) vehicle's steering limits [rad]
:param delta_time:          (float) discrete time period [s]

At every time step  
:param x:                   (float) vehicle's x-coordinate [m]
:param y:                   (float) vehicle's y-coordinate [m]
:param yaw:                 (float) vehicle's heading [rad]
:param velocity:            (float) vehicle's velocity in the x-axis [m/s]
:param acceleration:        (float) vehicle's accleration [m/s^2]
:param steering_angle:      (float) vehicle's steering angle [rad]

:return x:                  (float) vehicle's x-coordinate [m]
:return y:                  (float) vehicle's y-coordinate [m]
:return yaw:                (float) vehicle's heading [rad]
:return velocity:           (float) vehicle's velocity in the x-axis [m/s]
:return steering_angle:     (float) vehicle's steering angle [rad]
:return angular_velocity:   (float) vehicle's angular velocity [rad/s]
```

## Limitations

Just like with all other bicycle models, this model is a discrete model and loses its accuracy when the time step is set too large or the vehicle is made to travel at unreasonably high speeds. Usually, the FPS of the simulation should be set to the highest possible value for the greatest accuracy. However, for rendering high-quality GIFs, 50 FPS is found to be most optimal. See the [GIF89a specification](https://www.w3.org/Graphics/GIF/spec-gif89a.txt).

## Requirements

- Docker
- SSH key to your GitHub account

## Installation

1. Enable on host machine outside of container

    ```(cmd)
    xhost +
    ```

1. Change the directory to the repostiroy main directory.

    ```(cmd)
    cd /directory/to/KinematicBicycleModel
    ```

1. Open vs code.

    ```(cmd)
    code .
    ```

1. Install the [Dev Container Extension](https://code.visualstudio.com/docs/devcontainers/containers) from VS Code and execute Crtl + Shift + P -> Reopen folder in container.

1. Make everything executable with:

    ```(cmd)
    source ./make_all_executable.sh
    ```

1. Create the needed python environments for all submodules with:

    ```(cmd)
    source ./setup_environments.sh
    ```

1. Change the directory to mxck_ws.

    ```(cmd)
    cd mxck_ws
    ```

1. The ```catkin``` command should be recognized by the system. You can verify that via the autocomplete function using 'Tab' button on your keyboard. Configure the catkin build With the following command

    ```(cmd)
    catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3.10 -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so -DSETUPTOOLS_DEB_LAYOUT=OFF
    ```

1. Build the ROS catkin

    ```(cmd)
    catkin build
    ```

    Otherwise

    ```(cmd)
    source /opt/ros/melodic/setup.sh
    ```

    And then execute it.

    All packages should build now. This takes a few seconds.

1. Execute the following command to get the autofill features of ROS:

    ```(cmd)
    source ./devel/setup.bash
    ```

1. Finally play the animation via ROS.

    ```bash
    roslaunch bicycle_model animation.launch
    ```

## Concept

Though our implementation is titled the `Kinematic Bicycle Model`, it does take into account some forward friction. This was never intended to improve the accuracy of the model. Instead, it provides a more intuitive API by removing the need to constantly control the input throttle. However, that is where the differences end. You can read about the bicycle model in full detail by Theers et al., [here](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html).
