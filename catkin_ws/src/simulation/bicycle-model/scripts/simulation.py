#!/usr/bin/env /workspaces/KinematicBicycleModel/catkin_ws/src/simulation/bicycle-model/venv/bin/python3

# IDEA: 
# - The simulation runs with a steady fps rate that is configurable.
# - The Input values
from time import time_ns

from csv import reader
from dataclasses import dataclass
from math import radians

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from bicycle_model import KinematicBicycleModel
from car_description import CarDescription
from vesc_msgs.msg import VescStateStamped
from as_state_estimation_msg.msg import StateEstimationStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import rospy
import numpy as np
from watchdog import Watchdog


class Car:

    def __init__(self, init_x, init_y, init_yaw, sample_time):

        self.__x_init = init_x
        self.__y_init = init_y
        self.__yaw_init = init_yaw

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw

        #
        self.sample_time = sample_time
        self.time = 0.0
        self.velocity = 0.0
        self.wheel_angle = 0.0
        self.angular_velocity = 0.0
        max_steer = radians(33)
        wheelbase = 2.96

        # Acceleration parameters
        target_velocity = 7.0
        self.time_to_reach_target_velocity = 1.0
        self.required_acceleration = target_velocity / self.time_to_reach_target_velocity

        # Description parameters
        self.colour = 'black'
        overall_length = 4.97
        overall_width = 1.964
        tyre_diameter = 0.4826
        tyre_width = 0.265
        axle_track = 1.7
        rear_overhang = 0.5 * (overall_length - wheelbase)

        self.kinematic_bicycle_model = KinematicBicycleModel(wheelbase, max_steer, self.sample_time)
        self.description = CarDescription(overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase)

    def reset(self):
        self.x = self.__x_init
        self.y = self.__y_init
        self.yaw = self.__yaw_init
        self.wheel_angle = 0.0


    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def get_required_acceleration(self):
        self.time += self.sample_time
        return self.required_acceleration
    
    def plot_car(self):
        return self.description.plot_car(self.x, self.y, self.yaw, self.wheel_angle)

    def drive(self, throttle, wheel_angle):
        acceleration = 0 if self.time > self.time_to_reach_target_velocity else self.get_required_acceleration()
        self.x, self.y, self.yaw, self.velocity, _, _ = self.kinematic_bicycle_model.update(self.x, self.y, self.yaw, self.velocity, acceleration, wheel_angle)


class Simulation:

    FPS = 50
    SAMPLE_TIME = 1/FPS

    def __init__(self):

        # Outport
        self.publisher_state_estimation = rospy.Publisher('/as_simulation/output/StateEstimationStamped', StateEstimationStamped, queue_size=1) 

        # Inport
        self.subscriber_ackermann = rospy.Subscriber('/as_vehicle_control/output/ackermann_cmd', AckermannDriveStamped, self.update_inputs)
        self.subscriber_path = rospy.Subscriber('/fake/as_path_planning/output/Path', Path, self.update_path)
        self.subscriber_target_point = rospy.Subscriber('/as_vehicle_control/output/TargetPoint', Point, self.update_target_point)

        self.map_size_x = 70
        self.map_size_y = 40
        self.frames = 5000
        self.loop = False

        self.steering_angle = 0
        self.speed = 0
        self.timestamp = 0
        self.path = Path()
        self.target_point = Point()

        # violation = lambda : print("violated")
        self.watchdog = Watchdog(self.reset_ego_car_pose)
        
    def start(self):
        self.watchdog.start()

    def set_ego_car(self, ego_car: Car):
        self.ego_car = ego_car

    def reset_ego_car_pose(self):
        self.ego_car.x = 0
        self.ego_car.y = 0
        self.ego_car.yaw = 0
    
    def reset(self) -> bool:
        return self.__reset

    def update_inputs(self, data : AckermannDriveStamped):
        self.watchdog.trigger()

        self.steering_angle = data.drive.steering_angle
        self.speed = data.drive.speed
        self.timestamp = data.header.stamp
        
    def update_path(self, data : Path):
        self.path = data

    def update_target_point(self, data : Point):
        self.target_point = data

@dataclass
class Fargs:
    ax: plt.Axes
    sim: Simulation
    car: Car
    car_outline: plt.Line2D
    front_right_wheel: plt.Line2D
    front_left_wheel: plt.Line2D
    rear_right_wheel: plt.Line2D
    rear_left_wheel: plt.Line2D
    rear_axle: plt.Line2D
    annotation: plt.Annotation
    target_point: plt.Line2D
    desired_path: plt.Line2D
    is_alive : callable
   

def animate(frame, fargs : Fargs):

    ax                = fargs.ax
    sim               = fargs.sim
    car               = fargs.car
    car_outline       = fargs.car_outline
    front_right_wheel = fargs.front_right_wheel
    front_left_wheel  = fargs.front_left_wheel
    rear_right_wheel  = fargs.rear_right_wheel
    rear_left_wheel   = fargs.rear_left_wheel
    rear_axle         = fargs.rear_axle
    annotation        = fargs.annotation
    target_point      = fargs.target_point
    desired_path      = fargs.desired_path
    is_alive = fargs.is_alive


    # Camera tracks car
    ax.set_xlim(car.x - sim.map_size_x, car.x + sim.map_size_x)
    ax.set_ylim(car.y - sim.map_size_y, car.y + sim.map_size_y)

    if(not is_alive()):
        car.x = 0.0
        car.y = -2.0
        car.yaw = 0.0
        car.wheel_angle = 0.0
        sim.watchdog.reset()
        sim.watchdog.start()


    # Drive
    car.drive(0, sim.steering_angle)
    
    message_state_estimation = StateEstimationStamped()
    message_state_estimation.header.stamp = rospy.Time.now()
    message_state_estimation.state.speed = car.velocity
    message_state_estimation.state.pose.position.x = car.x
    message_state_estimation.state.pose.position.y = car.y
    message_state_estimation.state.pose.position.y = car.y
    
    sim.publisher_state_estimation.publish(message_state_estimation)
    
    # Draw
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car.plot_car()
    car_outline.set_data(*outline_plot)
    front_right_wheel.set_data(*fr_plot)
    rear_right_wheel.set_data(*rr_plot)
    front_left_wheel.set_data(*fl_plot)
    rear_left_wheel.set_data(*rl_plot)
    rear_axle.set_data(car.x, car.y)
    

    xs = []
    ys = []
    pose : PoseStamped
    for pose in sim.path.poses:
        xs.append(pose.pose.position.x)
        ys.append(pose.pose.position.y)
    
    desired_path.set_data(np.array(xs), np.array(ys))
    
    target_point.set_data(sim.target_point.x, sim.target_point.y)

    # Annotate car's coordinate above car
    annotation.set_text(f'{car.x:.1f}, {car.y:.1f}')
    annotation.set_position((car.x, car.y + 5))
    
    plt.title(f'{sim.SAMPLE_TIME*frame:.2f}s', loc='right')
    plt.xlabel(f'Speed: {car.velocity:.2f} m/s', loc='left')

    return car_outline, front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel, rear_axle, desired_path,


if __name__ == '__main__':
    rospy.init_node('as_vehicle_simulation', anonymous=True)

    x_init = 0
    y_init = -2
    yaw_init = 0


    simulation  = Simulation()
    ego_car  = Car(x_init, y_init, yaw_init, simulation.SAMPLE_TIME)
    simulation.set_ego_car(ego_car)

    interval = simulation.SAMPLE_TIME * 10**3

    fig = plt.figure()
    ax = plt.axes()
    ax.set_aspect('equal')

    road = plt.Circle((0, 0), 50, color='gray', fill=False, linewidth=30)
    ax.add_patch(road)

    empty              = ([], [])
    target,            = ax.plot(*empty, '+r')
    car_outline,       = ax.plot(*empty, color=ego_car.colour)
    front_right_wheel, = ax.plot(*empty, color=ego_car.colour)
    rear_right_wheel,  = ax.plot(*empty, color=ego_car.colour)
    front_left_wheel,  = ax.plot(*empty, color=ego_car.colour)
    rear_left_wheel,   = ax.plot(*empty, color=ego_car.colour)
    rear_axle,         = ax.plot(ego_car.x, ego_car.y, '+', color=ego_car.colour, markersize=2)
    annotation         = ax.annotate(f'{ego_car.x:.1f}, {ego_car.y:.1f}', xy=(ego_car.x, ego_car.y + 5), color='black', annotation_clip=False)
    desired_path,       = ax.plot(*empty, 'b')
    
    fargs = [Fargs(
        ax=ax,
        sim=simulation,
        car=ego_car,
        car_outline=car_outline,
        front_right_wheel=front_right_wheel,
        front_left_wheel=front_left_wheel,
        rear_right_wheel=rear_right_wheel,
        rear_left_wheel=rear_left_wheel,
        rear_axle=rear_axle,
        annotation=annotation,
        target_point=target,
        desired_path=desired_path,
        is_alive=simulation.watchdog.is_alive
    )]

    simulation.start()

    _ = FuncAnimation(fig, animate, frames=simulation.frames, init_func=lambda: None, fargs=fargs, interval=interval, repeat=simulation.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)
    
    plt.grid()
    plt.show()
