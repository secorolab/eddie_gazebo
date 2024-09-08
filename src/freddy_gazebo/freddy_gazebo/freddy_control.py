import sys
import termios
import threading
import tty
from pprint import pprint

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def clear_line(n=1):
    '''
    Function for carriaged printing from 
    https://itnext.io/overwrite-previously-printed-lines-4218a9563527
    '''
    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    for i in range(n):
        print(LINE_UP, end=LINE_CLEAR)


class FreddyGazeboPublisher(Node):
    """
    This class is responsible for publishing commands to the robot's components.
    
    It loads configuration from YAML files, sets up publishers and subscribers,
    and updates the state of each component based on incoming commands.
    
    Parameters:
    verbose (bool): Whether to print debug messages or not
    """
    def __init__(self, 
                 verbose=False) -> None:
        """
        Initializes the FreddyGazeboPublisher instance
        
        Args:
            verbose (bool): Whether to print debug messages or not
        """
        super().__init__('FreddyGazeboPublisher')
        self.verbose = verbose

        # Declare controller parameters
        self.declare_parameter('arm_controller', 'joint_trajectory')
        self.declare_parameter('base_controller', 'velocity')

        valid_controllers = {
            'arm': ['joint_trajectory_position', 'joint_trajectory_velocity', 'effort'],
            'base': ['position', 'velocity', 'effort']
        }

        # Timer for rolling state forward
        self.roll_forward_timer = self.create_timer(0.1, self.roll_state_forward)

        # Getting declared parameters
        self.arm_controller: str = self.get_parameter('arm_controller').get_parameter_value().string_value
        assert self.arm_controller in valid_controllers['arm'], f'Undefined arm_controller: {self.arm_controller}'

        self.base_controller: str = self.get_parameter('base_controller').get_parameter_value().string_value
        assert self.base_controller in valid_controllers['base'], f'Undefined base_controller: {self.base_controller}'

        arm_controller_name = \
            {side: f'arm_{side}_joint_trajectory_controller' if 'joint_trajectory' in self.arm_controller \
                else f'arm_{side}_effort_controller' for side in ['left', 'right']}
        base_controller_name = f'base_{self.base_controller}_controller'

        self.get_logger().info(f"Setting up command interface for selected arm controller: {arm_controller_name}")
        self.get_logger().info(f"Setting up command interface for selected base controller: {base_controller_name}")

        with open('install/freddy_gazebo/share/freddy_gazebo/config/freddy_controller.yaml') \
                as param_file:
            controller_params = yaml.safe_load(param_file)

        self.trajectory_durations = {
            "position": (1, 0),
            "velocity": (0, int(10e7)),
            "effort": None,
        }

        self.components = {
            "arm_left": {
                "publisher": self.create_publisher(
                    JointTrajectory if "joint_trajectory" in self.arm_controller \
                        else Float64MultiArray, 
                    f'/{arm_controller_name["left"]}/' + \
                        ("joint_trajectory" if "joint_trajectory" in self.arm_controller \
                            else 'commands'), 
                    10,
                    ),
                "state": {
                    "position": np.zeros(7),
                    "velocity": np.zeros(7),
                    "effort": np.zeros(7),
                    },
                "message": JointTrajectory() if "joint_trajectory" in self.arm_controller \
                    else Float64MultiArray(),
                "joints": controller_params[arm_controller_name["left"]]\
                    ["ros__parameters"]["joints"],
                "frame_id": "base_link",
                "trajectory_duration": self.trajectory_durations[self.arm_controller.split("_")[-1]],
            },
            "arm_right": {
                "publisher": self.create_publisher(
                    JointTrajectory if "joint_trajectory" in self.arm_controller \
                        else Float64MultiArray, 
                    f'/{arm_controller_name["right"]}/' + \
                        ("joint_trajectory" if "joint_trajectory" in self.arm_controller \
                            else 'commands'), 
                    10,
                    ),
                "state": {
                    "position": np.zeros(7),
                    "velocity": np.zeros(7),
                    "effort": np.zeros(7),
                    },
                "message": JointTrajectory() if "joint_trajectory" in self.arm_controller \
                    else Float64MultiArray(),
                "joints": controller_params[arm_controller_name["right"]]\
                    ["ros__parameters"]["joints"],
                "frame_id": "base_link",
                "trajectory_duration": self.trajectory_durations[self.arm_controller.split("_")[-1]],
            },
            "base": {
                "publisher": self.create_publisher(
                    Float64MultiArray, 
                    f'/{base_controller_name}/commands', 
                    10,
                    ),
                "state": np.zeros(8),
                "message": Float64MultiArray(),
            },
        }

        # Load initial positions for the arms if using joint_trajectory control for position control
        for component_name in self.components:
            if "arm" in component_name:
                if "joint_trajectory" in self.arm_controller:
                    with open('install/freddy_description/share/freddy_description/'+\
                            f'config/initial_positions_{component_name}.yaml') \
                            as initial_position_file:
                        initial_positions = yaml.safe_load(initial_position_file)

                        self.components[component_name]["state"]["position"] = np.array(\
                            [initial_positions[f"joint_{index}"] for index in range(1, 8)]
                            )

        if self.verbose:
            self.get_logger().info("Loaded keyboard control interface with the following components:")
            pprint(self.components)

        self.rollout_timing = {
            "previous": self.evaluate_time(self.get_clock().now()),
            "current": self.evaluate_time(self.get_clock().now()),
        }

    
    def evaluate_time(self, time_object: rclpy.time.Time):
        # Printing this somehow produces a fancy 'slide to the right' animation 
        # print(time_object.seconds_nanoseconds()[0] + (time_object.seconds_nanoseconds()[1] * 1e-9))
        return time_object.seconds_nanoseconds()[0] + (time_object.seconds_nanoseconds()[1] * 1e-9)
    

    def update_state(self, component_name: str, increment: np.ndarray, ) -> None:
        """
        Updates the state of a given component
        
        Args:
            component_name (str): Name of the component to update
            increment (np.ndarray): Increment to apply to the component's state
        """
        # Since commands are of variable length, only take the required number of elements
        if 'arm' in component_name:
            if self.arm_controller.split("_")[-1] == "position":
                self.components[component_name]["state"]["position"] += \
                    increment[:len(self.components[component_name]["state"]["position"])]
            elif self.arm_controller.split("_")[-1] == "velocity":
                self.components[component_name]["state"]["velocity"] += \
                    increment[:len(self.components[component_name]["state"]["velocity"])]
            elif self.arm_controller == "effort":
                self.components[component_name]["state"]["effort"] += \
                    increment[:len(self.components[component_name]["state"]["effort"])]
        else:
            self.components[component_name]["state"] += \
                increment[:len(self.components[component_name]["state"])]

        self.update_messages(component_name)

        return None


    def roll_state_forward(self, ) -> None:
        self.rollout_timing["current"] = self.evaluate_time(self.get_clock().now())
        time_step = self.rollout_timing["current"] - self.rollout_timing["previous"]

        for component_name in self.components:
            if "arm" in component_name:
                if self.arm_controller.split("_")[-1] == "velocity":
                    self.components[component_name]["state"]["position"] += \
                        self.components[component_name]["state"]["velocity"] * time_step
                    
            # Add more state rollouts if required

        self.rollout_timing["previous"] = self.rollout_timing["current"]
        self.update_messages()
        self.publish_commands()


    def update_messages(self, current_component=None) -> None:
        """
        Updates the messages for all components
        
        Args:
            current_component (str): Optional name of the component for printing
        """
        for component_name in self.components:
            if "arm" in component_name:
                if "joint_trajectory" in self.arm_controller:
                    msg = JointTrajectory()
                    msg.header = Header()
                    msg.header.frame_id = self.components[component_name]["frame_id"] 
                    msg.joint_names = self.components[component_name]["joints"]

                    trajectory_point = JointTrajectoryPoint()
                    trajectory_point.positions = \
                        self.components[component_name]["state"]["position"].tolist()
                    
                    trajectory_point.time_from_start = \
                        Duration(
                            sec=self.components[component_name]["trajectory_duration"][0], 
                            nanosec=self.components[component_name]["trajectory_duration"][1],
                            )
                    
                    if self.arm_controller.split("_")[-1] == "velocity":
                        trajectory_point.velocities = \
                            self.components[component_name]["state"]["velocity"].tolist()

                    msg.points.append(trajectory_point)
                    self.components[component_name]["message"] = msg

                elif self.arm_controller == 'effort':
                    msg = Float64MultiArray()
                    msg.data = self.components[component_name]["state"]["effort"].tolist()

                    self.components[component_name]["message"] = msg

            elif component_name == "base":
                msg = Float64MultiArray()
                msg.data = self.components[component_name]["state"].tolist()

                self.components[component_name]["message"] = msg

        with np.printoptions(suppress=True):
            if current_component is not None:
                clear_line()
                if "arm" in current_component:
                    print(f"Updated state of {current_component} to: ", \
                          np.array(self.components[current_component]["state"][self.arm_controller.split("_")[-1]]))
                else:
                    print(f"Updated state of {current_component} to: ", \
                          np.array(self.components[current_component]["state"]))
            elif self.verbose:
                print(f"Updated states to: ", \
                      [np.array(self.components[component]["state"]) for component in self.components])

        return None


    def publish_commands(self, ) -> None:
        """
        Publishes commands for all components
        """
        for component_name in self.components:
            # print(self.components[component_name]["message"])
            self.components[component_name]["publisher"].publish(
                self.components[component_name]["message"]
                )

        return None



class KeyboardPress():

    def __init__(self):

        self.key_bindings={
            'w': (1, 0, 0, 0, 0, 0, 0, 0),
            's': (-1, 0, 0, 0, 0, 0, 0, 0),
            'e': (0, 1, 0, 0, 0, 0, 0, 0),
            'd': (0, -1, 0, 0, 0, 0, 0, 0),
            'r': (0, 0, 1, 0, 0, 0, 0, 0),
            'f': (0, 0, -1, 0, 0, 0, 0, 0),
            't': (0, 0, 0, 1, 0, 0, 0, 0),
            'g': (0, 0, 0, -1, 0, 0, 0, 0),
            'y': (0, 0, 0, 0, 1, 0, 0, 0),
            'h': (0, 0, 0, 0, -1, 0, 0, 0),
            'u': (0, 0, 0, 0, 0, 1, 0, 0),
            'j': (0, 0, 0, 0, 0, -1, 0, 0),
            'i': (0, 0, 0, 0, 0, 0, 1, 0),
            'k': (0, 0, 0, 0, 0, 0, -1, 0),
            'o': (0, 0, 0, 0, 0, 0, 0, 1),
            'l': (0, 0, 0, 0, 0, 0, 0, -1),
        }
        self.speed_bindings={
                '+': (1.05, 1.1),
                '-': (.95, .9)
        }
        self.component_bindings={
            'q': 'arm_left', # Left arm
            'a': 'arm_right', # Right arm
            'z': 'base' # Base 
        }
        self.default_speed = 0.1
        self.speed = 0.1

        self.intro_message = ''.join([
        '\n',
        'Keyboard interface for commanding the Freddy robot in simulation\n',
        '\n',
        'Increment state using     w   e   r   t   y   u   i   o\n',
        '                          ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑\n',
        '                Joint     1   2   3   4   5   6   7   8\n',
        '                          ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓\n',
        'Decrement state using     s   d   f   g   h   j   k   l\n',
        '\n',
        'Increment state increment step using    +\n',
        '                                        ↑\n',
        '                                        ↓\n',
        'Decrement state decrement step using    -\n',
        '\n',
        'Choose robot component to command using\n',
        '                                        q : arm_left\n'
        '                                        a : arm_right\n'
        '                                        z : base\n'
        '\n'
        ])
        print(self.intro_message)

    def getKey(self,settings):
    
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    

    def modifyPosition(self, key, component) -> np.ndarray:

        # key = self.getKey(termios.tcgetattr(sys.stdin))
        if key in self.key_bindings.keys():
            moveBy: np.ndarray = np.array(self.key_bindings[key])

        elif key in self.speed_bindings.keys():
            self.speed *= self.speed_bindings[key][1 if component == 'base' else 0]
            
            clear_line()
            print(f'Updated increment step for {component} to {self.speed}')

            return []

        elif key == '\x03':
            return None
    
        else:
            return []
        
        increment: np.ndarray = moveBy * self.speed

        return increment
    
    def checkComponent(self,key):
        if key in self.component_bindings.keys():
            self.speed = self.default_speed

            clear_line()
            print(f'Switched to component {self.component_bindings[key]}')

            return self.component_bindings[key]

        return []
            


def main(args=None):
    rclpy.init(args=args)
    freddy_gazebo_publisher = FreddyGazeboPublisher()
    keyboard_press = KeyboardPress()
    component = 'arm_left' # by default component = base
    
    spinner = threading.Thread(target=rclpy.spin, args=(freddy_gazebo_publisher,))
    spinner.start()

    try:
        while True:
            # Get pressed key
            key = keyboard_press.getKey(termios.tcgetattr(sys.stdin))
            component_check = keyboard_press.checkComponent(key)
            if component_check != []:
                component = component_check
                continue

            increment = keyboard_press.modifyPosition(key,component)

            # if component == []:
            #     raise Exception('Key does not correspond to any component, 
            #     press \'q\': Left arm, \'a\': Right arm, \'z\': Base')

            # Update commands
            if increment is None:
                print("KeyboardInterrupt received, exiting.")
                break

            elif len(increment):
                freddy_gazebo_publisher.update_state(component, increment)

            # Publish messages
            freddy_gazebo_publisher.roll_state_forward()
            freddy_gazebo_publisher.publish_commands()
    
    # except Exception as e:
    #     print(e)

    finally:
        rclpy.shutdown()
        spinner.join()



if __name__ == '__main__':
    main()

