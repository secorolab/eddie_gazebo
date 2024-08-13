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
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FreddyGazeboPublisher(Node):
    def __init__(self, ) -> None:
        super().__init__('FreddyGazeboPublisher')

        # Declare controller parameters
        self.declare_parameter('arm_controller', 'joint_trajectory')
        self.declare_parameter('base_controller', 'velocity')

        valid_controllers = {
            'arm': ['joint_trajectory', 'effort'],
            'base': ['position', 'velocity', 'effort']
        }

        # Getting declared parameters
        self.arm_controller: str = self.get_parameter('arm_controller').get_parameter_value().string_value
        assert self.arm_controller in valid_controllers['arm'], f'Undefined arm_controller: {self.arm_controller}'

        self.base_controller: str = self.get_parameter('base_controller').get_parameter_value().string_value
        assert self.base_controller in valid_controllers['base'], f'Undefined base_controller: {self.base_controller}'

        arm_controller_name = \
            {side: f'arm_{side}_joint_trajectory_controller' if self.arm_controller == 'joint_trajectory' \
                else f'arm_{side}_effort_controller' for side in ['left', 'right']}
        base_controller_name = f'base_{self.base_controller}_controller'

        self.get_logger().info(f"Setting up command for arm controller: {arm_controller_name}")
        self.get_logger().info(f"Setting up command for base controller: {base_controller_name}")

        with open('install/freddy_gazebo/share/freddy_gazebo/config/freddy_controller.yaml') \
                as param_file:
            controller_params = yaml.safe_load(param_file)

        self.components = {
            "arm_left": {
                "publisher": self.create_publisher(
                    JointTrajectory if self.arm_controller == 'joint_trajectory' \
                        else Float64MultiArray, 
                    f'/{arm_controller_name["left"]}/' + \
                        ('joint_trajectory' if self.arm_controller == 'joint_trajectory' \
                            else 'commands'), 
                    10,
                    ),
                "state": np.zeros(7),
                "message": JointTrajectory() if self.arm_controller == 'joint_trajectory' \
                    else Float64MultiArray(),
                "joints": controller_params[arm_controller_name["left"]]\
                    ["ros__parameters"]["joints"],
                "frame_id": "base_link",
                "trajectory_duration": 1,
            },
            "arm_right": {
                "publisher": self.create_publisher(
                    JointTrajectory if self.arm_controller == 'joint_trajectory' \
                        else Float64MultiArray, 
                    f'/{arm_controller_name["right"]}/' + \
                        ('joint_trajectory' if self.arm_controller == 'joint_trajectory' \
                            else 'commands'), 
                    10,
                    ),
                "state": np.zeros(7),
                "message": JointTrajectory() if self.arm_controller == 'joint_trajectory' \
                    else Float64MultiArray(),
                "joints": controller_params[arm_controller_name["right"]]\
                    ["ros__parameters"]["joints"],
                "frame_id": "base_link",
                "trajectory_duration": 1,
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

        for component_name in self.components:
            if "arm" in component_name:
                with open('install/freddy_description/share/freddy_description/'+\
                        f'config/initial_positions_{component_name}.yaml') \
                        as initial_position_file:
                    initial_positions = yaml.safe_load(initial_position_file)
                    
                    self.components[component_name]["state"] = np.array(\
                        [initial_positions[f"joint_{index}"] for index in range(1, 8)]
                        )


        print("Loaded keyboard control interface with the following components:")
        pprint(self.components)


    def update_state(self, component_name: str, increment: np.ndarray, ) -> None:
        # Since commands are of variable length, only take the required number of elements
        print(f"Currently controlling {component_name}")
        self.components[component_name]["state"] += \
            increment[:len(self.components[component_name]["state"])]

        self.update_messages()

        return None


    def update_messages(self, ) -> None:
        for component_name in self.components:
            if "arm" in component_name:
                if self.arm_controller == 'joint_trajectory':
                    msg = JointTrajectory()
                    msg.header = Header()
                    msg.header.frame_id = self.components[component_name]["frame_id"] 
                    msg.joint_names = self.components[component_name]["joints"]

                    trajectory_point = JointTrajectoryPoint()
                    trajectory_point.positions = \
                        self.components[component_name]["state"].tolist()
                    trajectory_point._time_from_start = \
                        Duration(
                            sec=self.components[component_name]["trajectory_duration"], 
                            nanosec=0,
                            )

                    msg.points.append(trajectory_point)
                    self.components[component_name]["message"] = msg

                else:
                    msg = Float64MultiArray()
                    msg.data = self.components[component_name]["state"].tolist()

                    self.components[component_name]["message"] = msg

            elif component_name == "base":
                msg = Float64MultiArray()
                msg.data = self.components[component_name]["state"].tolist()

                self.components[component_name]["message"] = msg

        print(f"Updated states to: ", *[self.components[component_name]["state"] \
                                        for component_name in self.components])

        return None


    def publish_commands(self, ) -> None:
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
                '+': (1.03, 1.1),
                '-': (.6, .9)
        }
        self.component_bindings={
            'q': 'arm_left', # Left arm
            'a': 'arm_right', # Right arm
            'z': 'base' # Base 
        }
        self.speed = 0.1


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
            print(self.speed_bindings[key][1 if component == 'base' else 0])
            self.speed *= self.speed_bindings[key][1 if component == 'base' else 0]

            return []

        elif key == '\x03':
            return None
    
        else:
            return []
        
        increment: np.ndarray = moveBy * self.speed

        return increment
    
    def checkComponent(self,key):
        if key in self.component_bindings.keys():
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
            freddy_gazebo_publisher.publish_commands()
    
    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()



if __name__ == '__main__':
    main()

