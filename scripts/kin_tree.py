
import yaml
import numpy as np


class Tree:
    """Upon creation, this class allow to traverse along the tree structure 
    of bodies along joints
    """

    def __init__(self, bodies, joints):
        """Constructor for Tree class, needs dictionary of bodies class and
        Joint class with names of bodies and joints as keys respectively. 

        Arguments:
            bodies {dict of Class Body} -- Dictionary of bodies with body name as key.
            joints {dict of Class Joint} -- Dictionary of joints with joint name as key.
        """
        self.bodies = bodies
        self.joints = joints
        self.base_body = None
        self.tip_body = None


class Body:
    """This class defined a generic body which is equivalent to links 
    in a conventional Kinematics solver

    Raises:
        TypeError: If any required key is missing/invalid in the 
        parameters passed
    """

    def __init__(self, yaml_data):
        """Constructor for Body class

        Arguments:
            yaml_data {dict} -- Dictionary of body parameters read by yaml

        Raises:
            TypeError: If name/location key is missing/invalid in the parameters
        """
        try:
            # Mandatory keys, required for kinematics
            self.name = yaml_data['name']

            self.location = {
                'orientation': {
                    'r': float(yaml_data['location']['orientation']['r']),
                    'p': float(yaml_data['location']['orientation']['p']),
                    'y': float(yaml_data['location']['orientation']['y'])
                },
                'position': {
                    'x': float(yaml_data['location']['position']['x']),
                    'y': float(yaml_data['location']['position']['y']),
                    'z': float(yaml_data['location']['position']['z'])
                }
            }
        except ValueError:
            raise TypeError("Body cannot be parsed!")

        # All other optional keys
        try:
            self.mesh = yaml_data['mesh']
        except ValueError:
            pass
        try:
            self.mass = float(yaml_data['mass'])
        except ValueError:
            pass
        try:
            self.scale = float(yaml_data['scale'])
        except ValueError:
            pass
        self.parents = []
        self.children = []

    def __str__(self):
        """String print, this function helps in printing 
        stringified version of class object

        Returns:
            string -- YAML equvivalent of the object
        """
        return yaml.dump(self)


class Joint:
    """This class defined a generic Joint between two bodies 

    Raises:
        TypeError: If any required key is missing/invalid in the 
        parameters passed
    """

    def __init__(self, yaml_data):
        """Constructor for Joint class

        Arguments:
            yaml_data {dict} -- Dictionary of joint parameters read by yaml

        Raises:
            TypeError: If any required key is missing/invalid in the parameters
        """

        try:
            # Mandatory keys, required for kinematics
            self.name = yaml_data['name']
            self.parent = yaml_data['parent']
            self.child = yaml_data['child']

            self.type = yaml_data['type']

            self.parent_axis = {
                'x': float(yaml_data['parent axis']['x']),
                'y': float(yaml_data['parent axis']['y']),
                'z': float(yaml_data['parent axis']['z'])
            }
            self.parent_pivot = {
                'x': float(yaml_data['parent pivot']['x']),
                'y': float(yaml_data['parent pivot']['y']),
                'z': float(yaml_data['parent pivot']['z'])
            }

            self.child_axis = {
                'x': float(yaml_data['child axis']['x']),
                'y': float(yaml_data['child axis']['y']),
                'z': float(yaml_data['child axis']['z'])
            }
            self.child_pivot = {
                'x': float(yaml_data['child pivot']['x']),
                'y': float(yaml_data['child pivot']['y']),
                'z': float(yaml_data['child pivot']['z'])
            }

            self.joint_limits = {
                'high': float(yaml_data['joint limits']['high']),
                'low': float(yaml_data['joint limits']['low'])
            }

            self.controller = {
                'P': float(yaml_data['controller']['P']),
                'I': float(yaml_data['controller']['I']),
                'D': float(yaml_data['controller']['D'])
            }

        except ValueError:
            raise TypeError("Joint cannot be parsed!")

    def __str__(self):
        """String print, this function helps in printing 
        stringified version of class object

        Returns:
            string -- YAML equvivalent of the object
        """
        return yaml.dump(self)
