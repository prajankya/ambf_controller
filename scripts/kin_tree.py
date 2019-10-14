
import yaml
import numpy as np

from logger import logger as log


class Chain:
    """Upon creation, this class allow to traverse along the Chain structure
    of bodies along joints.

    Here, chain is similar to a tree with one base body and one tip 
    body with any number of  branches in within
    """

    def __init__(self, bodies, joints):
        """Constructor for Chain class, needs dictionary of bodies class and
        Joint class with names of bodies and joints as keys respectively.

        Arguments:
            bodies {dict of Class Body} -- Dictionary of bodies with body name as key.
            joints {dict of Class Joint} -- Dictionary of joints with joint name as key.
        """

        self.bodies = {}

        self.joints = joints

        for joint_name in joints:
            # Get all variables
            parent_body = bodies[joints[joint_name].parent]
            parent_body_name = joints[joint_name].parent
            child_body = bodies[joints[joint_name].child]
            child_body_name = joints[joint_name].child

            # adding parents to the child body
            if child_body.parents == None:
                child_body.parents = []
            if parent_body_name not in child_body.parents:
                child_body.parents.append(parent_body_name)

            # adding children to the parent body
            if parent_body.children == None:
                parent_body.children = []
            if child_body_name not in parent_body.children:
                parent_body.children.append(child_body_name)

            # -------------------------
            # adding parent joints to the child body
            if child_body.parent_joints == None:
                child_body.parent_joints = []
            if joint_name not in child_body.parent_joints:
                child_body.parent_joints.append(joint_name)

            # adding child joints to the parent body
            if parent_body.child_joints == None:
                parent_body.child_joints = []
            if joint_name not in parent_body.child_joints:
                parent_body.child_joints.append(joint_name)

            # add both to list of bodies
            if parent_body_name not in self.bodies:
                self.bodies[parent_body_name] = parent_body
            if child_body_name not in self.bodies:
                self.bodies[child_body_name] = child_body

        self.base_body = None
        self.tip_body = None

        # ATTEMPTING to find base and tip bodies
        for body_name in bodies:
            if len(bodies[body_name].parents) == 0:
                if self.base_body == None:
                    self.base_body = bodies[body_name]
                else:
                    raise ValueError("Found two Base bodies" +
                                     " within the given CHAIN! cannot Generate Chain.")

            if len(bodies[body_name].children) == 0:
                if self.tip_body == None:
                    self.tip_body = bodies[body_name]
                else:
                    raise ValueError("Found two Tip bodies" +
                                     " within the given CHAIN! cannot Generate Chain.")

        # log.debug(yaml.dump(self))
        log.debug("BASE:")
        log.debug(self.base_body)
        log.debug("TIP:")
        log.debug(self.tip_body)


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
        self.parent_joints = []
        self.child_joints = []

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
            # [5:] to remove "BODY " in start of name
            self.parent = yaml_data['parent'][5:]
            # [5:] to remove "BODY " in start of name
            self.child = yaml_data['child'][5:]

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
