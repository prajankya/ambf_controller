
import yaml


class Body:
    def __init__(self, yaml_data):
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

    def __str__(self):
        return yaml.dump(self)


class Joint:
    def __init__(self, yaml_data):
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
        return yaml.dump(self)
