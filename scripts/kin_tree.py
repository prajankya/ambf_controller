
import sys

import yaml
from collections import OrderedDict


class Body:
    def __init__(self, yaml_data):
        try:
            # Mandatory key, required for kinematics
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
            self.name = yaml_data['name']
        except ValueError:
            pass
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


# Joint Template for the some commonly used of afJoint's data
class Joint:
    def __init__(self):
        self._data = OrderedDict()
        self._data['name'] = ''
        self._data['parent'] = ''
        self._data['child'] = ''
        self._data['parent axis'] = {'x': 0, 'y': 0.0, 'z': 1.0}
        self._data['parent pivot'] = {'x': 0, 'y': 0.0, 'z': 0}
        self._data['child axis'] = {'x': 0, 'y': 0.0, 'z': 1.0}
        self._data['child pivot'] = {'x': 0, 'y': 0.0, 'z': 0}
        self._data['joint limits'] = {'low': -1.2, 'high': 1.2}
        self._data['controller'] = {'P': 1000, 'I': 0, 'D': 1}

    def __str__(self):
        return yaml.dump(self)
