import yaml
from Logger import Logger as log


class Pose:
    """6 DOF (x, y, z, rx, ry, rz) Pose
    """
    x = 0
    y = 0
    z = 0

    qx = 0
    qy = 0
    qz = 0
    qw = 1

    def __init__(self, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        """Constructor for Pose with (x, y, z) for position and
        (qx, qy, qz, qw) quaternion for orientation
        """
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def __str__(self):
        """String print, this function helps in printing 
        stringified version of class object

        Returns:
            string -- YAML equvivalent of the object
        """
        return yaml.dump(self)
