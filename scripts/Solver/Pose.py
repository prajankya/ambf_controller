
from Logger import Logger as log


class Pose:
    """6 DOF (x, y, z, rx, ry, rz) Pose
    """
    x = 0
    y = 0
    z = 0

    rx = 0
    ry = 0
    rz = 0

    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        """Constructor for Pose
        """
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
