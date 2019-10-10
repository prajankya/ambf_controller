
import sys

from ambf_client import Client  # Import the Client from ambf_client package
from colorama import Fore, Back, Style, init


from logger import logger as log


def connect_to_ambf_client():
    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bi-directional communication
    _client.connect()

    # check if AMBF is running by checking number of objects loaded in AMBF(using rostopic inside)
    if(len(_client.get_obj_names()) == 0):
        sys.exit(Fore.RED+"AMBF Simulator is not running")

    # Printing summary of ambf_client node
    log.debug(_client.print_summary())
    log.debug('\n\n----')
    log.debug("List of Objects")
    log.debug(_client.get_obj_names())

    for name in _client.get_obj_names():
        if(name != "World" and name != 'world'):
            obj_hdl = _client.get_obj_handle(name)
            if obj_hdl.get_num_joints() > 0:
                print("Found base link of a robot:"+obj_hdl.get_name())


# log.debug(obj_hdl.get_num_joints())
# # log.debug(obj_hdl.get_name())
# # log.debug(obj_hdl.get_all_joint_pos())
# log.debug('\n\n----')
# # log.debug("List of Joints")
# # log.debug(obj_hdl.get_joint_names())

# # log.debug('\n\n----')
# # log.debug("List of Children Names")
# # log.debug(obj_hdl.get_children_names())
# # log.debug("-----------------")
