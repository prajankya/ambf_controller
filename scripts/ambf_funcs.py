
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

    base_links = []
    for name in _client.get_obj_names():
        if(name != "World" and name != 'world'):
            obj_hdl = _client.get_obj_handle(name)
            if obj_hdl.get_num_joints() > 0:
                log.debug("Found base link of a robot:" + obj_hdl.get_name())
                base_links.append(obj_hdl.get_name())

    if len(base_links) == 0:
        sys.exit(Fore.RED+"NO base links found in AMBF simulator.")
