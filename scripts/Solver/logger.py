
import logging
from colorama import Fore, Back, Style, init

global logger

# create logger
logger = logging.getLogger(Fore.MAGENTA + 'AMBF_ctrl')

# if is_verbose:
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
# if is_verbose:
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter(
    "%(asctime)s - %(name)s - "+Fore.YELLOW+"%(levelname)s"+Fore.RESET+" - %(message)s")

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
