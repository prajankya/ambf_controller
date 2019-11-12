
import logging
from colorama import Fore, Back, Style, init

global Logger

# create logger
Logger = logging.getLogger(Fore.MAGENTA + 'Solver')

# if is_verbose:
Logger.setLevel(logging.DEBUG)

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
Logger.addHandler(ch)


Logger.CRITICAL = logging.CRITICAL
Logger.FATAL = logging.FATAL
Logger.ERROR = logging.ERROR
Logger.WARNING = logging.WARNING
Logger.WARN = logging.WARN
Logger.INFO = logging.INFO
Logger.DEBUG = logging.DEBUG
Logger.NOTSET = logging.NOTSET
