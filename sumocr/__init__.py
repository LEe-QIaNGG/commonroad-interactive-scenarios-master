import logging
import os
import sys
from logging import NullHandler

logging.getLogger("sumocr").addHandler(NullHandler())

# REFACTOR: should we keep this check?
# make sure $SUMO_HOME is in system path
if "SUMO_HOME" in os.environ:
    sumo_installed = True
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    if tools not in sys.path:
        sys.path.append(tools)
else:
    sumo_installed = False

# REFACTOR: this should definitelyz be configurable by the caller. Furthermore this is redefined multiple times throughout the codebase.
DOCKER_REGISTRY = "gitlab.lrz.de:5005/cps/sumo-interface/sumo_docker"
