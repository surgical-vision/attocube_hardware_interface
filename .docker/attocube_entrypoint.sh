#!/bin/bash
set -e

# setup ros environment
source "/catkin_ws/devel/setup.bash"
source "/nigel_start.sh"
exec "$@"