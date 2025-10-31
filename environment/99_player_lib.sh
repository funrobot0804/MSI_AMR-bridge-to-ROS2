#!/bin/bash
# Player library auto path
export PYTHONPATH=$PYTHONPATH:$AMENT_PREFIX_PATH/share/player_bridge/client_lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$AMENT_PREFIX_PATH/share/player_bridge/client_lib