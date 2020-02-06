#!/bin/bash
> parsed_data.js
python ~/usma_swarm/usma_files/serverSide/waypoint_server.py &
firefox ~/usma_swarm/usma_files/serverSide/index.html &
