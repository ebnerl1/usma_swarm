#!/bin/bash
> raw_data.csv
> parsed_data.js
python waypoint_server.py ; python geogen.py ; xdg-open index.html
