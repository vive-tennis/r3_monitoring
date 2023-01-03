#!/bin/bash

sudo docker run -it --rm \
		-v ~/ros:/mnt/ros \
		--net host \
		ros:noetic-perception \
		/bin/bash
		#/bin/bash -c "source /opt/ros/noetic/setup.bash && python3 /mnt/ros/r3_monitoring_client.py"


