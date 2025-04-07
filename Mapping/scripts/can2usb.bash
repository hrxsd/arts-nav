#!/bin/bash

# bring up can interface
sudo ip link set can3 up type can bitrate 500000

sudo ip link set can0 up type can bitrate 1000000