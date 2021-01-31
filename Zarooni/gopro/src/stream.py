# !/usr/bin/env python3
from goprocam import GoProCamera, constants
gopro=GoProCamera.GoPro()
gopro.overview()
gopro.stream("udp://127.0.0.1:10000")

#you have to install mpv using sudo apt install mpv