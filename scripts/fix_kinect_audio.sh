#!/bin/bash
jack_control start
pulseaudio -D
pacmd set-default-source alsa_output.platform-soc_audio.analog-stereo.monitor 
