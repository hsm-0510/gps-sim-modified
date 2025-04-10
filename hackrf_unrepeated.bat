@echo off

:: Set the window title
title HackRF Repeated

start cmd /k hackrf_transfer -t gpssim.bin -f 1575420000 -s 2600000 -a 1 -p 1 -x 47