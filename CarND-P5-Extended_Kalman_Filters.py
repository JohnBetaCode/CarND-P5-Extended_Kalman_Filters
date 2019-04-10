#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Date: 04/08/2019
	Programmer: John A. Betancourt G.
	Mail: john.betancourt93@gmail.com
    Web: www.linkedin.com/in/jhon-alberto-betancourt-gonzalez-345557129

Description: Project 8 - Udacity - self driving cars Nanodegree
    PID Controller

Tested on: 
    python 3.5
    UBUNTU 16.04
"""

# =============================================================================
# LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPEN
# =============================================================================
import os
from multiprocessing import Pool 
import subprocess

# =============================================================================
def run_process(process):                                                             
    os.system('{}'.format(process)) 

def getClipboardData():
    p = subprocess.Popen(['xclip','-selection', 'clipboard', '-o'], 
        stdout=subprocess.PIPE)
    retcode = p.wait()
    data = p.stdout.read()
    return data

def setClipboardData(data):
    p = subprocess.Popen(['xclip','-selection','clipboard'], 
        stdin=subprocess.PIPE)
    p.stdin.write(data)
    p.stdin.close()
    retcode = p.wait()

def main():
    # Copy command to clipboard to Create Video from desktop
    video_name = "PID_controller_test{}.mp4".format(1)
    command = "ffmpeg -video_size {}x{} -framerate {} -f x11grab -i :0.0+100,200 {}".format(
        1000, 650, 30, os.path.join(os.getcwd(), "video_results", video_name))
    setClipboardData(command.encode())

    # Run make in build fodler
    os.system('cd build && make') 

    # Run subprocess
    try:
        processes = (
            "{}".format(os.path.join(os.getcwd(), "term2_sim_linux", "term2_sim.x86_64")),
            "{}".format(os.path.join(os.getcwd(), "build", "pid"))
        )
        # Run simulator and socket
        pool = Pool(processes=len(processes))                                                        
        pool.map(run_process, processes)
    except Exception as e: 
        print(str(e))

    print("Process has finished")

# =============================================================================
if __name__=="__main__":
    main()

# =============================================================================