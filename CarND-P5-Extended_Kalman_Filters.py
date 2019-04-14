#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Date: 04/14/2019
	Programmer: John A. Betancourt G.
	Mail: john.betancourt93@gmail.com
    Web: www.linkedin.com/in/jhon-alberto-betancourt-gonzalez-345557129

Description: Project 5 - Udacity - self driving cars Nanodegree
    Extended Kalman Filter

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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

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

def plot_kalman_graph(kalman_file_path, save_file=None):

    # -------------------------------------------------------------------------
    # Variables inizialitation
    ground_truth_x = []
    ground_truth_y = []
    ground_truth_vx = [] 
    ground_truth_vy = [] 
    estimate_x = [] 
    estimate_y = [] 
    estimate_vx = [] 
    estimate_vy = [] 
    rmse_x = [] 
    rmse_y = [] 
    rmse_vx = [] 
    rmse_vy = [] 
    
    # Read csv file for steering pid controller
    with open(kalman_file_path) as csvfile:
        reader = csv.reader(csvfile)
        for idx, line in enumerate(reader):
            if idx:
                ground_truth_x.append(float(line[0])) # ground_truth_x 
                ground_truth_y.append(float(line[1])) # ground_truth_y 
                ground_truth_vx.append(float(line[2])) # ground_truth_vx 
                ground_truth_vy.append(float(line[3])) # ground_truth_vy 
                estimate_x.append(float(line[4])) # estimate_x 
                estimate_y.append(float(line[5])) # estimate_y
                estimate_vx.append(float(line[6])) # estimate_x 
                estimate_vy.append(float(line[7])) # estimate_y  
                rmse_x.append(float(line[8])) # rmse_x 
                rmse_y.append(float(line[9])) # rmse_y 
                rmse_vx.append(float(line[10])) # rmse_vx 
                rmse_vy.append(float(line[11])) # rmse_vy 
    
    # -------------------------------------------------------------------------
    plt.figure(figsize=(30,20))

    plt.subplot(2, 2, 1)
    plt.plot(ground_truth_x/np.max(ground_truth_x), '#0000FF')
    plt.plot(estimate_x/np.max(estimate_x), '#FF8000')
    plt.plot(rmse_x/np.max(rmse_x), '#FF0000')
    ground_truth_x_patch = mpatches.Patch(color='#0000FF', label='ground_truth_x - max:{}'.format(round(np.max(estimate_x), 2)))
    estimate_x_patch = mpatches.Patch(color='#FF8000', label='estimate_x - max:{}'.format(round(np.max(estimate_x), 2)))
    rmse_x_patch = mpatches.Patch(color='#FF0000', label='rmse_x - max:{}'.format(round(np.max(rmse_x), 2)))
    plt.legend(handles=[ground_truth_x_patch, estimate_x_patch, rmse_x_patch])
    plt.title('Cars X position')
    plt.xlabel('Iteration (t/[s])')
    plt.ylabel('Position [m]')
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(ground_truth_y/np.max(ground_truth_y), '#0000FF')
    plt.plot(estimate_y/np.max(estimate_y), '#FF8000')
    plt.plot(rmse_y/np.max(rmse_y), '#FF0000')
    ground_truth_y_patch = mpatches.Patch(color='#0000FF', label='ground_truth_y - max:{}'.format(round(np.max(estimate_y), 2)))
    estimate_y_patch = mpatches.Patch(color='#FF8000', label='estimate_y - max:{}'.format(round(np.max(estimate_y), 2)))
    rmse_y_patch = mpatches.Patch(color='#FF0000', label='rmse_y - max:{}'.format(round(np.max(rmse_y), 2)))
    plt.legend(handles=[ground_truth_y_patch, estimate_y_patch, rmse_y_patch])
    plt.title('Cars Y position')
    plt.xlabel('Iteration (t/[s])')
    plt.ylabel('Position [m]')
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(ground_truth_vx/np.max(ground_truth_vx), '#0000FF')
    plt.plot(estimate_vx/np.max(estimate_vx), '#8000FF')
    plt.plot(rmse_vx/np.max(rmse_vx), '#FF0000')
    ground_truth_vx_patch = mpatches.Patch(color='#0000FF', label='ground_truth_vx - max:{}'.format(round(np.max(ground_truth_vx), 2)))
    estimate_vx_patch = mpatches.Patch(color='#8000FF', label='estimate_vx - max:{}'.format(round(np.max(estimate_vx), 2)))
    rmse_vx_patch = mpatches.Patch(color='#FF0000', label='rmse_vx - max:{}'.format(round(np.max(rmse_vx), 2)))
    plt.legend(handles=[ground_truth_vx_patch, estimate_vx_patch, rmse_vx_patch])
    plt.title('Cars Vx Speed')
    plt.xlabel('Iteration (t/[s])')
    plt.ylabel('Vy Speed [m/s]')
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(ground_truth_vy/np.max(ground_truth_vy), '#0000FF')
    plt.plot(estimate_vy/np.max(estimate_vy), '#8000FF')
    plt.plot(rmse_vy/np.max(rmse_vy), '#FF0000')
    ground_truth_vy_patch = mpatches.Patch(color='#0000FF', label='ground_truth_vy - max:{}'.format(round(np.max(ground_truth_vy), 2)))
    estimate_vy_patch = mpatches.Patch(color='#8000FF', label='estimate_vy - max:{}'.format(round(np.max(estimate_vy), 2)))
    rmse_vy_patch = mpatches.Patch(color='#FF0000', label='rmse_vy - max:{}'.format(round(np.max(rmse_vy), 2)))
    plt.legend(handles=[ground_truth_vy_patch, estimate_vy_patch, rmse_vy_patch])
    plt.title('Cars Vy Speed')
    plt.xlabel('Iteration (t/[s])')
    plt.ylabel('Vy Speed [m/s]')
    plt.grid(True)

    # -------------------------------------------------------------------------
    if save_file is not None:
        plt.savefig(save_file)

    # Plot graphs
    plt.show()

def main():
    # Copy command to clipboard to Create Video from desktop
    video_name = "kalman_filter_result_{}.mp4".format(1)
    command = "ffmpeg -video_size {}x{} -framerate {} -f x11grab -i :0.0+100,200 {}".format(
        1200, 750, 30, os.path.join(os.getcwd(), "video_results", video_name))
    setClipboardData(command.encode())

    # Run make in build fodler
    os.system('cd build && make') 

    # Run subprocess
    try:
        processes = (
            "{}".format(os.path.join(os.getcwd(), "term2_sim_linux", "term2_sim.x86_64")),
            "{}".format(os.path.join(os.getcwd(), "build", "ExtendedKF"))
        )
        # Run simulator and socket
        pool = Pool(processes=len(processes))                                                        
        pool.map(run_process, processes)
    except Exception as e: 
        print(str(e))

    print("Process has finished")

# =============================================================================
if __name__=="__main__":

    # Run all stuff
    main()

    # Plot steering controller graph
    plot_kalman_graph(
        kalman_file_path="Extender_Kalman_Filter-Dataset_1.csv", 
        save_file="writeup_files/Extender_Kalman_Filter-Dataset_1_graph.png"
        )
    plot_kalman_graph(
        kalman_file_path="Extender_Kalman_Filter-Dataset_2.csv", 
        save_file="writeup_files/Extender_Kalman_Filter-Dataset_2_graph.png"
        )

# =============================================================================