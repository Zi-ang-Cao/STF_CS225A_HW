"""
python plot_data.py

Author: @Zi-ang-Cao
"""

import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

def read_data(file_path='./data_files/que_1.txt', question_idx=1):
    # Open and read the lines of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Initialize lists to store the data
    X_values = []

    Y_dim = len(lines[0].strip().split())-1

    Y_values_list = [ [] for i in range(Y_dim)]
    subTitle_values = [ " " for i in range(Y_dim)]

    title_values = " "
    if question_idx in [2]:
        title_values = f"Operational Space Controllers"
    elif question_idx in [3]:
        title_values = f"Control with operational space gravity compensation"
    elif question_idx in [4]:
        title_values = f"Control with operational space gravity compensation + Trajectory Tracking"
    else:
        raise ValueError("Invalid question_idx")
    
    i = 0  # Start after the header
    group_size = 1
    while i <= (len(lines) - group_size):
        line_content = lines[i].strip().split()
        # Extract X_values
        X_values.append(float(line_content[0]))
        # Extract Y_values
        for idx in range(Y_dim):
            if idx < 3:
                # In task space
                Y_values_list[idx].append(float(line_content[idx+1]))
            else:
                # In joint space
                Y_values_list[idx].append(float(line_content[idx+1]) * 180 / np.pi)

        i += group_size

    return X_values, Y_values_list, subTitle_values, title_values
    

# Function to plot the data
def plot_data(q_values, Y_values_list, subTitle_values, title_values, output_file_path='que_1.png'):
    X_label = 'control time'
    X_unit = 's'

    Y_dim = len(Y_values_list)
    axis = ['x', 'y', 'z']
    
    fig, axs = plt.subplots(Y_dim, 1, figsize=(10 , 8 * Y_dim /3))  # Create a figure with 3 subplots
    for i, Y_values in enumerate(Y_values_list):
        # Plotting M11 vs q_variable
        axs[i].plot(q_values, Y_values, marker='o', linestyle='-', color='purple')
        Y_label = f'end effector {axis[i]}' if i < 3 else f'joint_{i-2} angle'
        Y_unit = 'm' if i < 3 else 'radians'
        axs[i].set_title(f'{Y_label} vs {X_label}')
        axs[i].set_xlabel(f'{X_label} ({X_unit})')
        axs[i].set_ylabel(f'{Y_label} ({Y_unit})')

    # plt.suptitle(f'Simulated {mode} vs {X_label} ({intersted_range})')  # Set figure title
    plt.suptitle(f'{title_values}')  # Set figure title
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust subplots to fit the title

    parent_folder = os.path.dirname(output_file_path)
    os.makedirs(parent_folder, exist_ok=True)
    plt.savefig(output_file_path)  # Save the plot as an image



# Change directory to the desired path
os.chdir('/Users/zi-angcao/03_ResearchRepo/SAI2/OpenSai/STF_CS225A_HW/homework/hw2')

# for question_idx in [1, 2, 3, 4, 5]:
# for question_idx in [2]:
for question_idx in [3, 4]:
    file_path=f'./data_files/que_{question_idx}.txt'
    output_file_path=f'./img_out/que_{question_idx}.png'
    X_values, Y_values_list, subTitle_values, title_values = read_data(file_path, question_idx)
    plot_data(X_values, Y_values_list, subTitle_values, title_values, output_file_path)