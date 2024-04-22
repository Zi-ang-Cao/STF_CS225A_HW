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

    Y1_values, Y2_values, Y3_values = [], [], []

    # # Process each group of four lines after the header
    # Title_values = lines[0].strip().split()
    # if joint_id == 3:
    #     intersted_range = Title_values[2:5]
    # elif joint_id == 2:
    #     intersted_range = Title_values[1:4]
    # else:
    #     raise ValueError("Invalid joint_id")

    # intersted_range = " ".join(intersted_range)
    subTitle_values = " "
    if question_idx in [1, 2, 3, 4]:
        title_values = f"Control Method from HW1 Question {question_idx} -- Critical damping on joint_1"
    elif question_idx == 5:
        title_values = f"Control Method from HW1 Question 4 + 2.5kg additional payload -- Critical damping on joint_1"
    else:
        raise ValueError("Invalid question_idx")
    
    i = 1  # Start after the header

    group_size = 1
    while i <= (len(lines) - group_size):
        line_content = lines[i].strip().split()
        # Extract X_values
        X_values.append(float(line_content[0]))
        # Extract Y_values
        Y1_values.append(float(line_content[1]) * 180 / np.pi)    # joint 1
        Y2_values.append(float(line_content[3]) * 180 / np.pi)    # joint 3
        Y3_values.append(float(line_content[4]) * 180 / np.pi)    # joint 4

        i += group_size

    return X_values, Y1_values, Y2_values, Y3_values, subTitle_values, title_values
    

# Function to plot the data
def plot_data(q_values, Y1_values, Y2_values, Y3_values, subTitle_values, title_values, output_file_path='que_1.png'):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    X_label = 'control time'
    X_unit = 's'

    Y_label_list = ['joint_1 angle', 'joint_3 angle', 'joint_4 angle']
    # Y_unit = 'radians'
    Y_unit = 'degrees'


    for i, Y_values in enumerate([Y1_values, Y2_values, Y3_values]):
        # Plotting M11 vs q_variable
        axs[i].plot(q_values, Y_values, marker='o', linestyle='-', color='purple')
        Y_label = Y_label_list[i]
        
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
os.chdir('/Users/zi-angcao/03_ResearchRepo/SAI2/OpenSai/STF_CS225A_HW/homework/hw1')

for question_idx in [1, 2, 3, 4, 5]:
    file_path=f'./data_files/que_{question_idx}.txt'
    output_file_path=f'./img_out/que_{question_idx}.png'
    X_values, Y1_values, Y2_values, Y3_values, subTitle_values, title_values = read_data(file_path, question_idx)
    plot_data(X_values, Y1_values, Y2_values, Y3_values, subTitle_values, title_values, output_file_path)