import mujoco as mj
import numpy as np
from mujoco import viewer
import os
import matplotlib.pyplot as plt
import csv
import xlsxwriter
import matplotlib.image as mpimg
from matplotlib.animation import FuncAnimation

# get the full path
xml_path = "/Users/alyhatem/Desktop/Project 127/Mujoco Python/CuttleBot With Skin/tailTendon_Membrane.xml"
# dirname = os.path.dirname(__file__)
# abspath = os.path.join(dirname + "/" + xml_path)
# xml_path = abspath

simend = 2.5  # simulation time
z_val = []
z_val3 = []
times = []
amp_ends = 0.8
amp_mids = 0.625


def tendonPull(t):
    data.ctrl[t] = -6


def tendonPush(t):
    data.ctrl[t] = 6


def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    # for i in range(10, 15):
    #     data.ctrl[i] = -0.5
    # for i in range(0, 5):
    #     data.ctrl[i] = -amp_ends
    # for i in range(10, 15):
    #     data.ctrl[i] = -amp_mids
    pass


def controller(model, data):  # 25.8285 is half amp 51.57 is full amp
    # put the controller here. This function is called inside the simulation.
    global z_val, times

    ############### Testing Step Response ###############
    if data.time < 2:
        if data.time > 1:
            # for i in range(10, 15):
            #     data.ctrl[i] = amp_mids
            # for i in range(0, 5):
            #     data.ctrl[i] = amp_ends
            # tendonPull(7)
            tendonPull(0)
        else:
            # for i in range(10, 15):
            #     data.ctrl[i] = -amp_mids
            # for i in range(0, 5):
            #     data.ctrl[i] = -amp_ends
            # tendonPush(7)
            tendonPush(0)
    else:
        # for i in range(10, 15):
        #     data.ctrl[i] = -amp_mids
        # for i in range(0, 5):
        #     data.ctrl[i] = -amp_ends
        # tendonPush(7)
        tendonPush(0)


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

lowestTlen = 42.9
highestTlen = 42.9

# For extracting data
while data.time <= simend:
    mj.mj_step(model, data)


    # if data.ten_length[0] > highestTlen:
    #     highestTlen = data.ten_length[0]
    #
    # if data.ten_length[0] < lowestTlen:
    #     lowestTlen = data.ten_length[0]

    if data.time < 2:
        if data.time > 1:
            currentpos = data.sensor('amplitude1').data.copy()
            z_val.append(currentpos[2])
            # currentpos3 = data.sensor('amplitude3').data.copy()
            # z_val3.append(currentpos3[2])
            times.append(data.time)
    else:
        currentpos = data.sensor('amplitude1').data.copy()
        z_val.append(currentpos[2])
        # currentpos3 = data.sensor('amplitude3').data.copy()
        # z_val3.append(currentpos3[2])
        times.append(data.time)
mj.mj_resetData(model, data)
mj.mj_forward(model, data)

z_val = np.array(z_val) - z_val[0]
# z_val3 = np.array(z_val3) - z_val3[0]
times = np.array(times) - 1

# Specify the output Excel file
# excel_file = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/Results/Excel/step_response_tendon_membrane.xlsx'
#
# # Combine arrays into a list of rows
# plot = list(zip(times, z_val))
#
# # Write the data to the Excel file
# workbook = xlsxwriter.Workbook(excel_file)
# worksheet = workbook.add_worksheet()
#
# # Write header
# worksheet.write_row(0, 0, ['Time /s', 'Amplitude /mm'])
#
# # Write data rows
# for row_num, row_data in enumerate(plot):
#     worksheet.write_row(row_num + 1, 0, row_data)
#
# workbook.close()


# # Plot using normal scales
plt.plot(times, z_val, linestyle='-', label='fin 1')
# plt.plot(times, z_val3, linestyle='-', label='fin 3')
# Set labels and title
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Step Response')
plt.legend()
# Show the plot
plt.show()

# Just for visualisation
viewer.launch(model, data)
while data.time <= simend:
    mj.mj_step(model, data)

# # Create a figure and axis
# fig, ax = plt.subplots()
# # Initialize an empty image plot
# im = ax.imshow(frames[0])
#
# plt.axis('off')  # Hide axes
# # Function to update the image plot with each frame
# def update(frame):
#     im.set_data(frame)
#     return im,
#
# # Create the animation
# ani = FuncAnimation(fig, update, frames=frames, blit=True)
#
# # Show the animation
# plt.show()
