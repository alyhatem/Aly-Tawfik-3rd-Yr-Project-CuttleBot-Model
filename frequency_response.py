import mujoco as mj
import numpy as np
from mujoco import viewer
import xlsxwriter
import matplotlib.pyplot as plt
import mediapy as media

xml_path = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/CuttleBot With Skin/tailTendon_Membrane.xml'

# freq_range = np.logspace(np.log10(0.5), np.log10(11), 20)
freq_range = np.arange(0.5, 11.25, 0.25)
# freq_range = [0.5, 1]
test_duration = 4.1
simend = test_duration * len(freq_range)
current_cycle = 0
highestZ = float('-inf')
lowestZ = float('inf')
amplitudes = []
amp_ends = -1.6
amp_mids = -2.5

def sinTest(freq, amp):
    for i in range(0, 5):
        w = 2 * np.pi * freq
        data.ctrl[i] = amp * np.sin(w * data.time)

def sinTestTendon(t, freq, amp):
    w = 2 * np.pi * freq
    data.ctrl[t] = amp * np.sin(w * data.time)

def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):  # 25.8285 is half amp 51.57 is full amp
    global lowestZ, highestZ
    # put the controller here. This function is called inside the simulation.
    i_freq = current_cycle % len(freq_range)
    # sinTest(freq_range[i_freq], 2.5)
    if data.time > 2:
        sinTestTendon(0, freq_range[i_freq], -6)
        currentpos = data.sensor('amplitude1').data.copy()
        if currentpos[2] < lowestZ:
            lowestZ = currentpos[2]

        if currentpos[2] > highestZ:
            highestZ = currentpos[2]


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

# viewer.launch(model, data)
# while data.time <= simend:
#     mj.mj_step(model, data)
#
#     if data.time >= test_duration:
#         mj.mj_resetData(model, data)
#         mj.mj_forward(model, data)
#         current_cycle += 1
#     if current_cycle >= len(freq_range):
#         mj.mj_resetData(model, data)
#         mj.mj_forward(model, data)
#         break

# For extracting data
while data.time <= simend:
    mj.mj_step(model, data)

    if data.time >= test_duration:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        current_cycle += 1
        # Calculate amplitude and store in the list
        amplitude = highestZ - lowestZ
        amplitudes.append(amplitude)

        # Reset highestZ and lowestZ for the new frequency
        highestZ = float('-inf')
        lowestZ = float('inf')

    if current_cycle >= len(freq_range):
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        break



# Specify the output Excel file
excel_file = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/Results/Excel/Fin Frequency Response/freq_grav_membrane.xlsx'

amplitudes = np.array(amplitudes) / amplitudes[0]

# Combine arrays into a list of rows
plot = list(zip(freq_range, amplitudes))

# Write the data to the Excel file
workbook = xlsxwriter.Workbook(excel_file)
worksheet = workbook.add_worksheet()

# Write header
worksheet.write_row(0, 0, ['Frequency /Hz', 'Normalised Amplitude /mm'])

# Write data rows
for row_num, row_data in enumerate(plot):
    worksheet.write_row(row_num + 1, 0, row_data)

workbook.close()

# # Plot using normal scales
plt.semilogx(freq_range, amplitudes, marker='o', linestyle='-')
# Set labels and title
plt.xlabel('Frequency (Hz)')
plt.ylabel('Normalised Amplitude')
plt.title('Frequency Response')
# Show the plot
plt.show()



# Just for visualisation



