import mujoco as mj
import numpy as np
from mujoco import viewer
import os
import matplotlib.pyplot as plt
import xlsxwriter
import time
import sys

xml_path = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/CuttleBot With Skin/8Tendons_coupling.xml'

times = []
w = 0
current_cycle = 0
freq_range = np.arange(0.25, 2.1, 0.25)
# freq_range = [1.25, 1.5]
numerator_range = np.arange(52.5, 472.5+(52.5*3), 52.5)
# numerator_range = np.arange(52.5*7, 472.5, 52.5)
wavelength_range = numerator_range / 105
freq = 0
wavelength = 0

test_duration = 20
simend = test_duration * len(wavelength_range) * len(freq_range) # simulation time
print(simend)
vels = []
peakX = 0
peakY = 0
peakZ = 0
peak_vels = []
avg_vels = []

start_time = time.time()

amp_ends = -1.6
amp_mids = -1.6

def finsToRest():
    for i in range(0, 40):
        data.ctrl[i] = 0

def finsUp():
    for i in range(0, 40):
        data.ctrl[i] = 0.4

def finsDown():
    for i in range(0, 40):
        data.ctrl[i] = -0.4

def hover():
    for i in range(0, 40):
        data.ctrl[i] = 0.4*np.sin(w * data.time)

def goUp():
    for i in range(0, 40):
        if np.sin(2*np.pi*freq*data.time) >= 0:
            data.ctrl[i] = 0.4*np.sin(w * data.time)
        else:
            data.ctrl[i] = 0

def UpDC(freq, amp, dutyCycle):
    period = 1 / freq
    cycleThreshold = period * dutyCycle
    cycleTime = data.time % period

    if cycleTime < cycleThreshold:
        for i in range(0, 40):
            data.ctrl[i] = amp
    else:
        for i in range(0, 40):
            data.ctrl[i] = 0

def goDown():
    for i in range(0, 40):
        if np.sin(2*np.pi*freq*data.time) <= 0:
            data.ctrl[i] = 0.4*np.sin(w * data.time)
        else:
            data.ctrl[i] = 0

def goForward(phaseShift):
    for i in range(0, 10):
        data.ctrl[i] = 0.8 * np.sin(w * data.time + phaseShift * 0)
    for i in range(10, 20):
        data.ctrl[i] = 0.625 * np.sin(w * data.time + phaseShift * 1)
    for i in range(20, 30):
        data.ctrl[i] = 0.625 * np.sin(w * data.time + phaseShift * 2)
    for i in range(30, 40):
        data.ctrl[i] = 0.8 * np.sin(w * data.time + phaseShift * 3)

def goBackward(phaseShift):
    for i in range(0, 10):
        data.ctrl[i] = 0.4 * np.sin(w * data.time - phaseShift * 0)
    for i in range(10, 20):
        data.ctrl[i] = 0.4 * np.sin(w * data.time - phaseShift * 1)
    for i in range(20, 30):
        data.ctrl[i] = 0.4 * np.sin(w * data.time - phaseShift * 2)
    for i in range(30, 40):
        data.ctrl[i] = 0.4 * np.sin(w * data.time - phaseShift * 3)

def goForward_tendon(phaseShift):
    # Set the amplitude for each pair of actuators
    amp_pairs = [amp_ends, amp_mids, amp_mids, amp_ends]

    # Set the controls for each actuator
    for i in range(4):
        phase = phaseShift * i
        data.ctrl[2 * i] = amp_pairs[i] * np.sin(w * data.time + phase)
        data.ctrl[2 * i + 1] = amp_pairs[i] * np.sin(w * data.time + phase)

def UpDC_tendon(freq, amp, dutyCycle):
    period = 1 / freq
    cycleThreshold = period * dutyCycle
    cycleTime = data.time % period

    if cycleTime < cycleThreshold:
        for i in range(0, 8):
            data.ctrl[i] = amp
    else:
        for i in range(0, 8):
            data.ctrl[i] = 0

def rotateCW(phaseShift):
    # left side fins
    for i in range(0, 5):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 0)

    for i in range(10, 15):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 1)

    for i in range(20, 25):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 2)

    for i in range(30, 35):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 3)

    # right side fins

    for i in range(5, 10):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 2)

    for i in range(15, 20):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 3)

    for i in range(25, 30):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 0)

    for i in range(35, 40):
        data.ctrl[i] = 0.5 * np.sin(w * data.time - phaseShift * 1)


def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass


def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    global times, w, freq, wavelength, i_wavelength

    i_freq = (current_cycle // len(wavelength_range)) % len(freq_range)
    i_wavelength = current_cycle % len(wavelength_range)
    freq = freq_range[i_freq]
    wavelength = wavelength_range[i_wavelength]
    phaseShiftAtOneWL = 2 * np.pi / 3
    phaseShift = -phaseShiftAtOneWL / wavelength
    w = 2 * np.pi * freq

    # goForward(phaseShift)
    goForward_tendon(phaseShift)



# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

# viewer.launch(model, data)
# while data.time <= 1:
#     mj.mj_step(model, data)

# For extracting data
while data.time <= simend:
    mj.mj_step(model, data)

    elapsed_time = time.time() - start_time
    time_remaining_message = "Time elapsed: {:.2f} seconds".format(elapsed_time)

    sys.stdout.write('\r' + time_remaining_message + "| Sim Time: {:.2f}".format(data.time) + '| Cycle: ' + str(current_cycle))
    sys.stdout.flush()

    currentvel = data.sensor('global_vel').data.copy()

    xvel = currentvel[0]
    yvel = currentvel[1]
    zvel = currentvel[2]

    if abs(xvel) > peakX:
        peakX = xvel
    if abs(yvel) > peakY:
        peakY = yvel
    if abs(zvel) > peakZ:
        peakZ = zvel

    vels.append([xvel, yvel, zvel])
    times.append(data.time)

    if data.time >= test_duration:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

        current_cycle += 1
        vels = np.array(vels)
        thisavg = np.mean(vels, axis=0)
        avg_vels.append(thisavg)
        vels = []
        peak_vels.append([peakX, peakY, peakZ])
        print('\n' + "Peak X: {:.2f}".format(peakX) + "| Peak Y: {:.2f}".format(peakY) + "| Peak Z: {:.2f}".format(peakZ))
        print("Average X: {:.2f}".format(thisavg[0]) + "| Average Y: {:.2f}".format(thisavg[1]) + "| Average Z: {:.2f}".format(thisavg[2]))
        print("Freq: " + str(freq) + " Wavelength: " + str(numerator_range[i_wavelength]) + '\n—————————————————————————————————\n')
        peakX = 0
        peakY = 0
        peakZ = 0
    if current_cycle >= len(wavelength_range) * len(freq_range):
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        break

# Specify the output Excel file
excel_file = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/Results/Excel/8_velocity_up_down.xlsx'

avg_vels = np.array(avg_vels)
peak_vels = np.array(peak_vels)
# Convert peak_vels to a 2D array
X = peak_vels[:, 0]
Y = peak_vels[:, 1]
Z = peak_vels[:, 2]
X = X.reshape(len(freq_range), len(numerator_range))
Y = Y.reshape(len(freq_range), len(numerator_range))
Z = Z.reshape(len(freq_range), len(numerator_range))
print(X)
print(Y)
print(Z)

# Convert avg_vels to a 2D array
Xav = avg_vels[:, 0]
Yav = avg_vels[:, 1]
Zav = avg_vels[:, 2]
Xav = Xav.reshape(len(freq_range), len(numerator_range))
Yav = Yav.reshape(len(freq_range), len(numerator_range))
Zav = Zav.reshape(len(freq_range), len(numerator_range))
print(Xav)
print(Yav)
print(Zav)

# Write the data to the Excel file
workbook = xlsxwriter.Workbook(excel_file)

worksheet1 = workbook.add_worksheet()

# Write header
worksheet1.write_row(0, 0, ['Average X Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(Xav):
    worksheet1.write_row(row_num + 1, 0, row_data)

worksheet2 = workbook.add_worksheet()

# Write header
worksheet2.write_row(0, 0, ['Average Y Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(Yav):
    worksheet2.write_row(row_num + 1, 0, row_data)

worksheet3 = workbook.add_worksheet()

# Write header
worksheet3.write_row(0, 0, ['Average Z Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(Zav):
    worksheet3.write_row(row_num + 1, 0, row_data)

# Peak Values

worksheet1 = workbook.add_worksheet()

# Write header
worksheet1.write_row(0, 0, ['Peak X Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(X):
    worksheet1.write_row(row_num + 1, 0, row_data)

worksheet2 = workbook.add_worksheet()

# Write header
worksheet2.write_row(0, 0, ['Peak Y Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(Y):
    worksheet2.write_row(row_num + 1, 0, row_data)

worksheet3 = workbook.add_worksheet()

# Write header
worksheet3.write_row(0, 0, ['Peak Z Velocity /mm/s'])

# Write data rows
for row_num, row_data in enumerate(Z):
    worksheet3.write_row(row_num + 1, 0, row_data)



workbook.close()

# Create a 2D grid for heatmap
Yaxis, Xaxis = np.meshgrid(freq_range, numerator_range)

# Calculate the offsets
numerator_offset = (numerator_range[1] - numerator_range[0]) / 2.0
freq_offset = (freq_range[1] - freq_range[0]) / 2.0

# Adjust the tick locations
numerator_tick_locations = [w + numerator_offset for w in numerator_range]
freq_tick_locations = [f + freq_offset for f in freq_range]

# Update the plotting code with these new tick locations
extent = [
    min(numerator_range) - numerator_offset,
    max(numerator_range) + numerator_offset,
    max(freq_range) + freq_offset,
    min(freq_range) - freq_offset
]

# Plot heatmap of peak
plt.figure(figsize=(8, 6))
plt.imshow(Y, cmap='viridis', aspect='auto', extent=extent)
plt.colorbar(label='Peak Velocity (mm/s)')
plt.ylabel('Frequency (Hz)')
plt.xlabel('Wavelength (mm)')
plt.title('Peak Velocity Heatmap')
plt.yticks(freq_range, freq_range)
plt.xticks(numerator_range, numerator_range)
# plt.gca().invert_yaxis()
plt.show()

# Plot heatmap of avg
plt.figure(figsize=(8, 6))
plt.imshow(Yav, cmap='viridis', aspect='auto', extent=extent)
plt.colorbar(label='Average Velocity (mm/s)')
plt.ylabel('Frequency (Hz)')
plt.xlabel('Wavelength (mm)')
plt.title('Average Velocity Heatmap')
plt.yticks(freq_range, freq_range)
plt.xticks(numerator_range, numerator_range)
# plt.gca().invert_yaxis()
plt.show()

# Just for visualisation

