import mujoco as mj
import numpy as np
from mujoco import viewer
import matplotlib.pyplot as plt
import sys
import xlsxwriter

xml_path = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/CuttleBot With Skin/8Tendons_coupling.xml'

times = []
simend = 10  # simulation time
vels = []
peakX = 0
peakY = 0
peakZ = 0
pos = []
angvels = []
rots = []


def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x-axis, pitch is rotation around y-axis and yaw is rotation around z-axis.
    :param w: scalar component of the quaternion
    :param x: x component of the vector part of the quaternion
    :param y: y component of the vector part of the quaternion
    :param z: z component of the vector part of the quaternion
    :return: roll (phi), pitch (theta), and yaw (psi) in degrees
    """
    # Calculate the roll (phi)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Calculate the pitch (theta)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Calculate the yaw (psi)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # Convert from radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw


def vertical(freq, amp, dutyCycle):
    period = 1 / freq
    cycleThreshold = period * dutyCycle
    cycleTime = data.time % period

    if cycleTime < cycleThreshold:
        for i in range(0, 8):
            data.ctrl[i] = -amp
    else:
        for i in range(0, 8):
            data.ctrl[i] = 0


def barrel(freq, amp, phaseShift):
    w = 2 * np.pi * freq
    # Set the controls for each actuator
    for i in range(8):
            if i % 2 == 0:
                if np.sign(np.sin(w * data.time)) == 1:
                    data.ctrl[i] = -amp  # even fins
                else:
                    data.ctrl[i] = 0
            else:
                if np.sign(np.sin(w * data.time + phaseShift)) == 1:
                    data.ctrl[i] = -amp  # even fins
                else:
                    data.ctrl[i] = 0


def lateral(freq, amp, phaseShift):
    w = 2 * np.pi * freq
    # Set the controls for each actuator
    for i in range(8):
        if i % 2 == 0:
            data.ctrl[i] = amp * np.sin(w * data.time)  # even fins
        else:
            data.ctrl[i] = amp * np.sin(w * data.time + phaseShift)


def linear(freq, wavelength, amp):
    # Set the amplitude for each pair of actuators
    w = 2 * np.pi * freq
    phaseShiftAtOneWL = 2 * np.pi / 3
    phaseShift = phaseShiftAtOneWL / (wavelength / 105)

    # Set the controls for each actuator
    for i in range(4):
        phase = phaseShift * i
        data.ctrl[2 * i] = amp * np.sin(w * data.time + phase)
        data.ctrl[2 * i + 1] = amp * np.sin(w * data.time + phase)


def rotation(freq, wavelength, amp, sideShift):
    w = 2 * np.pi * freq
    phaseShiftAtOneWL = 2 * np.pi / 3
    phaseShift = phaseShiftAtOneWL / (wavelength / 105)

    # Set the controls for each actuator
    for i in range(4):
        phase = phaseShift * i
        data.ctrl[2 * i] = amp * np.sin(w * data.time + phase)
        data.ctrl[2 * i + 1] = amp * np.sin(w * data.time + phase + sideShift)


def controller(model, data):
    # put the controller here. This function is called inside the simulation.

    # vertical(0.3, 1.6, 0.8)
    linear(1, 157.5, 1.6)
    # rotation(0.5, 52.5*12, 1.6, -np.pi/16)
    # lateral(0.5, 1.6, -45)
    # barrel(0.5, 1.6, np.pi / 3)


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data

# set the controller
mj.set_mjcb_control(controller)

# # Just for visualisation
viewer.launch(model, data)
while data.time <= simend:
    mj.mj_step(model, data)

# For extracting data
while data.time <= simend:
    mj.mj_step(model, data)
    elapsed_time = data.time
    time_remaining_message = "Time elapsed: {:.2f} seconds".format(elapsed_time)

    sys.stdout.write('\r' + time_remaining_message)
    sys.stdout.flush()

    currentvel = data.sensor('global_vel').data.copy()
    currentpos = data.sensor('c_m').data.copy()
    currentang_vel = data.sensor('global_angvel').data.copy()
    currentquat = data.sensor('quat').data.copy()

    roll, pitch, yaw = quaternion_to_euler(*currentquat)

    xvel = currentvel[0]
    if abs(xvel) > peakX:
        peakX = xvel
    yvel = currentvel[1]
    if abs(yvel) > peakY:
        peakY = yvel
    zvel = currentvel[2]
    if abs(zvel) > peakZ:
        peakZ = zvel

    xpos = currentpos[0]
    ypos = currentpos[1]
    zpos = currentpos[2]

    roll_vel = np.degrees(currentang_vel[0])
    pitch_vel = np.degrees(currentang_vel[1])
    yaw_vel = np.degrees(currentang_vel[2])

    angvels.append([roll_vel, pitch_vel, yaw_vel])
    pos.append([xpos, ypos, zpos])
    vels.append([xvel, yvel, zvel])
    rots.append([roll, pitch, yaw])
    times.append(data.time)

mj.mj_resetData(model, data)
mj.mj_forward(model, data)

vels = np.array(vels)
pos = np.array(pos)
angvels = np.array(angvels)
rots = np.array(rots)


# Specify the output Excel file
excel_file = 'placeholder'

# Write the data to the Excel file
workbook = xlsxwriter.Workbook(excel_file)
worksheet = workbook.add_worksheet()

# Write header
worksheet.write_row(0, 0, ['Time /s', 'Roll', 'Pitch', 'Yaw'])

# Write data rows
for i, time in enumerate(times):
    # Writing time to the first column
    worksheet.write(i + 1, 0, time)

    # Extracting the second column from 'vels' 2D NumPy array and writing to the second column
    xroll = rots[i, 0]
    ypitch = rots[i, 1]
    zyaw = rots[i, 2]
    worksheet.write(i + 1, 1, xroll)
    worksheet.write(i + 1, 2, ypitch)
    worksheet.write(i + 1, 3, zyaw)
workbook.close()

# # Plot using normal scales
plt.figure()
plt.plot(times, vels[:, 0], linestyle='-', label='X Velocity')
plt.plot(times, vels[:, 1], linestyle='-', label='Y Velocity')
plt.plot(times, vels[:, 2], linestyle='-', label='Z Velocity')

# Set labels and title
plt.xlabel('Time (s)')
plt.ylabel('Velocity (mm/s)')
plt.title('Velocity vs Time')
plt.legend()
# Show the plot
plt.show()

plt.figure()
plt.plot(times, pos[:, 0], linestyle='-', label='Y Position')
plt.plot(times, pos[:, 1], linestyle='-', label='Y Position')
plt.plot(times, pos[:, 2], linestyle='-', label='Z Position')

# Set labels and title
plt.xlabel('Time (s)')
plt.ylabel('Position (mm)')
plt.title('Position vs Time')
plt.legend()
# Show the plot
plt.show()

plt.figure()
plt.plot(times, angvels[:, 0], linestyle='-', label='Roll')
plt.plot(times, angvels[:, 1], linestyle='-', label='Pitch')
plt.plot(times, angvels[:, 2], linestyle='-', label='Yaw')

# Set labels and title
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity vs Time')
plt.legend()
# Show the plot
plt.show()

# # Plot using normal scales
plt.figure()
# plt.plot(times, rots[:, 0], linestyle='-', label='Roll')
plt.plot(times, rots[:, 1], linestyle='-', label='Pitch')
# plt.plot(times, rots[:, 2], linestyle='-', label='Yaw')

# Set labels and title
plt.xlabel('Time (s)')
plt.ylabel('Rotation (degrees)')
plt.title('Orientation vs Time')
plt.legend()
# Show the plot
plt.show()

# Just for visualisation
viewer.launch(model, data)
while data.time <= 1:
    mj.mj_step(model, data)