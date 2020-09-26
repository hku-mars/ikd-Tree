import numpy as np
import matplotlib.pyplot as plt

plt.figure(0)
fig, axs = plt.subplots(3)
lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
lab_out = ['', 'out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
a_pre=np.loadtxt('mat_pre.txt')
a_out=np.loadtxt('mat_out.txt')
time=a_pre[:,0]
axs[0].set_title('Attitude')
axs[1].set_title('Translation')
axs[2].set_title('Velocity')
for i in range(1,4):
    # if i==1:
    axs[0].plot(time, a_pre[:,i],'.-', label=lab_pre[i])
    axs[0].plot(time, a_out[:,i],'.-', label=lab_out[i])
    axs[1].plot(time, a_pre[:,i+3],'.-', label=lab_pre[i])
    axs[1].plot(time, a_out[:,i+3],'.-', label=lab_out[i])
    axs[2].plot(time, a_pre[:,i+6],'.-', label=lab_pre[i])
    axs[2].plot(time, a_out[:,i+6],'.-', label=lab_out[i])

for i in range(3):
    axs[i].set_xlim(386,389)
    axs[i].grid()
    axs[i].legend()

## Draw IMU data
plt.figure(1)
fig, axs = plt.subplots(2)
imu=np.loadtxt('imu.txt')
time=imu[:,0]
axs[0].set_title('Gyroscope')
axs[1].set_title('Accelerameter')
lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
lab_2 = ['acc-x', 'acc-y', 'acc-z']
for i in range(3):
    # if i==1:
    axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
    axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
for i in range(2):
    axs[i].set_xlim(386,389)
    axs[i].grid()
    axs[i].legend()
plt.show()