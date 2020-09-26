import numpy as np
import matplotlib.pyplot as plt

plt.figure(0)
fig, axs = plt.subplots(2)
lab_pre = ['pre-x', 'pre-y', 'pre-z']
lab_out = ['out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
a_pre=np.loadtxt('mat_pre_suc0.txt')
a_out=np.loadtxt('mat_out_suc0.txt')
time=a_pre[:,0]
T_pre = a_pre[:, plot_ind]
T_out = a_out[:, plot_ind]
axs[0].set_title('succed')
for i in range(3):
    if i==1:
        axs[0].plot(time, T_pre[:,i],'.-', label=lab_pre[i])
        axs[0].plot(time, T_out[:,i],'.-', label=lab_out[i])
axs[0].grid()
axs[0].legend()

a_pre=np.loadtxt('mat_pre_fail0.txt')
a_out=np.loadtxt('mat_out_fail0.txt')
time=a_pre[:,0]
T_pre = a_pre[:, plot_ind]
T_out = a_out[:, plot_ind]
axs[1].set_title('Failed')
for i in range(3):
    if i==1:
        axs[1].plot(time, T_pre[:,i],'.-', label=lab_pre[i])
        axs[1].plot(time, T_out[:,i],'.-', label=lab_out[i])
axs[1].grid()
axs[1].legend()

## Draw IMU data
plt.figure(1)
fig, axs = plt.subplots(2)
imu=np.loadtxt('imu.txt')
time=imu[:,0]
axs[0].set_title('succed')
lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
lab_2 = ['acc-x', 'acc-y', 'acc-z']
for i in range(1,4):
    # if i==1:
    axs[0].plot(time, imu[:,i],'.-', label=lab_1[i])
    axs[1].plot(time, imu[:,i+3],'.-', label=lab_2[i])
axs[0].grid()
axs[0].legend()
axs[1].grid()
axs[1].legend()
plt.show()