import numpy as np
from numpy.random import randn
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import matplotlib.pylab as plt
import serial
import time
############################### Main Var #################################
# serial conction from arduino
ser = serial.Serial('COM9', 9600)  # open serial port
ser.timeout = 1                   #
print(ser.name)                   # show com connected
# Data arr to graph
Noise_data_arr, Kalman_data_arr_Dis, Kalman_data_arr_velocity, Real_data_arr = [], [], [],[]
Noise_data = 0
real_date = 0
Acc_error = 0
########################## Screne varible ###########################
Noise_scale = 0.3
line_width_noise_scale = 0.3
line_width_kalman_scale = 0.3
line_width_real_scale = 0.3
########################## Tuning varible ##############################
R_std = 0.35
Q_std = 0.04
############################## TKinter INIT #############################
# import tkinter and chose size and title
from tkinter import *
Kalman_window = Tk()
Kalman_window.title("Kalman Simulator")
Kalman_window.geometry('400x300')
############################ Frist order Kalman Filter #############################
def FirstOrderKF(R, Q, dt):
    """ Create first order Kalman filter.
    Specify R and Q as floats."""
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.zeros(2)
    kf.P *= np.array([[100, 0], [0, 1]])
    kf.R *= R
    kf.Q = Q_discrete_white_noise(2, dt, Q)
    kf.F = np.array([[1., dt], [0., 1]])
    kf.H = np.array([[1., 0]])
    return kf
filter = FirstOrderKF(R_std, Q_std, 1.0)
############################# Take Data Button ###################################
# Samples from Kalman & Real & Noise function
def Take_Samples_Button():
    global Acc_error, real_date
    global Noise_scale, Noise_data
    global line_width_real_scale
    global line_width_noise_scale
    global line_width_kalman_scale
    global filter
    for i in range(1000):
        # Add a noise data to system
        Acc_error = Acc_error + randn() * Noise_scale
        # Cul noise & real data
        try:
            Noise_data = float(str(ser.readline().decode())) + Acc_error
            real_date = float(str(ser.readline().decode()))
        except:
            pass
        #Add noise & real data
        Noise_data_arr.append(Noise_data)
        Real_data_arr.append(real_date)

        filter.predict()
        filter.update( (Noise_data + filter.x[0]) / 2.0 )
        #Take Dis Data from Kalman
        filter_data_Dis = getdouble(filter.x[0])
        Kalman_data_arr_Dis.append(filter_data_Dis)
        #print(float(str(ser.readline().decode())))
        #Take Velocity Data from Kalman
        # filter_data_velocity = getdouble(filter.x[1])
        # Kalman_data_arr_velocity.append(filter_data_velocity)
        # Print noise & real & kalman data graph
        # plt.plot(Kalman_data_arr, color='green')
        plt.plot(Noise_data_arr, linewidth=line_width_noise_scale,  color='black')
        plt.plot(Real_data_arr, linewidth=line_width_real_scale, color='red')
        plt.plot(Kalman_data_arr_Dis, linewidth=line_width_kalman_scale, color='blue')
        #plt.plot(Kalman_data_arr_velocity, linewidth=line_width_kalman_scale, color='blue')
        plt.legend(["Sensor Data", "Real Data", "Kalman filter"], loc="upper right")
        #time.sleep(0.05)
#Starting read samples button
Samples_Button =Button(Kalman_window,text="Take Samples",command=Take_Samples_Button).place(x=80,y=200)
################################ Plot Data ########################################
# Show out function from matplotlib
def OUTPUT_Button():
    #Graph Title
    plt.title("Kalman Filter")
    # X-axis label & Y-axis label
    plt.xlabel("samples")
    plt.ylabel("Distance")
    # View Plot of Kalman & Real & Noise Data
    plt.show()
################################# OUTPUT #############################################
# Show out Button
Output_Button=Button(Kalman_window,text="Result",bd='0',command=OUTPUT_Button).place(x=300,y=200)
################################ Screan lables ##########################################
# Noise Scale GUI
noise_scale = Label(Kalman_window,text='Noise:').place(x=10,y=10)
noise_scale_RB_value = IntVar()
noise_scale_RB1 = Radiobutton(Kalman_window,text='0.1',variable=noise_scale_RB_value,value=0.1).place(x=10,y=30)
noise_scale_RB2 = Radiobutton(Kalman_window,text='0.01',variable=noise_scale_RB_value,value=0.01).place(x=10,y=50)
noise_scale_RB3 = Radiobutton(Kalman_window,text='0.001',variable=noise_scale_RB_value,value=0.001).place(x=10,y=70)
noise_scale_RB4 = Radiobutton(Kalman_window,text='0.0001',variable=noise_scale_RB_value,value=0.0001).place(x=10,y=90)
# line width real scale
line_width_real_scale = Label(Kalman_window,text='Real scale:').place(x=70,y=10)
line_width_real_scale_RB_value = IntVar()
line_width_real_scale_RB1 = Radiobutton(Kalman_window,text='0.2',variable=line_width_real_scale_RB_value,value=0.2).place(x=70,y=30)
line_width_real_scale_RB2 = Radiobutton(Kalman_window,text='0.3',variable=line_width_real_scale_RB_value,value=0.3).place(x=70,y=50)
line_width_real_scale_RB3 = Radiobutton(Kalman_window,text='0.4',variable=line_width_real_scale_RB_value,value=0.4).place(x=70,y=70)
line_width_real_scale_RB4 = Radiobutton(Kalman_window,text='0.5',variable=line_width_real_scale_RB_value,value=0.5).place(x=70,y=90)
# line width noise scale
line_width_noise_scale = Label(Kalman_window,text='Noise scale:').place(x=130,y=10)
line_width_noise_scale_RB_value = IntVar()
line_width_noise_scale_RB1 = Radiobutton(Kalman_window,text='0.2',variable=line_width_noise_scale_RB_value,value=0.2).place(x=130,y=30)
line_width_noise_scale_RB2 = Radiobutton(Kalman_window,text='0.3',variable=line_width_noise_scale_RB_value,value=0.3).place(x=130,y=50)
line_width_noise_scale_RB3 = Radiobutton(Kalman_window,text='0.4',variable=line_width_noise_scale_RB_value,value=0.4).place(x=130,y=70)
line_width_noise_scale_RB4 = Radiobutton(Kalman_window,text='0.5',variable=line_width_noise_scale_RB_value,value=0.5).place(x=130,y=90)
# line width kalman scale
line_width_kalman_scale = Label(Kalman_window,text='kalman scale:').place(x=200,y=10)
line_width_kalman_scale_RB_value = IntVar()
line_width_kalman_scale_RB1 = Radiobutton(Kalman_window,text='0.2',variable=line_width_kalman_scale_RB_value,value=0.2).place(x=200,y=30)
line_width_kalman_scale_RB2 = Radiobutton(Kalman_window,text='0.3',variable=line_width_kalman_scale_RB_value,value=0.3).place(x=200,y=50)
line_width_kalman_scale_RB3 = Radiobutton(Kalman_window,text='0.4',variable=line_width_kalman_scale_RB_value,value=0.4).place(x=200,y=70)
line_width_kalman_scale_RB4 = Radiobutton(Kalman_window,text='0.5',variable=line_width_kalman_scale_RB_value,value=0.5).place(x=200,y=90)
#
R_std_lable = Label(Kalman_window,text='R_std:').place(x=10,y=150)
R_std_value = StringVar()
R_std_Entry = Entry(Kalman_window, textvariable=R_std_value).place(x=10,y=170)
#R_std_Entry.focus()
#
Q_std_lable = Label(Kalman_window,text='Q_std:').place(x=140,y=150)
Q_std_value = StringVar()
Q_std_Entry = Entry(Kalman_window,  textvariable=Q_std_value).place(x=140,y=170)
#Q_std_Entry.focus()
#
def Tune_value():
    global Q_std, Q_std_value
    global R_std, R_std_value
    if float(Q_std_value.get()) > -5:
        if float(Q_std_value.get()) < 5:
            Q_std = float(Q_std_value.get())
        else:
            Q_std = 1
    else:
        Q_std = 0.01
    if float(R_std_value.get()) > -5:
        if float(R_std_value.get()) < 5:
            R_std = float(R_std_value.get())
        else:
            R_std = 1
    else:
        R_std = 0.01
Tune_value_button = Button(Kalman_window,text="Tune",command=Tune_value).place(x=280,y=160)

#CHANGE Values Button
def Change_value():
    global noise_scale_RB_value, Noise_scale
    Noise_scale = noise_scale_RB_value.get()
    global line_width_real_scale, line_width_real_scale_RB_value
    global line_width_noise_scale, line_width_noise_scale_RB_value
    global line_width_kalman_scale, line_width_kalman_scale_RB_value
    line_width_real_scale = line_width_real_scale_RB_value.get()
    line_width_noise_scale = line_width_noise_scale_RB_value.get()
    line_width_kalman_scale = line_width_kalman_scale_RB_value.get()
Change_value_button=Button(Kalman_window,text="Change",command=Change_value).place(x=150,y=120)
################################## EXIT ###########################################
#Exit button work to destory the main window App
Exit_Button = Button(Kalman_window, text="Exit", bd='0', fg="black", command=Kalman_window.destroy)
#Make button in Bottom of window
Exit_Button.pack(side=BOTTOM)
#Loop GUI in tkinter
Kalman_window.mainloop()
#################################################