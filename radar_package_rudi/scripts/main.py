#!/usr/bin/env python

# ROS python API
import rospy
from radar_msgs.msg import *
#from scipy.fftpack import fftshift, fftfreq
import numpy as np
#import matplotlib.pyplot as plt
import frame as fr
import algo_result as alg_res
import algo_process
#import csv
#import pandas as pd

PLOT = 1
SAMPLES_PER_CHIRP = 64
CHIRPS_PER_FRAME = 32
T = 300e-6
SPEED_OF_LIGHT = 3e8
START_FREQUENCY = 24.025e9
B = 200e6
PULSE_REPETITION_INTERVAL = 500e-6
SAMPLE_PERIOD = T/SAMPLES_PER_CHIRP
SAMPLE_FREQUENCY = 1/SAMPLE_PERIOD
LAMBDA = SPEED_OF_LIGHT/START_FREQUENCY
RANGE_PAD = 256
DOPPLER_PAD = 128
ANTENNA_SPACING = 6.22e-3
PEAK_THRESHOLD = 0.1 # Minimum threshold in MTI spectrum for detection
PEAK_SLICE = 2 # extract a region of PEAK_SLICE meters of the range FFT around the target location 
PEAK_WIDTH = 1
PEAK_RELEVANCE = 0
#SEARCH_ZONE = 2
MIN_DIST = 1.5
ANGLE_CALIBRATION = -150
ANGLE_PRECISION = 1
ANTENNA_NUMBER = 2
MAX_TARGETS = 10
RANGE2BIN = np.true_divide(SAMPLES_PER_CHIRP*SPEED_OF_LIGHT,RANGE_PAD*2*B)
range_bin = np.arange(0, RANGE_PAD)*RANGE2BIN
vel_bin = np.fft.fftshift(np.fft.fftfreq(DOPPLER_PAD, PULSE_REPETITION_INTERVAL))*SPEED_OF_LIGHT/(2*START_FREQUENCY)

RECORDING_FILE_PATH = '/home/risc/catkin_ws/data/test_1.csv'
CALIBRATION_DATA_PATH = '/home/risc/catkin_ws/data/calibration_data.csv'
column_order = ['R1', 'I1', 'R2', 'I2']
'''
fig1,ax = plt.subplots(4,1)
fig2 = plt.figure()
polar_ax = fig2.add_subplot(111, polar='True')
ax[0].set_xlim(range_bin[0],range_bin[-1])
ax[0].grid()
line, = ax[0].plot([],lw=3)
fig1.ion()	
fig1.show()
fig2.show()
'''
'''
# use real-time plotting
plt.ion()

# setup each of the subplots
ax = []
fig, ax = plt.subplots(2, 1, sharex=False, sharey=False)
fig2 = plt.figure()
polar_ax = fig2.add_subplot(111, polar='True')

# set up each of the lines/curves to be plotted on their respective subplots
lines = []
line, = ax[0].plot([], lw=3)
lines.append(line)
line, = ax[0].plot([], lw=3, marker='o')
lines.append(line)
line, = ax[1].plot([], lw=3)
lines.append(line)
line, = polar_ax.plot([], marker='o')
lines.append(line)

# initial drawing of the canvas
fig.canvas.draw()

# setup variable to contain incoming serial port data
y_data = []
x_data = []
'''
'''
def update_graph(x,y,p,q,w,v,m,n):
    # update each line object
    lines[0].set_data(x, y)
    lines[1].set_data(p, q)
    lines[2].set_data(w, v)
    lines[3].set_data(m, n) 
    fig.canvas.draw()
'''
# Main class: Converts joystick commands to position setpoints
class Radar:
    # initialization method
    def __init__(self):
        #calib_data = pd.read_csv(CALIBRATION_DATA_PATH).values[:,1:]
        #calib_data = np.array((calib_data[:,0] + 1j*calib_data[:,1],calib_data[:,2] + 1j*calib_data[:,3])).T
        #self.calibration_data = np.zeros((SAMPLES_PER_CHIRP,2))
        #for i in range(len(calib_data[:,0])/SAMPLES_PER_CHIRP):
        #	k = i + 1
        #	self.calibration_data = ((k-1)*self.calibration_data + calib_data[i*SAMPLES_PER_CHIRP:(i+1)*SAMPLES_PER_CHIRP,:])/k
	self.radar = Event()
	self.algo_process_output = Radar_processing_out()

    # Callbacks

    ## Drone State callback
    def stateCb(self, msg):
        self.radar = msg

    ## Update setpoint message
    def updateSp(self):
		self.processData()

    def processData(self):
		sam_x_chirp = self.radar.dimy
		chirp_x_frame = self.radar.dimx
		real_1 = np.array(self.radar.data_rx1_re)
		imag_1 = np.array(self.radar.data_rx1_im)
		real_2 = np.array(self.radar.data_rx2_re)
		imag_2 = np.array(self.radar.data_rx2_im)
		#print(self.radar.data_rx1_re[0:64])
		print("Non zero elements:")
		print(np.count_nonzero(real_1))
		myFrame = fr.Frame(real_1,real_2,imag_1,imag_2,sam_x_chirp,chirp_x_frame)
		#myFrame.calibrate(self.calibration_data)

		#Call signal processing routine
		if myFrame.sam_x_chirp > 0:
			if(sam_x_chirp != SAMPLES_PER_CHIRP):
				print("Unexpected samples per chirp")
			if(chirp_x_frame != CHIRPS_PER_FRAME):
				print("Unexpected chirps per frame")
			data_out, target_info, MTI_out = algo_process.algo_process(myFrame,RANGE_PAD,DOPPLER_PAD,PEAK_THRESHOLD,PEAK_SLICE,PEAK_WIDTH,PEAK_RELEVANCE,MIN_DIST,ANGLE_CALIBRATION,ANGLE_PRECISION,round(LAMBDA/ANTENNA_SPACING),RANGE2BIN)
			self.algo_process_output.mti_out = MTI_out.tolist()
			self.algo_process_output.total_targets = target_info.num_targets
			print("Total targets")
			print(target_info.num_targets)
			for target in range(MAX_TARGETS):
				if target < target_info.num_targets:
					self.algo_process_output.doppler_spectrum[target*DOPPLER_PAD:(target+1)*DOPPLER_PAD] = target_info.doppler_spectrum[:,target].tolist()
					self.algo_process_output.target_position[target] = target_info.location[target]
					self.algo_process_output.target_strength[target] = target_info.strength[target]
					self.algo_process_output.target_angle[target] = target_info.angle[np.argmax(target_info.angle_spectrum[:,target])]*np.pi/180
				else:
					self.algo_process_output.doppler_spectrum[target*DOPPLER_PAD:(target+1)*DOPPLER_PAD] = np.zeros(DOPPLER_PAD).tolist()
					self.algo_process_output.target_position[target] = 0
					self.algo_process_output.target_strength[target] = 0
					self.algo_process_output.target_angle[target] = 0
			####################
			# FAKE DATA
			####################
			self.algo_process_output.total_targets = 1
			self.algo_process_output.target_position = np.zeros(MAX_TARGETS).tolist()
			self.algo_process_output.target_position[0] = 2.5
			self.algo_process_output.target_angle = np.zeros(MAX_TARGETS).tolist()
			self.algo_process_output.target_angle[0] = np.pi/4

			#df = pd.DataFrame({'R1': real_1, 'I1': imag_1, 'R2': real_1, 'I2': imag_2})
			#df[column_order].to_csv(RECORDING_FILE_PATH, mode='a', header=False)
			'''
			if(PLOT):
				if target_info.num_targets > 0:
					update_graph(range_bin,MTI_out,target_info.location[0],MTI_out[int(target_info.location[0]/RANGE2BIN)],vel_bin,target_info.doppler_spectrum[:,0],target_info.angle[np.argmax(target_info.angle_spectrum[:,0])]*np.pi/180, target_info.location[0])
				else:
					update_graph(range_bin,MTI_out,0,0,vel_bin,np.zeros(DOPPLER_PAD),0,0)
			'''
			'''
				ax[0].plot(range_bin, MTI_out)
				ax[3].plot(real_1[0:256])
				for target in range(target_info.num_targets):
					ax[1].plot(target_info.location[target], target_info.strength[target],'o')
					ax[2].plot(vel_bin, target_info.doppler_spectrum[:,target])
					polar_ax.plot(target_info.angle[np.argmax(target_info.angle_spectrum[:,target])]*np.pi/180, target_info.location[target], 'o')
			'''
			'''
				ax[0].set_xlim(left=range_bin[0],right=range_bin[-1])
				ax[0].set_ylim(bottom=0,top=0.3)
				ax[1].set_xlim(left=vel_bin[0],right=vel_bin[-1])
				ax[1].set_ylim(bottom=0,top=0.3)
				polar_ax.set_rmax(5)
			'''
			'''
				ax[1].set_xlim(range_bin[1],range_bin[-1])
				ax[1].grid()
				ax[1].set_ylim(bottom=0,top=ax[1].get_ylim()[1])
				
				ax[2].set_xlim(vel_bin[0],vel_bin[-1])
				ax[2].grid()
				
				#raw_input("Press Enter to continue...")
				#plt.close('all')
			'''
			

    
    
# Main function
def main():

    # Initiate node
    rospy.init_node('main', anonymous=True)

    # Flight mode object
    
    # controller object
    rad = Radar()
    

    # ROS loop rate, [Hz]
    rate = rospy.Rate(10.0)

    # Subscribe to drone state
    rospy.Subscriber('/radar', Event, rad.stateCb)

    #df = pd.DataFrame({'R1': [], 'I1': [], 'R2': [], 'I2': []})
    #column_order = ['R1', 'I1', 'R2', 'I2']
    #df[column_order].to_csv(RECORDING_FILE_PATH, mode='w')

    pub = rospy.Publisher('/radar_processing_output', Radar_processing_out, queue_size=10)

    # ROS main loop
    while not rospy.is_shutdown():
        rad.updateSp()
        pub.publish(rad.algo_process_output)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
