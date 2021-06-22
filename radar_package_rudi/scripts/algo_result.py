import numpy as np


class algo_result:
    def __init__(self,target_idx,target_str,bin2range,target_radius,chirps_per_frame,doppler_len,wav_spa_rat,angle_bins):
        self.num_targets = len(target_idx)
        self.bin2range = bin2range
        self.location = [idx*bin2range for idx in target_idx]
        self.strength = target_str
        self.radius = target_radius
        self.chirps_per_frame = chirps_per_frame
        self.doppler_len = doppler_len
        self.wavelength_spacing_ratio = wav_spa_rat
        self.last_added = 0
        self.local_print = np.zeros((int(2*round(target_radius/bin2range)),chirps_per_frame,len(target_idx)))
        self.print_location = np.zeros((int(2*round(target_radius/bin2range)),len(target_idx)))
        self.doppler_mark = np.zeros((chirps_per_frame,len(target_idx)), dtype='complex')
        self.doppler_spectrum = np.zeros((doppler_len,len(target_idx)))
        self.angle = np.arcsin(wav_spa_rat*np.linspace(-np.pi, np.pi, angle_bins)/(2*np.pi))/np.pi*180
        self.beam_forming_matrix = initialize_beam_forming_matrix(angle_bins)
        self.angle_spectrum = np.zeros((angle_bins, self.num_targets))
        
    def collect_target_range_spectrum(self, target_spectrum_info):
        self.print_location[:,self.last_added] = target_spectrum_info[0]*self.bin2range
        self.local_print[:,:,self.last_added] = target_spectrum_info[1]
        self.last_added += 1
        
    def collect_target_angle(self, target_angle):
		self.angle_spectrum[:,self.last_added-1] = flip(target_angle)
        
    def collect_target_doppler_mark(self, doppler_mark):
        self.doppler_mark[:,self.last_added-1] = doppler_mark
        #divide by self.doppler_len to normalize
        self.doppler_spectrum[:,self.last_added-1] = np.fft.fftshift(np.abs(np.fft.fft(np.pad(doppler_mark,(0,self.doppler_len-self.chirps_per_frame),mode='constant'))))/np.sqrt(self.doppler_len)
        
		
def initialize_beam_forming_matrix(angle_bins):
	deg2rad = np.pi/180
	tot_angle_bins = angle_bins
	angle = np.arange(-np.pi, np.pi, 1*deg2rad)
	complex_angle = (np.exp(1j*angle)).reshape((tot_angle_bins,1))
	phase_matrix = np.ones((tot_angle_bins,1))
	beam_forming_matrix = np.append(phase_matrix, complex_angle, axis=1)
	return beam_forming_matrix
    
def flip(myArr):
    array_len = len(myArr)
    for i in range(array_len/2):
        tmp = myArr[i]
        myArr[i] = myArr[-1-i]
        myArr[-1-i] = tmp
    return myArr
