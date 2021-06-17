import numpy as np

class Frame:
	def __init__(self, re1, re2, im1, im2, Ns, Nc):
	    self.sam_x_chirp = Ns
	    self.chirp_x_frame = Nc
	    self.chirp = np.zeros((Ns, 2, Nc), dtype='complex')
	    #Create the data-cube
	    for i in range(self.chirp_x_frame):
			self.chirp[:,0,i] = re1[Ns*i:Ns*(i+1)] + 1j*im1[Ns*i:Ns*(i+1)]
			self.chirp[:,1,i] = re2[Ns*i:Ns*(i+1)] + 1j*im2[Ns*i:Ns*(i+1)]
            
    #calibration_chirp is expected to be a numpy array
	def calibrate(self,calibration_chirp):
		if calibration_chirp.shape == (self.sam_x_chirp,2):
			for chirp in range(self.chirp_x_frame):
				self.chirp[:,:,chirp] -= calibration_chirp
