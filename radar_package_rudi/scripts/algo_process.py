import frame
import algo_result as alg_res
import numpy as np

def apply_window(data_in, window, direction):
    if direction == 'fast':
        return (data_in.T*window).T
    if direction == 'slow':
        return (data_in*window)
        
def algo_process(frame,range_zero_pad,doppler_zero_pad,peak_thresh,peak_slice,peak_width,peak_relevance,search_zone,angle_calibration_offset,angle_precision,wavelen_spacing_rat,bin2range):

    #Fast time Windowing
    tmp_frame = frame.chirp
    window = np.hamming(frame.sam_x_chirp)
    tmp_frame = apply_window(tmp_frame, window,'fast')
    #Zero-pad & Fast FFT & normalization
    fast_fft_data_out = np.fft.fft(np.pad(tmp_frame,((0,range_zero_pad-frame.sam_x_chirp),(0,0),(0,0)), mode='constant'), axis=0, norm='ortho')
    #MTI filter
    eta = 0.2
    F = np.zeros(range_zero_pad, dtype='complex')
    K = np.zeros(range_zero_pad, dtype='complex')
    X = np.zeros(range_zero_pad, dtype='complex')
    avg_0 = 0*np.mean(fast_fft_data_out[:,0,:], axis=1)
    fast_fft_data_out[:,0,:] = (fast_fft_data_out[:,0,:].T - avg_0).T
    avg_1 = 0*np.mean(fast_fft_data_out[:,0,:], axis=1)
    fast_fft_data_out[:,1,:] = (fast_fft_data_out[:,1,:].T - avg_1).T
    for chirp in range(frame.chirp_x_frame):
        X = fast_fft_data_out[:,0,chirp] - K
        F = (1-eta)*F + eta*X
        K = (1-eta)*K + eta*F
    selected_chirp = np.abs(X)
    #Peak search
    search_zone = round(search_zone/bin2range)
    target_position,target_strength = peak_search(selected_chirp, np.abs(fast_fft_data_out[:,0,-1]) ,'fast',peak_thresh,peak_width,peak_relevance,search_zone)
    #Initialize target info
    target_info = alg_res.algo_result(target_position, target_strength,bin2range, peak_slice, frame.chirp_x_frame, doppler_zero_pad,wavelen_spacing_rat,int(360/angle_precision))
    #Target info collection
    for target in range(target_info.num_targets):
        range_spectrum_slice = peak_extraction(np.abs(fast_fft_data_out[:,0,:]),target_position[target],target_info.radius,target_info.bin2range)
        target_info.collect_target_range_spectrum(range_spectrum_slice)
        doppler_mark = apply_window((fast_fft_data_out[int(target_position[target]),0,:]),np.hamming(frame.chirp_x_frame),'slow')
        target_info.collect_target_doppler_mark(doppler_mark)
        antenna_phase_difference = phase_difference_estimation(fast_fft_data_out[int(target_position[target]), :, 0], target_info.beam_forming_matrix, angle_calibration_offset, angle_precision)
        target_info.collect_target_angle(antenna_phase_difference)
    return fast_fft_data_out,target_info, selected_chirp 
    
def peak_search(chirp,non_filtered_chirp,direction,thresh,peak_width,peak_relevance,min_search_zone):
    #slices = int(round(len(chirp)/search_zone))
    #print(search_zone)
    #print(slices)
    peak_frequency = []
    peak_str = []
    fbc = int(min_search_zone)
    for spectrum_slice in range(len(chirp) - 4):
        fbc = fbc + 1        
        fbl2 = fbc - 2
        fbl1 = fbc - 1
        fbr1 = fbc + 1
        fbr2 = fbc + 2
        if fbr2 >= len(chirp):
            continue
        pvl2 = chirp[fbl2]
        pvl1 = chirp[fbl1]
        pvc = chirp[fbc]
        pvr1 = chirp[fbr1]
        pvr2 = chirp[fbr2]
        if pvl2+peak_relevance < pvl1 and pvr1 > pvr2+peak_relevance and (pvc > thresh) and pvc >= pvl1 and pvc >= pvr1:
            peak_frequency.append((fbl2*non_filtered_chirp[fbl2] + fbl1*non_filtered_chirp[fbl1]+ fbc*non_filtered_chirp[fbc] + fbr1*non_filtered_chirp[fbr1] + fbr2*non_filtered_chirp[fbr2])/(non_filtered_chirp[fbl2] + non_filtered_chirp[fbl1] + non_filtered_chirp[fbc] + non_filtered_chirp[fbr1] + non_filtered_chirp[fbr2]))
            peak_str.append(non_filtered_chirp[fbc]) 
    return peak_frequency,peak_str
    
def peak_extraction(chirp,idx,radius,b2r):
    idx_slice = np.arange(round(idx) - round(radius/b2r),round(idx) + round(radius/b2r)).astype(int)
    #Prevent having negative idx or idx above length fft
    if((round(idx) - round(radius/b2r) < 0) or (round(idx) + round(radius/b2r) >= len(chirp))):
        #print("Warning: in peak_extraction idx_slice is out of bounds! Target is too close or too far.")
        #print(round(idx) - round(radius/b2r))
        #print(round(idx) + round(radius/b2r))
        
        for i in range(len(idx_slice)):
        #    if(idx_slice[i] < 0):
        #        idx_slice[i] = 0
            if(idx_slice[i] >= len(idx_slice)):
                idx_slice[i] = len(idx_slice) - 1
            
    return idx_slice,chirp[idx_slice,:]
    
def phase_difference_estimation(phase_array,beam_forming_matrix,offset,precision):
    phase_array[1] *= np.exp(1j*offset/180*np.pi)
    result = np.matmul(beam_forming_matrix,phase_array)
    return np.abs(result)
