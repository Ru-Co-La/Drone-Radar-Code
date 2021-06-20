#include "radar_ros_driver/ofxRadar24Ghz.h"
#include <iostream>
#include<unistd.h>



//--------------------------------------------------------------
void ofxRadar24Ghz::setup() {

	// via definitions
	num_chirps = NUM_CHIRPS_PER_FRAME;
	num_samples_per_chirp = NUM_SAMPLES_PER_CHIRP;
	esignalpart = E_SIGNAL_PART;
	rx_mask = RX_MASK;
	num_antennas = countSetBits(rx_mask);
	speed_of_light = SPEED_OF_LIGHT;// SPEED OF LIGHT

	// allocate memory for callbacks
	full_data_block = (float *)malloc(num_antennas * num_chirps * num_samples_per_chirp * 2 * sizeof(float) );	// I and Q
	temperature = (float *)malloc(1 * sizeof(float));
	frame_format_current = (Frame_Format_t *)malloc(1 * sizeof(Frame_Format_t));
	device_info = (Device_Info_t *)malloc(1 * sizeof(Device_Info_t));
	chirp_duration_ns = (uint32_t*)malloc(1*sizeof(uint32_t));
	min_frame_interval_us = (uint32_t*)malloc(1*sizeof(uint32_t));
	tx_power_001dBm = (uint32_t*)malloc(1*sizeof(uint32_t));
	fmcw_cfg = (Fmcw_Configuration_t*)malloc(1 * sizeof(Fmcw_Configuration_t));
	// data
	adc_real_tx1rx1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_imag_tx1rx1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_real_tx1rx2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_imag_tx1rx2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));

	// generals
	radar_handle = 0;
	num_of_ports = 0;
	res = -1;
	protocolHandle = 0;
	endpointRadarBase = 0;

	// START CONNECTION TO RADAR VIA USB
	startRadarUSB();

	Device_Info_t *this_device_infos = (Device_Info_t *) (device_info);
 	fC = ((double)this_device_infos->max_rf_frequency_kHz + (double)this_device_infos->min_rf_frequency_kHz)/2.0 * 1e3;

	fs = 426666; // Adcxmc configuration
	PRT = 0.0005;//chirp_duration_ns + DOWN_CHIRP_DURATION + CHIRP_TO_CHIRP_DELAY;
	BW = 200000000; // in HZ
	range_fft_size = RANGE_FFT_SIZE;
	doppler_fft_size = DOPPLER_FFT_SIZE;
	range_threshold = RANGE_THRESHOLD;
	doppler_threshold = DOPPLER_THRESHOLD;
	min_distance = MIN_DISTANCE; // m
	max_distance = MAX_DISTANCE;
	max_num_targets = MAX_NUM_TARGETS;
	lambda = SPEED_OF_LIGHT/fC;
	
	hz_to_mps_constant=lambda/2.0;
	if_scale= 1;//16 * 3.3*range_fft_size/num_samples_per_chirp;
}


void ofxRadar24Ghz::startRadarUSB(){

	// open COM port
	protocolHandle = radar_auto_connect();

	// get endpoint ids
	if (protocolHandle >= 0)
	{
		for (int i = 1; i <= protocol_get_num_endpoints(protocolHandle); ++i) {
			// current endpoint is radar base endpoint
			if (ep_radar_base_is_compatible_endpoint(protocolHandle, i) == 0) {
				endpointRadarBase = i;
				continue;
			}
			if (ep_radar_fmcw_is_compatible_endpoint(protocolHandle, i) == 0) {
				endpointRadarFmcw = i;
				continue;
			}
			if (ep_radar_p2g_is_compatible_endpoint(protocolHandle, i) == 0) {
				endpointRadarP2G = i;
				continue;
			}
		}
	}

	if (endpointRadarBase >= 0)
	{
		// compatible in all means
		uint32_t is_compatible = ep_radar_base_is_compatible_endpoint(protocolHandle,endpointRadarBase);
		// ------------------------------ --------------------------------------------------
		// ofLog(OF_LOG_WARNING, "EP RADAR IS COMPATIBLE"); 
		// ------------------------------ --------------------------------------------------
		print_status_code( protocolHandle, is_compatible);

		// callback get device info
		ep_radar_base_set_callback_device_info(this->get_device_info, device_info);
		// callback for frame format messages.
		ep_radar_base_set_callback_frame_format(this->received_frame_format, frame_format_current);
		// callback min frame interval
		ep_radar_base_set_callback_min_frame_interval(this->get_min_frame_interval, min_frame_interval_us);
		// callback chirp_duration
		ep_radar_base_set_callback_chirp_duration(this->get_chirp_duration, chirp_duration_ns);
		// register call back functions for adc data
		ep_radar_base_set_callback_data_frame(this->received_frame_data, full_data_block);
		// register call back for tx power read
		ep_radar_base_set_callback_tx_power(this->get_tx_power, tx_power_001dBm);
		// register call back for fmcw configuration
		ep_radar_fmcw_set_callback_fmcw_configuration(this->get_fmcw_configuration, fmcw_cfg);

		// Stopping the device during setup is necessary to avoid finding the device busy!
		uint32_t stop_trigger = ep_radar_base_set_automatic_frame_trigger(protocolHandle, endpointRadarBase,0);
		printf("Stop auto trigger: ");
		print_status_code(protocolHandle,stop_trigger);

		// get device info
		uint32_t dev_info_status = ep_radar_base_get_device_info(protocolHandle,endpointRadarBase);
		print_status_code( protocolHandle, dev_info_status);
		// get power
		int32_t answer = ep_radar_base_get_tx_power(protocolHandle, endpointRadarBase, 0);
		print_status_code(protocolHandle, answer);
		// get current frame format
		Frame_Format_t* frame_format_now;
		frame_format_now = (Frame_Format_t *)malloc(1 * sizeof(Frame_Format_t));
		this->get_frame_format(protocolHandle, endpointRadarBase, frame_format_now);

		/* If the frame format contains a 0, this makes no sense. */
		if ((frame_format_now->rx_mask == 0) ||
		  (frame_format_now->num_chirps_per_frame  == 0) ||
			(frame_format_now->num_samples_per_chirp == 0) ||
			  (frame_format_now->num_chirps_per_frame  > (uint32_t)num_chirps) ||
				(frame_format_now->num_samples_per_chirp > (uint32_t)num_samples_per_chirp))
		{
			printf("frame format error\n");
			// ------------------------------ --------------------------------------------------
			// ofExit();
			// ------------------------------ --------------------------------------------------
		}

		// set current frame format to 64 64
		frame_format_now->num_chirps_per_frame = num_chirps;
		frame_format_now->num_samples_per_chirp = num_samples_per_chirp;
		frame_format_now->eSignalPart = (Signal_Part_t)esignalpart;
		frame_format_now->rx_mask = rx_mask;
		//std::cout << "Ehi guarda qui " << frame_format_now->num_chirps_per_frame << endl;
		int32_t jj =  ep_radar_base_set_frame_format(protocolHandle,endpointRadarBase,frame_format_now);
		this->get_frame_format(protocolHandle, endpointRadarBase, frame_format_now);
		print_status_code(protocolHandle, jj);

		//get chirp duration
		int32_t chirp_duration_status = ep_radar_base_get_chirp_duration(protocolHandle, endpointRadarBase);
		print_status_code( protocolHandle, chirp_duration_status);

		// get min frame interval
		uint32_t min_frame = ep_radar_base_get_min_frame_interval(protocolHandle, endpointRadarBase);
		print_status_code( protocolHandle, min_frame);

		// distance calculations
		Device_Info_t *this_device_info = (Device_Info_t *) (device_info);
		double bandwidth_hz = (double)(this_device_info->max_rf_frequency_kHz-this_device_info->min_rf_frequency_kHz)*1000.0;
		double ramp_time_s = (double)(*chirp_duration_ns)*1e-9;
		printf("bandwidth_Hz %f\n", bandwidth_hz);
		printf("ramp_time_s %f\n", ramp_time_s);
		printf("speed_of_light %f\n", speed_of_light);

		islive = true;
	}else{
		printf("init radar failed");
		islive = false; // no radar
	}
	if(endpointRadarFmcw >= 0){
		Fmcw_Configuration_t fmcw_configuration;
		fmcw_configuration.lower_frequency_kHz = 24025000;
		fmcw_configuration.upper_frequency_kHz = 24225000;
		fmcw_configuration.direction = EP_RADAR_FMCW_DIR_UPCHIRP_ONLY;
		std::cout << "Tx power? [1 - 7]" << endl;
		int tx_pow = 7;
		std::cin >> tx_pow;
		if(tx_pow >= 1 && tx_pow <= 7){
			fmcw_configuration.tx_power = (uint8_t) tx_pow;
		}
		else{
			std::cout << "Value out of range: set Tx power to 4" << endl;
			fmcw_configuration.tx_power = 4;
		}
		int32_t return_code = ep_radar_fmcw_set_fmcw_configuration(protocolHandle, endpointRadarFmcw, &fmcw_configuration);
		print_status_code(protocolHandle, return_code);
		return_code = ep_radar_fmcw_get_fmcw_configuration(protocolHandle, endpointRadarFmcw);
	}

	if (endpointRadarP2G >= 0){
		//std::cout << "PGA level? [1 - 7]" << endl;
		int pga_level = 4;
		//std::cin >> pga_level;
		if(pga_level >= 1 && pga_level <= 7){
			int res = ep_radar_p2g_set_pga_level(protocolHandle, endpointRadarP2G, pga_level);
		}
		else{
			std::cout << "Value out of range: set PGA level level to 4" << endl;
			int res = ep_radar_p2g_set_pga_level(protocolHandle, endpointRadarP2G, pga_level);
		}
	}

}

//--------------------------------------------------------------
int ofxRadar24Ghz::radar_auto_connect() {
	// usb connections
	num_of_ports = com_get_port_list(comp_port_list, (size_t)256);
	if (num_of_ports == 0)
	{
		return -1;
	}
	else
	{
		comport = strtok(comp_port_list, delim);
		while (num_of_ports > 0)
		{
			num_of_ports--;
			// open COM port
			radar_handle = protocol_connect(comport);
			if (radar_handle >= 0)
			{
				break;
			}
			comport = strtok(NULL, delim);
		}
		return radar_handle;
	}
}

//--------------------------------------------------------------
void ofxRadar24Ghz::update() {
	// get raw data
	if(islive and !isloaddata){
		if(!acq_started){
			// start acquisition
			// enable/disable automatic trigger
			if (AUTOMATIC_DATA_FRAME_TRIGGER){
				res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
																endpointRadarBase,
																AUTOMATIC_DATA_TRIGER_TIME_US);
			}else{
				res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
																endpointRadarBase,
																0);

			}
			if(res != -1){
				acq_started = true;
				const char* pointer_to_status_code = protocol_get_status_code_description(protocolHandle, res);
				while(*pointer_to_status_code != '\0'){
					std::cout << *pointer_to_status_code;
					pointer_to_status_code++;
				}
				std::cout << endl;
			}else{
				printf("CANNOT START ACQUISITION\n");
				islive = false;
			}
		}
		unsigned int millisecond = 1000;
		int counter = 0;
		while(counter < 25){
			res = ep_radar_base_get_frame_data(protocolHandle,	endpointRadarBase,	0);
			usleep(200 * millisecond);//sleeps for 5 ms
			counter++;
			std::cout << counter;
		}		

		//res = ep_radar_base_get_frame_data(protocolHandle,	endpointRadarBase,	1);
		if(res != -1){
			// IF LIVE DATA
			for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
				for (uint32_t ANTENNA_NUMBER = 0;ANTENNA_NUMBER < (uint32_t)num_antennas ; ANTENNA_NUMBER++){
					for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < (uint32_t)num_samples_per_chirp; SAMPLE_NUMBER++){


						double this_adc_real = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						double this_adc_img  = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						//if (SAMPLE_NUMBER == 56 && ANTENNA_NUMBER==0){
						//	printf("%d: %f\n", CHIRP_NUM, this_adc_real);
						//}


						if(ANTENNA_NUMBER == 0){
							adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}else if (ANTENNA_NUMBER == 1){
							adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}

					}
				}
			} // chirp
			//printf("\n");
		}else{
			islive = false; // something has happed to the radar connection
		}
	}

	if(isloaddata and !islive ){
		// open file and process
		if(bindayDataIn.is_open()){
			STARTFILE:
			bindayDataIn.read((char *) full_data_block, num_antennas * num_chirps * num_samples_per_chirp * 2*sizeof(float) );
		}
		if(!bindayDataIn.good()) {
			printf("FILE ENDED");
			bindayDataIn.clear();
			bindayDataIn.seekg(0, ios::beg);
			goto STARTFILE;
		}else{
			for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
				for (uint32_t ANTENNA_NUMBER = 0;ANTENNA_NUMBER < (uint32_t)num_antennas ; ANTENNA_NUMBER++){
					for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER <(uint32_t) num_samples_per_chirp; SAMPLE_NUMBER++){

						double this_adc_real = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						double this_adc_img  = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;

						if(ANTENNA_NUMBER == 0){
							adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}else if (ANTENNA_NUMBER == 1){
							adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}

					}
				} // antenna number
			} // chirp
		}
	}
}

//===========================================================================
int ofxRadar24Ghz::compare_float(const void *a, const void *b){
	int retval = 0;

	float a_f = *(float*)a;
	float b_f = *(float*)b;

	if (a_f > b_f)
	{
		retval = 1;
	}
	else if (a_f < b_f)
	{
		retval = -1;
	}

	return retval;
}
//===========================================================================
void ofxRadar24Ghz::print_status_code( int32_t protocol_handle, int32_t status){

	//check status
	const char * hr_current_frame_format = protocol_get_status_code_description(protocol_handle, status);
	char buffer [1500];
	//int n;
	sprintf(buffer, "%s", hr_current_frame_format);
	printf("[%s]\n",buffer);


}

//===========================================================================
void ofxRadar24Ghz::get_frame_format( int32_t protocol_handle,
        uint8_t endpoint,
        Frame_Format_t* frame_format){

	// query
	int32_t current_frame_format = ep_radar_base_get_frame_format(protocolHandle, endpointRadarBase);
	print_status_code(protocolHandle, current_frame_format);
	//std::cout << "Ehi guarda qui dentro get_frame_format " << ((Frame_Format_t *) frame_format_current)->num_chirps_per_frame << endl;

	// cast and read data format
	Frame_Format_t * frame_format_disp = (Frame_Format_t *) (frame_format_current);
	printf("num_chirps_per_frame %d\n", frame_format_disp->num_chirps_per_frame);
	printf("num_samples_per_chirp %d\n", frame_format_disp->num_samples_per_chirp);
	printf("rx_mask %d\n", frame_format_disp->rx_mask);
	printf("ONLY_I = 0 /  ONLY_Q = 1 / I_AND_Q = 2 %d\n", frame_format_disp->eSignalPart);

	frame_format->num_chirps_per_frame = frame_format_disp->num_chirps_per_frame;
	frame_format->num_samples_per_chirp = frame_format_disp->num_samples_per_chirp;
	frame_format->rx_mask = frame_format_disp->rx_mask;
	frame_format->eSignalPart = frame_format_disp->eSignalPart;

}

//===========================================================================
void ofxRadar24Ghz::received_frame_data(void* context,
						int32_t protocol_handle,
		                uint8_t endpoint,
						const Frame_Info_t* frame_info){

    float *full_data_block = (float *) (context);
    int num_ant = 2;
    if(frame_info->rx_mask == 3){
    	num_ant = 2;
    }

	/*printf("RFM frame_number %d\n", frame_info->frame_number); //12
	printf("RFM num_chirps %d\n", frame_info->num_chirps);
	printf("RFM num_rx_antennas %d\n", frame_info->num_rx_antennas);
	printf("RFM num_samples_per_chirp %d\n", frame_info->num_samples_per_chirp);
	printf("RFM rx_mask %d\n", frame_info->rx_mask);
	printf("RFM interleaved_rx %d\n", frame_info->interleaved_rx);
	printf("RFM data_format %d\n", frame_info->data_format);*/
    /*frame_start = &frame_info->sample_data[CHIRP_NUMBER *
                                  num_rx_antennas *
                                  num_samples_per_chirp *
                                ((data_format == RADAR_RX_DATA_REAL)? 1 : 2)];*/

    /* data_value_real = frame_start[(2 * ANTENNA_NUMBER    ) *
                                   num_samples_per_chirp + SAMPLE_NUMBER];
    * data_value_imag = frame_start[(2 * ANTENNA_NUMBER + 1) *
    *                               num_samples_per_chirp + SAMPLE_NUMBER];*/


	for (uint32_t ANTENNA_NUMBER = 0; ANTENNA_NUMBER < (uint32_t)num_ant ; ANTENNA_NUMBER++){
		//uint32_t start = ant*frame_info->num_chirps*frame_info->num_samples_per_chirp*1
		for (uint32_t CHIRP_NUMBER = 0;CHIRP_NUMBER <  frame_info->num_chirps; CHIRP_NUMBER++){
			for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < frame_info->num_samples_per_chirp; SAMPLE_NUMBER++)
			{
				if(frame_info->data_format != 0){
					const float * frame_start =  &frame_info->sample_data[CHIRP_NUMBER*num_ant*frame_info->num_samples_per_chirp*2];

					full_data_block[CHIRP_NUMBER*4*frame_info->num_samples_per_chirp + (2*ANTENNA_NUMBER)*frame_info->num_samples_per_chirp + SAMPLE_NUMBER] =
												frame_start[(2*ANTENNA_NUMBER)*frame_info->num_samples_per_chirp+SAMPLE_NUMBER];

					full_data_block[CHIRP_NUMBER*4*frame_info->num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*frame_info->num_samples_per_chirp + SAMPLE_NUMBER] =
												frame_start[(2*ANTENNA_NUMBER+1)*frame_info->num_samples_per_chirp+SAMPLE_NUMBER];

				}else{
					printf("Not implemented: data format is real.. please check format.");
				}
			}
		}
	}
}

/* Function to get no of set bits in binary
   representation of positive integer n */
//===========================================================================
int ofxRadar24Ghz::countSetBits(unsigned int n){
	unsigned int count = 0;
	while (n) {
		count += n & 1;
		n >>= 1;
	}
	return count;
}

//===========================================================================
void ofxRadar24Ghz::received_temperature(void* context,
		int32_t protocol_handle,
        uint8_t endpoint,
		uint8_t temp_sensor,
        int32_t temperature_001C){

	//
    //float *temperature = (float *) (context);
    //printf("temperature %d:\n", frame_info->num_temp_sensors);

}

//===========================================================================
void ofxRadar24Ghz::received_frame_format(void* context,
		int32_t protocol_handle,
        uint8_t endpoint,
        const Frame_Format_t* frame_format){

	Frame_Format_t *frame_format_current = (Frame_Format_t *) (context);

	frame_format_current->num_chirps_per_frame = frame_format->num_chirps_per_frame;
	frame_format_current->num_samples_per_chirp = frame_format->num_samples_per_chirp;
	frame_format_current->rx_mask = frame_format->rx_mask;
	frame_format_current->eSignalPart = frame_format->eSignalPart;

}

//===========================================================================
void ofxRadar24Ghz::get_device_info(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
		const Device_Info_t * device_info){

	Device_Info_t *this_device_info = (Device_Info_t *) (context);

	this_device_info->description = device_info->description;
	this_device_info->min_rf_frequency_kHz = device_info->min_rf_frequency_kHz;
	this_device_info->max_rf_frequency_kHz = device_info->max_rf_frequency_kHz;
	this_device_info->num_tx_antennas = device_info->num_tx_antennas;
	this_device_info->num_rx_antennas = device_info->num_rx_antennas;
	this_device_info->max_tx_power = device_info->max_tx_power;
	this_device_info->num_temp_sensors = device_info->num_temp_sensors;
	this_device_info->major_version_hw = device_info->major_version_hw;
	this_device_info->minor_version_hw = device_info->minor_version_hw;
	this_device_info->interleaved_rx = device_info->interleaved_rx;
	this_device_info->data_format = device_info->data_format;

	printf("max_tx_power %d\n", device_info->max_tx_power);
	printf("num_tx_antennas %d\n", device_info->num_tx_antennas);
	printf("num_rx_antennas %d\n", device_info->num_rx_antennas);
	printf("data_format %d interleaved_rx %d\n", device_info->data_format, device_info->interleaved_rx);
	printf("min_rf_frequency_kHz %d max_rf_frequency_kHz %d\n", device_info->min_rf_frequency_kHz, device_info->max_rf_frequency_kHz);
	printf("bandwidth %d kHz\n", device_info->max_rf_frequency_kHz-device_info->min_rf_frequency_kHz);
	printf("num_temp_sensors  %d\n", device_info->num_temp_sensors);
	printf("version %d-%d\n", device_info->major_version_hw, device_info->minor_version_hw);

}

/* * \param[in] context          The context data pointer, provided along with
 *                             the callback itself through
 *                             \ref ep_radar_base_set_callback_tx_power.
 * \param[in] protocol_handle  The handle of the connection, the sending
 *                             device is connected to.
 * \param[in] endpoint         The number of the endpoint that has sent the
 *                             message.
 * \param[in] tx_antenna       The number of the TX antenna from which the
 *                             power was measured.
 * \param[in] tx_power_001dBm  The power value in 0.001 dBm.*/
//===========================================================================
void ofxRadar24Ghz::get_tx_power(void* context,
			int32_t protocol_handle,
			uint8_t endpoint,
			uint8_t tx_antenna,
			int32_t tx_power_001dBm){

	uint32_t * power_set = (uint32_t *) context;
	*power_set = tx_power_001dBm;
	printf("power is set to %f dBm\n", (double)tx_power_001dBm*(1e-3));


}

void ofxRadar24Ghz::get_fmcw_configuration(void* context,
			int32_t protocol_handle,
			uint8_t endpoint,
			const Fmcw_Configuration_t *fmcw_configuration){
	std::cout << "TX power " << (int)fmcw_configuration->tx_power << endl;
}

//===========================================================================
void ofxRadar24Ghz::set_fmcw_conf(void* context,
            int32_t protocol_handle,
            uint8_t endpoint,
            const Fmcw_Configuration_t*
              fmcw_configuration){

	//Fmcw_Configuration_t *sd = (Fmcw_Configuration_t * )context;
	printf("lower_frequency_kHz %d \n", fmcw_configuration->lower_frequency_kHz);
	printf("upper_frequency_kHz %d \n", fmcw_configuration->upper_frequency_kHz);
	printf("tx_power %d \n", fmcw_configuration->tx_power);


}

//===========================================================================
void ofxRadar24Ghz::get_bw_sec(void* context,
		   int32_t protocol_handle,
		   uint8_t endpoint,
		   uint32_t bandwidth_per_second){

	uint32_t * bps = (uint32_t *) context;
	printf("bandwidth_per_second %d \n", bandwidth_per_second);
	* bps = bandwidth_per_second;

}

//===========================================================================
void ofxRadar24Ghz::get_chirp_duration(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
        uint32_t chirp_duration_ns){

	uint32_t * cd = (uint32_t *) context;
	printf("chirp Duration is %d ns\n", chirp_duration_ns);
	*cd = chirp_duration_ns;
}

//===========================================================================
void ofxRadar24Ghz::get_min_frame_interval(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
        uint32_t min_frame_interval_us){

	uint32_t * cd = (uint32_t *) context;
	printf("min_frame_interval is %d us\n", min_frame_interval_us);
	*cd = min_frame_interval_us;

}

// --------------------------------------------------------------
/**
 * to map a range of values to RGB color gradients
 */
//===========================================================================
