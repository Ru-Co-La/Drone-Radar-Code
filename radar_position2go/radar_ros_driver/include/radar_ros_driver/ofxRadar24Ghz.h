#include <stdio.h>
#include <string.h>
#include <valarray>

// Radar - Host communication
#include "radar_ros_driver/EndpointCalibration.h"
#include "radar_ros_driver/EndpointTargetDetection.h"
#include "radar_ros_driver/EndpointRadarAdcxmc.h"
#include "radar_ros_driver/EndpointRadarBase.h"
#include "radar_ros_driver/EndpointRadarDoppler.h"
#include "radar_ros_driver/EndpointRadarFmcw.h"
#include "radar_ros_driver/EndpointRadarP2G.h"
#include "radar_ros_driver/EndpointRadarIndustrial.h"
#include "radar_ros_driver/Protocol.h"
#include "radar_ros_driver/COMPort.h"

// FFT
#include <iostream>
#include <complex>
#include <fstream>
#include <vector>

// KF
#include <radar_ros_driver/Eigen/Dense>
#include "radar_ros_driver/Hungarian.h"
#include <ros/ros.h>


// Global Definitions
#define FRAME_PERIOD_MSEC 			(150U)    	    // Time period of one frame to capture
#define ANTENNA_SPACING 		    0.0062          // For angle estimation
#define LAMBDA 					    0.0124          // For angle estimation

// Tracking
#define LIFE_TIME_COUNT	            (10U)			// Number of frames after which track is killed
#define GHOST_LIFE_TIME	            (5U)			// Number of frames after which ghost track is killed
#define DELTA_PX_CM		            (80.0f)		    // Size of prediction window, 2 (80),3 (120) bins
#define FRAME_PERIOD_MSEC 		    (150U)

#define RANGE_FFT_SIZE              256
#define DOWN_CHIRP_DURATION         0.0001
#define CHIRP_TO_CHIRP_DELAY        0.0001
#define RANGE_THRESHOLD             700//200
#define DOPPLER_THRESHOLD           50
#define MIN_DISTANCE                0
#define MAX_DISTANCE                4
#define MAX_NUM_TARGETS             5
#define MAX_NUM_TRACKS              5
#define INDEX_ZERO_DOPPLER          17

#define MAX_MEDIAN_FILTER_LEN       17	            // Must be odd
#define NUM_OF_CHIRPS               32
#define CURRENT_NUM_OF_TRACKS       1

#define AUTOMATIC_DATA_FRAME_TRIGGER 0		        // define if automatic trigger is active or not
#define AUTOMATIC_DATA_TRIGER_TIME_US (200000)	        // get ADC data each 300us in not automatic trigger mode
#define SPEED_OF_LIGHT              2.998e8

#define CHIRP_DUR_NS                300000
#define	NUM_CHIRPS_PER_FRAME        32
#define DOPPLER_FFT_SIZE            64	            // == NUM_CHIRPS_PER_FRAME!!!
#define	NUM_SAMPLES_PER_CHIRP       64
#define E_SIGNAL_PART               2  			    //ONLY_I = 0 /  ONLY_Q = 1 / I_AND_Q = 2
#define RX_MASK 					3			
// Each available RX antenna is represented by a bit in this mask. 
// If a bit is set, the IF signal received through the according RX antenna is captured during chirp processing.
#define RX_NUM_ANTENNAS             2
#define PEAK_TRESHOLD               0.7

#define MIN_ANGLE_FOR_ASSIGNMENT    50.0
#define IGNORE_NAN		            (555U)
#define ANGLE_QUANTIZATION			(1U)		    // Enable and set the Number of degrees

#define MTI_FILTER_LEN              100
#define MAXIMUM_NUMBER_HISTORY      40

#define LOGNAME_FORMAT 				"data/radar24G_dump_%Y%m%d_%H%M%S.dump"
#define LOGNAME_SIZE 				100

#define PI							3.14159265358979323846

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * \brief Data structure for Median filtering.
 * @{
 */
typedef struct
{
	uint8_t  is_full;
	uint32_t median_filter_len;
	float    buffer[MAX_MEDIAN_FILTER_LEN];
} Median_Filtering_t;

/**
 * \brief Data structure for current measurements used in data association.
 * @{
 */
typedef struct
{
	uint16_t is_associated;
	float    strength;
	float    range;
	float    speed;
	float	 angle;
	float    rx1_angle_arg_re;
	float    rx1_angle_arg_im;
	float    rx2_angle_arg_re;
	float    rx2_angle_arg_im;
} Measurement_elem_t;

/**
 * \brief Data structure for Track parameters used in tracking.
 * @{
 */
typedef struct
{
	uint8_t  track_id;
	uint8_t  is_alived;
	uint16_t speed_count;
	uint16_t range_change_flag;
	uint16_t lifetime_counter;
	uint32_t measurement_counter;
	float    strength;
	float    range;
	float    speed;
	float    speed_th;
	float    angle;
	float    rx1_angle_arg_re[NUM_OF_CHIRPS];
	float    rx1_angle_arg_im[NUM_OF_CHIRPS];
	float    rx2_angle_arg_re[NUM_OF_CHIRPS];
	float    rx2_angle_arg_im[NUM_OF_CHIRPS];
	float    d_phi;
} Tracking_Params_t;

/**
 * \brief Data structure for Tracking List.
 * @{
 */
typedef struct
{
	uint32_t num_of_tracks;
	uint32_t max_num_of_tracks;
	Tracking_Params_t elems[CURRENT_NUM_OF_TRACKS];
} tracking_list_t;

/*
 * Algorithm settings structure
 */
typedef struct
{
	uint8_t    isUpdated;
	uint8_t    isChecked;

	uint32_t   max_number_of_targets;
	uint32_t   max_number_of_tracks;

	uint32_t   num_of_tracks;
	uint32_t   mvg_avg_len;
	uint32_t   median_filter_len;
	uint32_t   mti_filter_len;
	uint32_t   mti_filter_enable;
	uint32_t   range_offset_cm;    // provided via calibration endpoint/struct!
	int16_t    angle_offset_deg;   // provided via calibration endpoint/struct!

	uint32_t   min_distance_cm;
	uint32_t   max_distance_cm;
	uint32_t   range_detection_threshold;

	uint32_t   min_speed_kmh;
	uint32_t   max_speed_kmh;
	float      wave_length_ant_spacing_ratio;
	float	   min_angle_for_track_assignment;
} algo_settings_t;


typedef struct
{
	double d_phi;
	float target_angle;
}target_angle_data;

typedef struct
{
	double x;
	double y;
}target_history;


typedef struct
{
	int index;
	double peak_val;
}target_peak;


using namespace std;



class ofxRadar24Ghz {

	public:
		void setup();
		// void draw();
		void update();
		int countSetBits(unsigned int n);
		static int compare_float(const void* a, const void* b);
		void startRadarUSB();

		// ofxVectorGraphics output;

		//TRACKING
		Median_Filtering_t *median_angle_arr;//[CURRENT_NUM_OF_TRACKS];
		//Median_Filtering_t median_angle_arr[CURRENT_NUM_OF_TRACKS];;//[CURRENT_NUM_OF_TRACKS];
		float *rx_angle_fft;//[2*DOPPLER_FFT_SIZE];
		float *rx_angle_fft_spectrum;//[DOPPLER_FFT_SIZE];
		uint32_t frame_period_usec;
		algo_settings_t *cp_algo_settings;
		tracking_list_t *tracking_list;
		target_history * pos_history;
		int n_points_history;

		// frame initialize memory
		int num_chirps;
		int num_samples_per_chirp;
		int esignalpart;
		int rx_mask;
		int num_antennas;

		int radar_handle = 0;
		int num_of_ports = 0;
		char comp_port_list[256];
		char* comport;
		const char *delim = ";";

		int res;
		int protocolHandle;
		int endpointRadarBase;
		int endpointRadarFmcw;
		int endpointRadarP2G;
		bool acq_started;


		// Algorithm
		bool enable_tracking;
		double speed_of_light;
		double fC;
		double PRT;
		int fs;
		int BW;
		int range_fft_size;
		int doppler_fft_size;
		int range_threshold;
		int doppler_threshold;
		int min_distance;
		int max_distance;
		int max_num_targets;
		double lambda;
		double hz_to_mps_constant;
		double if_scale;
		double *range_window_func;
		double *dopper_window_func;
		int r_max;
		double dist_per_bin;
		vector<double> array_bin_range;
		double fD_max;
		double fD_per_bin;
		vector<double> array_bin_fD;

		bool enable_mti_filtering;
		complex<double> * range_fft_spectrum_hist1; // FOR MTI
		complex<double> * range_fft_spectrum_hist2; // FOR MTI
		complex<double> * fft_1; // range FFT
		complex<double> * fft_2; // range FFT


		// FFT
		//fftw_complex *out;
		//fftw_plan plan_forward;
		double *adc_real_tx1rx1;	// REAL
		double *adc_imag_tx1rx1;	// IMG
		double *adc_real_tx1rx2;	// REAL
		double *adc_imag_tx1rx2;	// IMG

		// ADC I AND Q two antennas
		float *full_data_block;
		float *temperature;
		void  *frame_format_current;
		void  *device_info;
		void  *fmcw_cfg;
		uint32_t  *chirp_duration_ns;
		uint32_t  *min_frame_interval_us;
		uint32_t  *tx_power_001dBm;

		// Gating distance:
		double d_abs = 0.4;//meters
		// Frame number
		uint32_t frame_id = 0;
		// Measurement vector to be buffered and fed into tracking algo
		vector<Measurement_elem_t> measurements;
		// true angle offset
		double angle_correction = -8.0;
		// Covariance initiation factor:
		double cov_factor = 1;
		double time_stamp = 0;
		double time_rec = 0;
		double delay_s;
		// =========================================

		vector<bool> recordingstatus;
		ofstream binaryDataOut;
		ifstream bindayDataIn;
		bool isrecording;

		bool isloaddata;
		bool file_loaded;
		bool repeat_mode;

		bool islive;


		int 	radar_auto_connect();  // just pick the only radar available
		
		void 	print_status_code( int32_t protocol_handle, int32_t status);

		////////////////////////////////////
		// callback functions COMMUNICATION
		////////////////////////////////////

		// query frame format
		void get_frame_format(int32_t protocol_handle,
				uint8_t endpoint,
				Frame_Format_t* frame_format);


		static void 	received_frame_data(void* context,
							int32_t protocol_handle,
							uint8_t endpoint,
							const Frame_Info_t* frame_info);

		static void received_frame_format(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Frame_Format_t* frame_format);

		static void received_temperature(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint8_t temp_sensor,
				int32_t temperature_001C);

		static void get_chirp_duration(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint32_t chirp_duration_ns);

		static void get_device_info(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Device_Info_t* device_info);

		static void get_tx_power(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint8_t tx_antenna,
				int32_t tx_power_001dBm);

		static void get_fmcw_configuration(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Fmcw_Configuration_t* fmcw_configuration);


		static void get_bw_sec(void* context,
			int32_t protocol_handle,
			uint8_t endpoint,
			uint32_t bandwidth_per_second);


		static void set_fmcw_conf(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Fmcw_Configuration_t*
				fmcw_configuration);

		static void get_min_frame_interval(void* context,
										int32_t protocol_handle,
										uint8_t endpoint,
										uint32_t min_frame_interval_us);

	};
