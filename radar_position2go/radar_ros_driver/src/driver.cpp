/**
 * This file is part of the odroid_ros_dvs package - MAVLab TU Delft
 * 
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 * 
 * */

#include "radar_ros_driver/driver.h"

namespace radar_ros_driver {

    RadarRosDriver::RadarRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh){

        ns = ros::this_node::getNamespace();
        if (ns == "/"){
            ns = "/radar";
        }
        
        radar_pub_ = nh_.advertise<radar_msgs::Event>("radar", 10);

        myradar.setup();
        running_ = true;

        ROS_INFO("Starting radar listener...");
    }

    RadarRosDriver::~RadarRosDriver(){

        if (running_){
            //TODO: close driver
            ROS_INFO("Shutting down listener...");
            running_ = false;
        }
    }

    void RadarRosDriver::readout(uint16_t count){

        if(running_){

            myradar.update();

            num_samples_per_chirp = myradar.num_samples_per_chirp;
            num_chirps = myradar.num_chirps;           

            adc_real_tx1rx1 = myradar.adc_real_tx1rx1;
            adc_imag_tx1rx1 = myradar.adc_imag_tx1rx1;
            adc_real_tx1rx2 = myradar.adc_real_tx1rx2;
            adc_imag_tx1rx2 = myradar.adc_imag_tx1rx2;


            for(int i=0; i<num_samples_per_chirp*num_chirps; i++){
                adc_real_tx1rx1_f[i] = (float_t)(adc_real_tx1rx1[i]);
                adc_imag_tx1rx1_f[i] = (float_t)(adc_imag_tx1rx1[i]);
                adc_real_tx1rx2_f[i] = (float_t)(adc_real_tx1rx2[i]);
                adc_imag_tx1rx2_f[i] = (float_t)(adc_imag_tx1rx2[i]);
            }
            
            // raw radar message (I,Q, both antennas)
            radar_msgs::Event event_msg;
            event_msg.dimx = num_chirps;
            event_msg.dimy = num_samples_per_chirp;
            event_msg.data_rx1_re = adc_real_tx1rx1_f; 
			event_msg.data_rx1_im = adc_imag_tx1rx1_f;
			event_msg.data_rx2_re = adc_real_tx1rx2_f;
			event_msg.data_rx2_im = adc_imag_tx1rx2_f;
            event_msg.ts = ros::Time::now(); 
            radar_pub_.publish(event_msg);  
        }

    }

} // namespace
