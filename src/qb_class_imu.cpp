
#include "qb_class_imu.h"

//-----------------------------------------------------
//                                         qb_class_imu
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qb_class_imu class
/ *****************************************************
/   parameters:
/				port - usb port tfo activate 
/					   communication
/   return:
/
*/

qb_class_imu::qb_class_imu(){

	// Variables to get param 
	vector<int> ID_imuboard;

	string aux;

	// Initialize ROS Node
	node_ = new ros::NodeHandle("qb_interface_node_imu_");

	// Get param from roslaunch or yaml file
	node_->searchParam("/IDimuboards", aux);
	node_->getParam(aux, ID_imuboard);


    qbImuBoard* tmp_imuboard;

    for (int i = ID_imuboard.size(); i--;) {

        tmp_imuboard = new qbImuBoard(qb_comm_, ID_imuboard[i]);
        
       	// IF an error is find
        if (tmp_imuboard == NULL){
        	cout << "[ERORR] Unable to allocate space for imu board structure." << endl;
            return;
        }

        imuboard_chain_.push_back(tmp_imuboard);
    } 


	
    // init publisher
	imuboard_pub_acc_  = node_->advertise<std_msgs::Float64>("/qb_class_imu/acc", 1);
	imuboard_pub_gyro_ = node_->advertise<std_msgs::Float64>("/qb_class_imu/gyro", 1);
	imuboard_pub_mag_  = node_->advertise<std_msgs::Float64>("/qb_class_imu/mag", 1);
	imuboard_pub_quat_ = node_->advertise<std_msgs::Float64>("/qb_class_imu/quat", 1);
	imuboard_pub_temp_ = node_->advertise<std_msgs::Float64>("/qb_class_imu/temp", 1);

}

//-----------------------------------------------------
//                                            ~qb_class_imu
//-----------------------------------------------------

/*
/ *****************************************************
/ Descructor of qb_class_imu class
/ *****************************************************
/   parameters:
/   return:
/
*/


qb_class_imu::~qb_class_imu(){

}




//-----------------------------------------------------
//                                              readIMU
//-----------------------------------------------------

/*
/ *****************************************************
/ Read measurements of all IMUs
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/
bool qb_class_imu::readIMU(){
	
	for (int k = imuboard_chain_.size(); k--;){
	    imuboard_chain_[k]->getImuReadings();
		//qualcosa = imuboard_chain_[i]->imu_values;
		
		for (int i = 0; i < imuboard_chain_[k]->n_imu_; i++) {
			
			// std::cout<< i << " numero di imu " << imuboard_chain_[k]->n_imu_ << std::endl;
			// printf("IMU: %d\n", imuboard_chain_[k]->ids_[i]);
		
			// if (imuboard_chain_[k]->imu_table_[5*i + 0]){
			// 	printf("Accelerometer\n");
			// 	printf("%f, %f, %f\n", imuboard_chain_[k]->imu_values_[3*3*i], imuboard_chain_[k]->imu_values_[3*3*i+1], imuboard_chain_[k]->imu_values_[3*3*i+2]);
			// }
			// if (imuboard_chain_[k]->imu_table_[5*i + 1]){
			// 	printf("Gyroscope\n");
			// 	printf("%f, %f, %f\n", imuboard_chain_[k]->imu_values_[3*3*i+3], imuboard_chain_[k]->imu_values_[3*3*i+4], imuboard_chain_[k]->imu_values_[3*3*i+5]);
			// }
			// if (imuboard_chain_[k]->imu_table_[5*i + 2] ){
			// 	printf("Magnetometer\n");
			// 	printf("%f, %f, %f\n", imuboard_chain_[k]->imu_values_[3*3*i+6], imuboard_chain_[k]->imu_values_[3*3*i+7], imuboard_chain_[k]->imu_values_[3*3*i+8]);
			// }
			
			// printf("\n");
			if (imuboard_chain_[k]->imu_table_[5*i + 0])
				sendImuAcc();
			if (imuboard_chain_[k]->imu_table_[5*i + 1])
				sendImuGyro();
			if (imuboard_chain_[k]->imu_table_[5*i + 2] )
				sendImuMag();
		}
	
	}


}


//-----------------------------------------------------
//                                             spinOnce
//-----------------------------------------------------

/*
/ *****************************************************
/ Read all devices and set position if new ref. is
/ arrived.
/ *****************************************************
/   parameters:
/   return:
/
*/

void qb_class_imu::spinOnce(){

	qb_class::spinOnce();

	// Read measurementes of all IMUs
	readIMU();

	// send on topic
}

//-----------------------------------------------------
//                                                 spin
//-----------------------------------------------------

/*
/ *****************************************************
/ Read all devices and set position if new ref. is 
/ arrived.  
/ *****************************************************
/   parameters:
/   return:
/
*/

void qb_class_imu::spin(){

	// 1/step_time is the rate in Hz
	ros::Rate loop_rate(1.0 / step_time_);

	while(ros::ok()) {
		spinOnce();

		ros::spinOnce();

		loop_rate.sleep();
	}

}


//-----------------------------------------------------
//										    sendMeasIMU
//-----------------------------------------------------

/*
/ *****************************************************
/ Send function to post on topic hands measurement
/ *****************************************************
/   parameters:
/			closure - vector of measurment
/   return:
/
*/

void qb_class::sendHandMeas(vector<float> closure){

	// if (hand_chain_.empty())
	// 	return;

	qb_interface::handPos read_meas;

	// Fill structure
	read_meas.closure = closure;

	// Publish on right topic

	hand_pub.publish(read_meas);

}

