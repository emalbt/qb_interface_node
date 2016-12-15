
#include "qb_class_imu.h"

//-----------------------------------------------------
//                                         qb_class_imu
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qb_class_imu class
/ *****************************************************
/   parameters:
/				port - # of IMUs connected
/					   
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
	imuboard_pub_acc_  = node_->advertise<qb_interface::inertialSensorArray>("/qb_class_imu/acc", 1);
	imuboard_pub_gyro_ = node_->advertise<qb_interface::inertialSensorArray>("/qb_class_imu/gyro", 1);
	imuboard_pub_mag_  = node_->advertise<qb_interface::inertialSensorArray>("/qb_class_imu/mag", 1);
	// imuboard_pub_quat_ = node_->advertise<qb_interface::inertialSensor>("/qb_class_imu/quat", 1);
	// imuboard_pub_temp_ = node_->advertise<qb_interface::inertialSensor>("/qb_class_imu/temp", 1);

	Acc_.resize(imuboard_chain_[0]->n_imu_,3);
	Acc_old_.resize(imuboard_chain_[0]->n_imu_,3);

	Acc_.setZero();
	Acc_old_.setZero();
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
	qb_interface::inertialSensor tmp_acc, tmp_gyro, tmp_mag;
	qb_interface::inertialSensorArray acc, gyro, mag;


	for (int k = imuboard_chain_.size(); k--;){
	    imuboard_chain_[k]->getImuReadings();
		
		for (int i = 0; i < imuboard_chain_[k]->n_imu_; i++) 
		{
			
			// printf("IMU: %d\n", imuboard_chain_[k]->ids_[i]);
		
			if (imuboard_chain_[k]->imu_table_[5*i + 0])
			{
				tmp_acc.id = imuboard_chain_[k]->ids_[i];
				tmp_acc.x  = imuboard_chain_[k]->imu_values_[3*3*i];
				tmp_acc.y  = imuboard_chain_[k]->imu_values_[3*3*i+1];
				tmp_acc.z  = imuboard_chain_[k]->imu_values_[3*3*i+2];
				acc.m.push_back(tmp_acc);

				Acc_(i,0) = tmp_acc.x; 
				Acc_(i,1) = tmp_acc.y; 
				Acc_(i,2) = tmp_acc.z; 
			}
			if (imuboard_chain_[k]->imu_table_[5*i + 1])
			{
				tmp_gyro.id = imuboard_chain_[k]->ids_[i];
				tmp_gyro.x  = imuboard_chain_[k]->imu_values_[3*3*i+3];
				tmp_gyro.y  = imuboard_chain_[k]->imu_values_[3*3*i+4];
				tmp_gyro.z  = imuboard_chain_[k]->imu_values_[3*3*i+5];
				gyro.m.push_back(tmp_gyro);
			}

			if (imuboard_chain_[k]->imu_table_[5*i + 2] )
			{
				tmp_mag.id = imuboard_chain_[k]->ids_[i];
				tmp_mag.x  = imuboard_chain_[k]->imu_values_[3*3*i+6];
				tmp_mag.y  = imuboard_chain_[k]->imu_values_[3*3*i+7];
				tmp_mag.z  = imuboard_chain_[k]->imu_values_[3*3*i+8];
				mag.m.push_back(tmp_mag);
			}

			// TO DO ----->>>> pub(quat), pub(temp)
			
			// verify if this usleep is needed
			usleep(0.5);
		}

		if ((Acc_old_ - Acc_).sum() != 0)
	 	{
			imuboard_pub_acc_.publish(acc);
			imuboard_pub_gyro_.publish(gyro);			
			imuboard_pub_mag_.publish(mag);	
	 	}

	 	Acc_old_ = Acc_;
		
	
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

	// Read measurementes of all IMUs and send them on topics
	readIMU();

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