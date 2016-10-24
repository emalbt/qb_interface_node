#include "qbImuBoard.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(const int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer to the cube
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(comm_settings* cs, const int id) : qbInterface(cs, id) {
	
	initImuBoard();
	
}


//-----------------------------------------------------
//                                          ~qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbImuBoard class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbImuBoard::~qbImuBoard() {}


//========================== OTHER FUNCTIONS ===================================


//-----------------------------------------------------
//                                         initImuBoard
//-----------------------------------------------------
void qbImuBoard::initImuBoard() {

	uint8_t aux_string[2000];
	uint8_t PARAM_SLOT_BYTES = 50;
	int num_of_params;
	
	commGetParamList(cube_comm_, id_, 0, NULL, 0, 0, aux_string);
	
	num_of_params = aux_string[5];
	
	//aux_string[6] <-> packet_data[2] on the firmware
	n_imu_ = aux_string[1*PARAM_SLOT_BYTES + 8];
	printf("Number of connected IMUs: %d\n", n_imu_);
	
	ids_ = (uint8_t *) calloc(n_imu_, sizeof(uint8_t));
	for (int i=0; i< n_imu_; i++){
		ids_[i] = aux_string[2*PARAM_SLOT_BYTES + 8 + i];
	}
	
	// Retrieve magnetometer calibration parameters
	mag_cal_ = (uint8_t *) calloc(n_imu_, 3*sizeof(uint8_t));
	for (int i=0; i< n_imu_; i++){
		mag_cal_[3*i + 0] = aux_string[3*PARAM_SLOT_BYTES + 8 + 3*i];
		mag_cal_[3*i + 1] = aux_string[3*PARAM_SLOT_BYTES + 9 + 3*i];
		mag_cal_[3*i + 2] = aux_string[3*PARAM_SLOT_BYTES + 10 + 3*i];
		printf("MAG PARAM: %d %d %d\n", mag_cal_[3*i + 0], mag_cal_[3*i + 1], mag_cal_[3*i + 2]);
		
	}
	
	imu_table_ = (uint8_t *) calloc(n_imu_, 5*sizeof(uint8_t));
	for (int i=0; i< n_imu_; i++){
		imu_table_[5*i + 0] = aux_string[4*PARAM_SLOT_BYTES + 8 + 50*i];
		imu_table_[5*i + 1] = aux_string[4*PARAM_SLOT_BYTES + 9 + 50*i];
		imu_table_[5*i + 2] = aux_string[4*PARAM_SLOT_BYTES + 10 + 50*i];
		printf("ID: %d  - %d, %d, %d, %d, %d\n", ids_[i], imu_table_[5*i + 0], imu_table_[5*i + 1], imu_table_[5*i + 2], imu_table_[5*i + 3], imu_table_[5*i + 4]);
		
	}
	
	// Imu values is a 3 sensors x 3 axes x n_imu_ values
	imu_values_ = (float *) calloc(n_imu_, 3*3*sizeof(float));
		
}



//-----------------------------------------------------
//                                  	 getImuReadings
//-----------------------------------------------------
void qbImuBoard::getImuReadings() {
	
	commGetImuReadings(cube_comm_, id_, imu_table_, mag_cal_, n_imu_, imu_values_);
			
	for (int i = 0; i < n_imu_; i++) {
		
		printf("IMU: %d\n", ids_[i]);
	
		if (imu_table_[5*i + 0]){
			printf("Accelerometer\n");
			printf("%f, %f, %f\n", imu_values_[3*3*i], imu_values_[3*3*i+1], imu_values_[3*3*i+2]);
		}
		if (imu_table_[5*i + 1]){
			printf("Gyroscope\n");
			printf("%f, %f, %f\n", imu_values_[3*3*i+3], imu_values_[3*3*i+4], imu_values_[3*3*i+5]);
		}
		if (imu_table_[5*i + 2] ){
			printf("Magnetometer\n");
			printf("%f, %f, %f\n", imu_values_[3*3*i+6], imu_values_[3*3*i+7], imu_values_[3*3*i+8]);
		}
		
		printf("\n");
	}
	
}

/* END OF FILE */