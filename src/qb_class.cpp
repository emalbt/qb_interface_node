
#include "qb_class.h"

//-----------------------------------------------------
//                                         qb_class
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qb_class class
/ *****************************************************
/   parameters:
/				port - usb port tfo activate 
/					   communication
/   return:
/
*/

qb_class::qb_class(){

	// Variables to get param 
	vector<int> ID_cube, ID_hand;
	int unit;

	string port;
	string aux;

	// Initialize ROS Node

	node_ = new ros::NodeHandle("qb_interface_node_");

	// Get param from roslaunch or yaml file

	node_->param("/eq_preset", flagCMD_type_, true);
	node_->param("/hand_perc", flag_HCMD_type_, false);
	node_->param<double>("/step_time_", step_time_, 0.002);
	node_->param<string>("/port", port, "/dev/ttyUSB0");

	node_->searchParam("/IDcubes", aux);
	node_->getParam(aux, ID_cube);

	node_->searchParam("/IDhands", aux);
	node_->getParam(aux, ID_hand);

	node_->param<string>("/unit", aux, "DEG");

	// Choose Right unit of measurement
	
	if(!strcmp(aux.c_str(), "DEG") || !strcmp(aux.c_str(), "deg") || !strcmp(aux.c_str(), "Deg"))
		meas_unit_ = DEG;
	else{
		if(!strcmp(aux.c_str(), "RAD") || !strcmp(aux.c_str(), "rad") || !strcmp(aux.c_str(), "Rad"))
			meas_unit_ = RAD;
		else{
			if(!strcmp(aux.c_str(), "TICK") || !strcmp(aux.c_str(), "tick") || !strcmp(aux.c_str(), "Tick"))	
				meas_unit_ = TICK;
			else{
				cerr << "[WARNING] Unit of measurement is not correctly set, degree default enable." << endl;
				meas_unit_ = DEG;
			}
		}
	}

	qb_comm_ = NULL;

	// Open port 

	if (!open(port.c_str())){
		qb_comm_ = NULL;

		cout << "[ERROR] USB " << port << " was not open correctly." << endl;
		return;
	}


	// Create cubes and hands chain from ID read

	// Add Cubes

	qbCube* tmp_cube;

    // Allocate object qbCube, one for each cube in cubeBuf

    for (int i = 0; i < ID_cube.size(); ++i) {
        tmp_cube = new qbCube(qb_comm_, ID_cube[i]);
        
       	// IF an error is find
        if (tmp_cube == NULL){
        	cout << "[ERORR] Unable to allocate space for cube structure." << endl;
            return;
        }

        cube_chain_.push_back(tmp_cube);
    } 

	// Add Hands
	
	qbHand* tmp_hand;

    // Allocate object qbCube, one for each cube in cubeBuf

    for (int i = 0; i < ID_hand.size(); ++i) {

        tmp_hand = new qbHand(qb_comm_, ID_hand[i]);
        
       	// IF an error is find
        if (tmp_hand == NULL){
        	cout << "[ERORR] Unable to allocate space for cube structure." << endl;
            return;
        }

        hand_chain_.push_back(tmp_hand);
    } 

	// Activate
	if (!activate())
		cout << "[ERROR] Some qbDevice are not correctly activated." << endl;

	// Initialize publisher and subscriber

	if (!hand_chain_.empty()){

		// Subscriber initialize	
 		hand_sub = node_->subscribe("/qb_class/hand_ref", 1000, &qb_class::handRefCallback, this); 		

		// Publisher initialize

		// Outside command publisher, self-open but not internally used topic
		handRef_pub = node_->advertise<qb_interface::handRef>("/qb_class/hand_ref", 1000);

		// Publisher to publish new read positions
		hand_pub = node_->advertise<qb_interface::handPos>("/qb_class/hand_measurement", 1);
	}

	if (!cube_chain_.empty()){

		// Subscriber initialize
		cube_sub = node_->subscribe("/qb_class/cube_ref", 1000, &qb_class::cubeRefCallback, this);
		// Publisher initialize

		// Outside command publisher, self-open but not internally used topic
		cubeRef_pub = node_->advertise<qb_interface::cubeRef>("/qb_class/cube_ref", 1000);

		// Publisher to publish new read positions
		if (flagCMD_type_ == EQ_PRESET)
			cube_pub = node_->advertise<qb_interface::cubeEq_Preset>("/qb_class/cube_measurement", 1);
		else
			cube_pub = node_->advertise<qb_interface::cubePos>("/qb_class/cube_measurement", 1);
	}
}

//-----------------------------------------------------
//                                            ~qb_class
//-----------------------------------------------------

/*
/ *****************************************************
/ Descructor of qb_class class
/ *****************************************************
/   parameters:
/   return:
/
*/


qb_class::~qb_class(){

	// Deactivate
	if (!deactivate())
		cout << "[ERROR] Some qbDevices was not correctly deactivated." << endl;

	// Close port
	close();

	qb_comm_ = NULL;

}

//-----------------------------------------------------
//                                                 open
//-----------------------------------------------------

/*
/ *****************************************************
/ Open connection with all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_class::open(const char* port) {

    // Check connection state
    if (qb_comm_ != NULL)
        return false;

    qb_comm_ = new comm_settings;

    // Call open communication function

    openRS485(qb_comm_, port);

    if (qb_comm_->file_handle == INVALID_HANDLE_VALUE)
   		return false;
    

    // Set all cubes communication

    for (int i = cube_chain_.size(); i--;)
            cube_chain_[i]->cube_comm = qb_comm_;

    // Set all hands communication

    for (int i = hand_chain_.size(); i--;)
            hand_chain_[i]->cube_comm = qb_comm_;


    return true;
}

//-----------------------------------------------------
//                                                close
//-----------------------------------------------------

/*
/ *****************************************************
/ Close connection of all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_class::close() {

    // Check connection status

    if (qb_comm_ == NULL)
        return false;

    // Call ccommunication function

    closeRS485(qb_comm_);

    delete qb_comm_;

    qb_comm_ = NULL;

    // Set all cubes communication

    for (int i = cube_chain_.size(); i--;)
		cube_chain_[i]->cube_comm = qb_comm_;

    // Set all hands communication

    for (int i = hand_chain_.size(); i--;)
		hand_chain_[i]->cube_comm = qb_comm_;

    return true;
}


//-----------------------------------------------------
//                                             activate
//-----------------------------------------------------

/*
/ *****************************************************
/ Close connection of all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_class::activate() {

    bool status = true;

    // Check connection status
    if (qb_comm_ == NULL)
        return false;

    // Activate all cubes

    int err_count = 0;

	for (int i = cube_chain_.size(); i--;){

        while(!(cube_chain_[i]->activate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    for (int i = hand_chain_.size(); i--;){

        while(!(hand_chain_[i]->activate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    return status;
}

//-----------------------------------------------------
//                                           deactivate
//-----------------------------------------------------

/*
/ *****************************************************
/ Close connection of all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_class::deactivate() {

    bool status = true;

    // Check connection status
    if (qb_comm_ == NULL)
        return false;

    int err_count = 0;

    // Deactivate all cubes

	for (int i = cube_chain_.size(); i--;){

        while(!(cube_chain_[i]->deactivate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    for (int i = hand_chain_.size(); i--;){

        while(!(hand_chain_[i]->deactivate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    return status;
}

//-----------------------------------------------------
//                                                 read
//-----------------------------------------------------

/*
/ *****************************************************
/ Read position of all motors
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/

bool qb_class::read(){

	bool status = true;
	short int meas[3];


	vector<float> closure;

	// Read position from cubes

	if (flagCMD_type_ != EQ_PRESET){

		vector<float> pos_1;
		vector<float> pos_2;
		vector<float> pos_L;

	    for (int i = 0; i < cube_chain_.size(); ++i){
	    	if (!cube_chain_[i]->getMeas(meas)){
				cerr << "[WARNING] Unable to retrieve measurements of cube: " << cube_chain_[i]->getID() << endl;
	            status = false;	

	           	pos_1.push_back(NAN);
	    		pos_2.push_back(NAN);
	    		pos_L.push_back(NAN);
	    	}
	    	else{
	    		pos_1.push_back(meas[0]);
	    		pos_2.push_back(meas[1]);
	    		pos_L.push_back(meas[2]);
	    	}
	    }

	    // Send data on topic

	    send(pos_1, pos_2, pos_L);

	}else{

	    float position, pSet;

		vector<float> pos;
		vector<float> preset;

	    for (int i = 0; i < cube_chain_.size(); ++i){
	        if(!cube_chain_[i]->getPosAndPreset(&position, &pSet, meas_unit_)) {
	            cerr << "[WARNING] Unable to retrieve measurements of cube: " << cube_chain_[i]->getID() << endl;
	            status = false;

				pos.push_back(NAN);
	    		preset.push_back(NAN);
	        }else{
		    	pos.push_back(position);
		    	preset.push_back(pSet);
		    }
	    }

	    // Send data on topic

	    send(pos, preset);
	}

	// Read Hand position in TICK

	for (int i = 0; i < hand_chain_.size(); ++i){
    	if (!hand_chain_[i]->getMeas(meas)){
    		cerr << "[WARNING] Unable to retrieve measurements of hand: " << hand_chain_[i]->getID() << endl;
            status = false;

            closure.push_back(NAN);
        }else
        	closure.push_back(meas[0]);
    }

    // Send Data hands
	send(closure);
  
    return true;
}

//-----------------------------------------------------
//                                            moveCubes
//-----------------------------------------------------

/*
/ *****************************************************
/ Move the chain to desiderated position and preset or
/ pos_1 and pos_2, in Percent mode valeus are from 0.0
/ to 1.0
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_class::move() {

	bool status = true;

    // Check if new position are received
    if ((p_1_.size() != 0) && (p_2_.size() != 0)){

		// Cubes
	    if (flagCMD_type_ == EQ_PRESET){

		    	// Command cubes in Equilibrium Position and Preset
		    	for (int i = cube_chain_.size(); i--;)
		    		cube_chain_[i]->setPosAndPreset(p_1_[i], p_2_[i], meas_unit_);
		    	
	    }else{

	    	// Command cubes in Pos1 and Pos2
	    	short int meas[2];

	    	for (int i = cube_chain_.size(); i--;){
	    		meas[0] = (short int) p_1_[i];
	    		meas[1] = (short int) p_2_[i];

	    		cube_chain_[i]->setInputs(meas);
	    	}
	    }
	}
    // Hands

    // Check if new position are received
    if (pos_.size() != 0){

	    if (flag_HCMD_type_ == PERC){

	    	// Command cubes in percents

	    	for (int i = hand_chain_.size(); i--;)
	    		hand_chain_[i]->setPosPerc(pos_[i]);

	    }else{

	    	// Command cubes in Pos1 and Pos2, TICK
	    	short int meas[2];

	    	for (int i = hand_chain_.size(); i--;){
	    		meas[0] = (short int) pos_[i];
	    		meas[1] = (short int) pos_[i];

	    		hand_chain_[i]->setInputs(meas);
	    	}
	    }
	}

    return true;
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

void qb_class::spin(){

	if (qb_comm_ == NULL){
		cout << "[ERROR] Connection error in spin() function." << endl;
		return;
	}

	// 1/step_time is the rate in Hz 
	ros::Rate loop_rate(1.0 / step_time_);

	while(ros::ok()){

		// Read positions of all devices

		read();

		// Set Position of all devices

		move();

		// Reset Referiments
		p_1_.clear();
		p_2_.clear();
		pos_.clear();

		// Ros spin

		ros::spinOnce();

		// Wait residual time

		loop_rate.sleep();
	}
}

//-----------------------------------------------------
//                                      handRefCallback
//-----------------------------------------------------

/*
/ *****************************************************
/ Callback function to get referiments of hands from
/ outside.
/ *****************************************************
/   parameters:
/			msg - message received from topic
/   return:
/
*/

void qb_class::handRefCallback(const qb_interface::handRef::ConstPtr& msg){
	pos_ = msg->closure;
}

//-----------------------------------------------------
//                                     closeRefCallback
//-----------------------------------------------------

/*
/ *****************************************************
/ Callback function to get referiments of cubes from
/ outside.
/ *****************************************************
/   parameters:
/			msg - message received from topic
/   return:
/
*/

void qb_class::cubeRefCallback(const qb_interface::cubeRef::ConstPtr& msg){
	p_1_ = msg->p_1;
	p_2_ = msg->p_2;
}

//-----------------------------------------------------
//                                     			   send
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

void qb_class::send(vector<float> closure){

	if (hand_chain_.empty())
		return;

	qb_interface::handPos read_meas;

	read_meas.closure = closure;

	// Publish on right topic

	hand_pub.publish(read_meas);

}

//-----------------------------------------------------
//                                     			   send
//-----------------------------------------------------

/*
/ *****************************************************
/ Send function to post on topic cubes measurements in
/ equilibrium and preset mode
/ *****************************************************
/   parameters:
/			eq - vector of equilibrium measurments
/			preset - vector of preset measurements
/   return:
/
*/

void qb_class::send(vector<float> eq, vector<float> preset){

	if (cube_chain_.empty())
		return;

	qb_interface::cubeEq_Preset read_meas;

	read_meas.eq = eq;
	read_meas.preset = preset;

	// Publish on right topic
	cube_pub.publish(read_meas);
}

//-----------------------------------------------------
//                                     			   send
//-----------------------------------------------------

/*
/ *****************************************************
/ Send function to post on topic cubes measurements in
/ pos1/pos2/posL mode
/ *****************************************************
/   parameters:
/			pos_1 - position of the first motor
/			pos_2 - position of the second motor
/			pos_L - position of the shalf
/   return:
/
*/

void qb_class::send(vector<float> pos_1, vector<float> pos_2, vector<float> pos_L){

	if (cube_chain_.empty())		
		return;

	qb_interface::cubePos read_meas;

	read_meas.p_1 = pos_1;
	read_meas.p_2 = pos_2;
	read_meas.p_L = pos_L;

	// Publish on right topic

	cube_pub.publish(read_meas);
}