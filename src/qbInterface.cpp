#include <iostream>

#include "qbInterface.h"



//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbInterface class
/ *****************************************************
/   argument:
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir = 1;
    else
        axis_dir = -1;

    ID = abs(id);

    cube_comm = NULL;
}


//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbInterface class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(comm_settings* cs, const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir = 1;
    else
        axis_dir = -1;

    ID = abs(id);

    cube_comm = cs;
}


//-----------------------------------------------------
//                                         ~qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbInterface class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbInterface::~qbInterface() {

    close();

}


//======================= OTHER FUNCTIONS ======================================

//-----------------------------------------------------
//                                                 open
//-----------------------------------------------------

/*
/ *****************************************************
/ Open serial communication with cube
/ *****************************************************
/   arguments:
/       port, the communication port
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::open(const char* port) {

    // Check Connection State

    if(cube_comm != NULL) {
        cerr << "WARNING: Port already opened" << endl;
        return false;
    }

    cube_comm = new comm_settings;

    // Establish serial connection

    openRS485(cube_comm, port);

    if (cube_comm->file_handle == INVALID_HANDLE_VALUE) {
        cerr << "ERROR: Unable to open port" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                                close
//-----------------------------------------------------

/*
/ *****************************************************
/ Close serial communication
/ *****************************************************
/   arguments:
/   return:
/
*/

void qbInterface::close() {

    deactivate();

    // close commnication
    if (cube_comm != NULL) {
        closeRS485(cube_comm);
        delete cube_comm;
        cube_comm = NULL;
    }
}


//-----------------------------------------------------
//                                             activate
//-----------------------------------------------------

/*
/ *****************************************************
/ Activate the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::activate() {

    // Check connection status
    if (cube_comm == NULL) {
        cerr << "ERROR:[qbInterface activate()] Port not opened" << endl;
        return false;
    }

    // Activate board
    commActivate(cube_comm, ID, 1);

    // Wait setting time

    usleep(1000);

    // Check if board is active
    char status = 0;
    commGetActivate(cube_comm, ID, &status);

    // Check status
    if (!status){
        cerr << "Unable to activate ID: " << ID <<  endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                           deactivate
//-----------------------------------------------------

/*
/ *****************************************************
/ Active the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::deactivate() {

    // Check connection status
    if (cube_comm == NULL){
        //cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Deactivate board
    commActivate(cube_comm, ID, 0);

    // Check if the board is inactive
    char status;
    commGetActivate(cube_comm, ID, &status);

    if (status) {
        cerr << "Unable to deactivate" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                              getMeas
//-----------------------------------------------------

/*
/ *****************************************************
/ Get measurement of positions [1, 2, 3]  in ticks
/ *****************************************************
/   arguments:
/       - meas, 3 elements array pointer for measurements
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::getMeas(short int* meas) {

    if (cube_comm == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    if (commGetMeasurements(cube_comm, ID, meas))
        return false;

    // Axis direction
    
    meas[0] *= axis_dir;
    meas[1] *= axis_dir;
    meas[2] *= axis_dir;

    return true;
}


//-----------------------------------------------------
//                                            setInputs
//-----------------------------------------------------

/*
/ *****************************************************
/ Set board inputs in ticks [1, 2]
/ *****************************************************
/   arguments:
/       - inputs, 2 elements array pointer for inputs
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::setInputs(short int* inputs) {

    if (cube_comm == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Axis direction

    inputs[0] *= axis_dir;
    inputs[1] *= axis_dir;

    commSetInputs(cube_comm, ID, inputs);

    return true;
}

//-----------------------------------------------------
//                                                getID
//-----------------------------------------------------

/*
/ *****************************************************
/ Getter for variable ID
/ *****************************************************
/   arguments:
/   return:
/       ID
/
*/

int qbInterface::getID() {

    return ID;
}



/* END OF FILE */
