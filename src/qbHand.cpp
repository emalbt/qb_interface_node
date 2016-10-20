#include "qbHand.h"

//-----------------------------------------------------
//                                               qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbHand class
/ *****************************************************
/   parameters:
/               id, ID of the cube
/   return:
/
*/

qbHand::qbHand(int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                               qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbHand class
/ *****************************************************
/   parameters:
/       cs, communications channel
/       id, ID of the cube
/   return:
/
*/

qbHand::qbHand(comm_settings* cs, int id) : qbInterface(cs, id) {
    retrieveParams();
}


//-----------------------------------------------------
//                                              ~qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Destructor of qbHand class
/ *****************************************************
/   parameters:
/   return:
/
*/

qbHand::~qbHand() {}


//====================== OTHER FUNCTIONS =======================================

//-----------------------------------------------------
//                                           setPosPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set closure of the hand
/ *****************************************************
/   parameters:
/       - rateC, close hand rate
/   return:
/       [state]
/
*/

bool qbHand::setPosPerc(float rateC) {

    // Check input value

    if (rateC < 0.0)
        rateC = 0;

    if (rateC > 1.0)
        rateC = 1.0;

    short int curr_ref[2];

    // Motor of the hand position

    curr_ref[0] = rateC * POS_LIMIT_M1_[1] / 4.0 * axis_dir_;
    curr_ref[1] = rateC * POS_LIMIT_M1_[1] / 4.0 * axis_dir_;

    // Call cube position

    commSetInputs(cube_comm_, id_, curr_ref);

    return true;
}


//-----------------------------------------------------
//                                           getPosPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Get Measurement in angle of the hand closure
/ *****************************************************
/   parameters:
/   return:
/               value in radiants of hand closure
/
*/

bool qbHand::getPosPerc(float* angle) {

    // Get measurment in tic

    short int meas[3];

    if (!getMeas(meas))
        return false;

    // Trasform in perc and return measured value

    *angle = (((float) meas[0]) / DEG_TICK_MULTIPLIER) * (M_PI/180.0) / (POS_LIMIT_M1_[1] / 4) * axis_dir_;

    return true;
}

//-----------------------------------------------------
//                                       retrieveParams
//-----------------------------------------------------

/*
/ *****************************************************
/ Inizialize default values for the cube
/ *****************************************************
/   parameters:
/   return:
/
*/

void qbHand::retrieveParams() {

    int pos_limits[4];
    int PARAM_POS_LIMIT = 11; // Position limits index

    // Retrive informations

    for (int i = 0; i < NUM_OF_TRIALS; i++) {
        if(!commGetParamList(cube_comm_, id_, PARAM_POS_LIMIT, pos_limits, 4, 2, NULL)) {
            // Save limits
            POS_LIMIT_M1_[0] = pos_limits[0] / 2;
            POS_LIMIT_M1_[1] = pos_limits[1] / 2;
            POS_LIMIT_M2_[0] = pos_limits[2] / 2;
            POS_LIMIT_M2_[1] = pos_limits[3] / 2;

            return;
        }
    }

    std::cerr << "Unable to retrieve hand params. ID: " << id_ << std::endl;
}

//-----------------------------------------------------
//                                        getPosAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get positions and current for each motor, default 
/ input is set to radiants
/ *****************************************************
/   arguments:
/       - position, measured angle
/       - current, motor current
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbHand::getPosAndCurr(float* position, float* current, angular_unit unit) {
     
    short int meas[3], curr[2];     

    // Get position measurements and currents
    if(!getMeasAndCurr(meas, curr))
        return false;

    // Return position in the right unit

    if (unit == DEG)
        position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER);
    else
        if (unit == RAD)
            position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
        else
            position[0] = (float) meas[0];

    // save Currents
    current[0] = (float) curr[0];
    current[1] = (float) curr[1];

    return true;
}