#include "qbCube.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

//-----------------------------------------------------
//                                               qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbCube class
/ *****************************************************
/   argument:
/       - id, ID of the cube
/   return:
/
*/

qbCube::qbCube(const int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                               qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbCube class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer to the cube
/       - id, ID of the cube
/   return:
/
*/

qbCube::qbCube(comm_settings* cs, const int id) : qbInterface(cs, id) {}


//-----------------------------------------------------
//                                              ~qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbCube class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbCube::~qbCube() {}


//========================== OTHER FUNCTIONS ===================================


//-----------------------------------------------------
//                                      setPosAndPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness preset of the cube, default
/ input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPreset, stiffness preset
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

void qbCube::setPosAndPreset(float position, float stiffPreset, angular_unit unit) {

    // Check input values consistence

    if (stiffPreset > DEFAULT_STIFFNESS) {

        stiffPreset = DEFAULT_STIFFNESS;
        cerr << "WARNING: Preset saturated to " << stiffPreset << endl;

    } else if (stiffPreset < 0) {

        stiffPreset = 0;
        cerr << "WARNING: Preset saturated to " << stiffPreset << endl;

    }

    // Convert position in 'deg' if needed

    if (unit == RAD)
        position *= (180.0 / M_PI);

    // Check Max Position available

    if (position > (DEFAULT_SUP_LIMIT / DEG_TICK_MULTIPLIER) - stiffPreset) {

        position = (DEFAULT_SUP_LIMIT / DEG_TICK_MULTIPLIER) - stiffPreset;
        cerr << "WARNING: Position saturated to " << position << endl;

    } else if (position < (DEFAULT_INF_LIMIT / DEG_TICK_MULTIPLIER) + stiffPreset) {

            position = (DEFAULT_INF_LIMIT / DEG_TICK_MULTIPLIER) + stiffPreset;
            cerr << "WARNING: Position saturated to " << position << endl;
    }

    short int pos, sPreset;
    short int curr_ref[2];

    pos = position * DEG_TICK_MULTIPLIER;
    sPreset = stiffPreset * DEG_TICK_MULTIPLIER;

    // Set position for the 2 motors of the cube

    curr_ref[0] = pos - sPreset;
    curr_ref[1] = pos + sPreset;

    // Call API function

    commSetInputs(cube_comm, ID, curr_ref);
}


//-----------------------------------------------------
//                                      getPosAndPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Get position and stiffness preset of the cube, default
/ input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPreset, stiffness preset
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPosAndPreset(float* position, float* preset, angular_unit unit) {
 
    short int meas[3];

    // Get measurements
    if(!getMeas(meas))
        return false;

    // Return position in the right unit

    if (unit == DEG)
        *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER);
    else
        *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);

    // Compute preset value
    *preset = ((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER;

    return true;
}


//-----------------------------------------------------
//                                  setPosAndPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness percentage of the cube,
/ default input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPerc, stiffness percentage; range: [0 - 32767]
/       - unit, measurement unit for position [rad or deg]
/
/   return:
/       true  on success
/       false on failure
/
*/

void qbCube::setPosAndPresetPerc(float position, float stiffPerc, angular_unit unit) {

    // Check input values consistence

    if (stiffPerc > 100) {

        stiffPerc = 100;
        cerr << "WARNING: Stiffness percentage saturated to " << stiffPerc << endl;

    } else if (stiffPerc < 0) {

        stiffPerc = 0;
        cerr << "WARNING: Stiffness percentage saturated to " << stiffPerc << endl;

    }

    // Convert position in 'deg' if needed

    if (unit == RAD) {

        position *= (180.0 / M_PI);

    }


    // XXX TODO: Check Max Position available

    short int curr_ref[2];

    curr_ref[0] = (short int)(position * DEG_TICK_MULTIPLIER);
    curr_ref[1] = (short int)((stiffPerc * 32767.0) / 100.0);

    // Call API function

    commSetInputs(cube_comm, ID, curr_ref);
}


//-----------------------------------------------------
//                                  getPosAndPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness percentage of the cube,
/ default input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPerc, stiffness percentage; range: [0 - 32767]
/       - unit, measurement unit for position [rad or deg]
/
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPosAndPresetPerc(float* position, float* stiffPerc, angular_unit unit) {

    return true;
}

//-----------------------------------------------------
//                                               getPos
//-----------------------------------------------------

/*
/ *****************************************************
/ Get position angle in the chosen unit of measure
/ *****************************************************
/   arguments:
/       - angle, getted position in angle
/       - unit, unit of measure [rad o deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPos(float* angle, angular_unit unit) {

    short int meas[3];

    // Get measurements
    if(!getMeas(meas))
        return false;

    // Return position in the right unit

    if (unit == DEG)
        *angle = (((float) meas[2]) / DEG_TICK_MULTIPLIER);
    else
        *angle = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);

    return true;
}

//-----------------------------------------------------
//                                            getPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Get preset of the cube
/ *****************************************************
/   parameters:
/   return:
/       - preset value
/
*/

bool qbCube::getPreset(float* preset) {

    short int meas[3];

    // Get position
    if(!getMeas(meas))
        return false;

    // Compute preset value
    *preset = ((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER;

    return true;
}


//-----------------------------------------------------
//                                        getPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Get preset of the cube
/ *****************************************************
/   parameters:
/   return:
/       - preset value
/
*/

bool qbCube::getPresetPerc(float* preset) {

    // in realta non so come ottenerla

    return true;
}

/* END OF FILE */