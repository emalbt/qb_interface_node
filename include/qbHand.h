#ifndef QBHAND_H
#define QBHAND_H

#include <qbInterface.h>

#define NUM_OF_TRIALS 5


class qbHand : public qbInterface
{
    public:
        //------------------------------ qbHand
        // Costructor of qbHand class
        qbHand(int);

        //------------------------------ qbHand
        // External costructor of qbHand class
        qbHand(comm_settings*, int);

        //------------------------------ ~qbHand
        // Destructor of qbHand class
        ~qbHand();


        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ setPosition
        // Set closure of the hand
        bool setPosPerc(float);

        //------------------------------ getAngle
        // Get Measurement in angle of the hand closure
        bool getPosPerc(float*);

        //------------------------------ getAngle
        // Get Position and Current of the hand closure
        bool getPosAndCurr(float*, float*, angular_unit);

        //------------------------------ init
        // Inizialize default values for the cube
        void retrieveParams();

    protected:

        // Position limit for motors
        int POS_LIMIT_M1_[2], POS_LIMIT_M2_[2];

    private:


};

#endif // QBHAND_H