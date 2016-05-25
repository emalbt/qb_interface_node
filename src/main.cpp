#include "qb_class.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "qb_interface_node");

    qb_class qb_int;

    cout << "[INFO] Start to spin qb_interface_node" << endl;

    qb_int.spin();

	return 0;
}