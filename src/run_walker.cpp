#include "Walker.h"

int main(int argc, char **argv) {
    // Initiate new ROS node named "stopper"
    ros::init(argc, argv, "walker");

    // Create new stopper object
    Walker stopper;

    // Start the movement
    stopper.startMoving();

    return 0;
}
