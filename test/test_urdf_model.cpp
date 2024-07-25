#include "urdf_model.h"

int main() {
    URDFModel model;
    std::string urdfFilePath = "/data/ros/cdu_cimpl/prbt_description.urdf"; // Replace with your URDF file path

    if (model.loadURDF(urdfFilePath)) {
        std::cout << "Successfully loaded URDF file: " << urdfFilePath << std::endl;

        std::map<std::string, double> jointAngles = {
            {"prbt_joint_1", 0.0},
            {"prbt_joint_2", 1.0},
            {"prbt_joint_3", 2.0},
            {"prbt_joint_4", 0.0},
            {"prbt_joint_5", 0.0},
            {"prbt_joint_6", 0.0}
        };
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates();
        model.printModel();
        model.printCollisionCoordinates();
    } else {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        return -1;
    }

    return 0;
}