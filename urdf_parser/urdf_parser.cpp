#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <Eigen/Dense>

using namespace Eigen;

// Function to convert KDL Frame to Eigen Isometry3d
Isometry3d KDLToEigen(const KDL::Frame& frame) {
    Isometry3d transform = Isometry3d::Identity();
    transform.translation() << frame.p.data[0], frame.p.data[1], frame.p.data[2];

    Matrix3d rotation;
    rotation << frame.M.data[0], frame.M.data[1], frame.M.data[2],
                frame.M.data[3], frame.M.data[4], frame.M.data[5],
                frame.M.data[6], frame.M.data[7], frame.M.data[8];

    transform.linear() = rotation;
    return transform;
}

void printLinkTransforms(const KDL::Chain& chain, const std::vector<double>& joint_angles, urdf::Model& model) {
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray joint_positions(chain.getNrOfJoints());
    
    for (size_t i = 0; i < joint_angles.size(); ++i) {
        joint_positions(i) = joint_angles[i];
    }
    
    KDL::Frame frame;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        fk_solver.JntToCart(joint_positions, frame, i+1);
        Isometry3d transform = KDLToEigen(frame);
        std::cout << "Link: " << chain.getSegment(i).getName() << std::endl;
        std::cout << transform.matrix() << std::endl;

        // Print collision information
        std::shared_ptr<const urdf::Link> urdf_link = model.getLink(chain.getSegment(i).getName());
        if (urdf_link) {
            for (const auto& collision : urdf_link->collision_array) {
                if (collision) {
                    Isometry3d collision_transform = transform *
                        Translation3d(collision->origin.position.x, collision->origin.position.y, collision->origin.position.z) *
                        AngleAxisd(Quaterniond(collision->origin.rotation.w,
                                               collision->origin.rotation.x,
                                               collision->origin.rotation.y,
                                               collision->origin.rotation.z));
                    Vector3d center = collision_transform.translation();
                    std::cout << "  Collision center position: (" << center.x() << ", " << center.y() << ", " << center.z() << ")" << std::endl;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    urdf::Model model;
    std::string urdf_file_path = "/data/ros/cdu_cimpl/urdf_load/prbt_description.urdf";
    
    // Load URDF model from file
    std::ifstream urdf_file(urdf_file_path);
    if (!urdf_file) {
        std::cerr << "Failed to open URDF file: " << urdf_file_path << std::endl;
        return -1;
    }
    std::stringstream urdf_string;
    urdf_string << urdf_file.rdbuf();
    urdf_file.close();
    
    if (!model.initString(urdf_string.str())) {
        std::cerr << "Failed to parse URDF string" << std::endl;
        return -1;
    }
    
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        std::cerr << "Failed to convert URDF to KDL tree" << std::endl;
        return -1;
    }

    KDL::Chain chain;
    if (!tree.getChain("prbt_base_link", "prbt_tcp", chain)) { // 修改根链接和末端链接名称
        std::cerr << "Failed to get KDL chain from tree" << std::endl;
        return -1;
    }

    // Set joint angles
    std::map<std::string, double> joint_angles_map = {
        {"prbt_joint_1", 1.0},
        {"prbt_joint_2", 0.0},
        {"prbt_joint_3", 2.0},
        {"prbt_joint_4", 3.0},
        {"prbt_joint_5", 1.0},
        {"prbt_joint_6", 0.0}
    };

    std::vector<double> joint_angles(chain.getNrOfJoints(), 0.0);
    for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
        const std::string& joint_name = chain.getSegment(i).getJoint().getName();
        if (joint_angles_map.find(joint_name) != joint_angles_map.end()) {
            joint_angles[i] = joint_angles_map[joint_name];
        }
    }

    printLinkTransforms(chain, joint_angles, model);

    return 0;
}
