#ifndef URDF_MODEL_H
#define URDF_MODEL_H

#include <iostream>
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>

#include "aabb.h"
#include "obb.h"

using namespace tinyxml2;

struct CollisionGeom {
    enum ShapeType { BOX, SPHERE, CYLINDER, MESH };
    ShapeType type;
    Eigen::Vector3d collisionPositions;
    Eigen::Quaterniond collisionRotations;
    std::shared_ptr<bodies::Body> body;
    AABB<double> aabb;
    OBB<double> obb;
};

struct Link {
    std::string name;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::vector<CollisionGeom> collisions;
};

struct Joint {
    std::string name;
    std::string parent;
    std::string child;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::string type;
    Eigen::Vector3d axis;
};

class URDFModel{

public:

    std::map<std::string, Link> links;
    std::vector<Joint> joints;
    std::map<std::string, double> jointAngles;
    std::map<std::string, Link> worldLinks;

    void parseURDF(const std::string& filePath);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot);

    bool loadURDF(const std::string& filePath);
    void setJointAngles(const std::map<std::string, double>& joint_angles);
    void calculateWorldCoordinates();
    void printModel() const;
    void printCollisionCoordinates() const;

    void computeAABB();
    void computeOBB();

    void configureCollisionOBB(CollisionGeom& collision);
    void configureCollisionAABB(CollisionGeom& collision);

};

#endif