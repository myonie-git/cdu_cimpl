#ifndef URDF_MODEL_H
#define URDF_MODEL_H

#include <iostream>
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <Eigen/Dense>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>

#include "aabb.h"
#include "obb.h"

using namespace tinyxml2;

struct Link; 

struct CollisionGeom {
    
    using S = double;

    enum ShapeType { BOX, SPHERE, CYLINDER, MESH };
    ShapeType type;
    Eigen::Vector3d collisionPositions; //用于表征真实向量
    Eigen::Quaterniond collisionRotations; //用于表征现实中的旋转,四元数
    std::shared_ptr<bodies::Body> body;
    AABB<S> aabb;
    OBB<S> obb;
    S aabb_radius;
    Link* parentLink;
};

struct Link {
    std::string name;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::vector<std::shared_ptr<CollisionGeom>> collisions;
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
    std::vector<std::shared_ptr<CollisionGeom>> collisionGeometries;

    void parseURDF(const std::string& filePath);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot);

    bool loadURDF(const std::string& filePath);
    void setJointAngles(const std::map<std::string, double>& joint_angles);
    void calculateWorldCoordinates();
    void printModel() const;
    void printCollisionCoordinates() const;

    void computeAABB();
    void computeOBB();

    void configureCollisionOBB(std::shared_ptr<CollisionGeom>& collision);
    void configureCollisionAABB(std::shared_ptr<CollisionGeom>& collision);

    void configureCollisionAABBRadius(std::shared_ptr<CollisionGeom>& collision);
    
    int getCollisionNum();
};

#endif