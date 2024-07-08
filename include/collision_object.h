// include/collision_object.h
#ifndef COLLISION_OBJECT_H
#define COLLISION_OBJECT_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "aabb.h"


// CollisionObject 类定义
template <typename S>
class CollisionObject
{
public:    

    CollisionObject(const Vector3<S>& center, S radius, const Eigen::Transform<S, 3, Eigen::Isometry>& transform)
        : aabb_center(center), aabb_radius(radius), t(transform)
    {
    }

    CollisionObject(const AABB<S>& aabb_) : aabb(aabb_)
    {
    }

    void computeAABB();

    //the position of the center of aabb
    Vector3<S> aabb_center;

    //the radius of aabb 
    S aabb_radius;

    //the transform matrix of AABB (can be zero)
    Eigen::Transform<S, 3, Eigen::Isometry> t;

    //the value of collision obj
    AABB<S> aabb;
};



// 读取盒子数据的函数
template <typename S>
std::vector<CollisionObject<S>*> readBoxesFromFile(const std::string& filename) {
    std::vector<Vector3<S>> sizes;
    std::vector<Vector3<S>> positions;

    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)) {
        
        if (line.find("Size:") != std::string::npos) {
            Vector3<S> size;
            std::stringstream ss(line.substr(line.find(":") + 1));
            ss >> size[0] >> size[1] >> size[2];
            sizes.push_back(size);
        } else if (line.find("Position:") != std::string::npos) {
            Vector3<S> position;
            std::stringstream ss(line.substr(line.find(":") + 1));
            ss >> position[0] >> position[1] >> position[2];
            positions.push_back(position);
        }
    }

    infile.close();

    std::vector<CollisionObject<S>*> collision_objects;
    for (size_t i = 0; i < sizes.size(); ++i) {
        Vector3<S> min = positions[i] - sizes[i] / 2;
        Vector3<S> max = positions[i] + sizes[i] / 2;
        AABB<S> aabb(min, max);
        collision_objects.push_back(new CollisionObject<S>(aabb));
    }

    std::cout << sizes.size() << std::endl;

    return collision_objects;

}

#endif // collision_object
