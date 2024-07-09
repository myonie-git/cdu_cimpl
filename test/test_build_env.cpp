#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "aabb.h"

int main(){

    using S = double;
    const std::string filename = "/data/ros/cdu_cimpl/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

    CollisionEnv<S> env;
    env.InitTree(collision_objects);


    return 0;
}