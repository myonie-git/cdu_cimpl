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
    const std::string filename = "boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

    return 0;
}