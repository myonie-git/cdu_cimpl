#include "urdf_model.h"

using namespace tinyxml2;

bool URDFModel::loadURDF(const std::string& filePath) {
    parseURDF(filePath);
    return !links.empty() && !joints.empty();
}

void URDFModel::setJointAngles(const std::map<std::string, double>& joint_angles) {
    jointAngles = joint_angles;
}

void URDFModel::calculateWorldCoordinates() {
    if (links.empty() || joints.empty()) {
        std::cerr << "Links or joints are not loaded properly." << std::endl;
        return;
    }
    const std::string rootLink = "prbt_base_link"; // Replace with actual root link if different
    computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0));
}

void URDFModel::printModel() const {
    for (const auto& [name, link] : worldLinks) {
        std::cout << "Link: " << name << ", Position: (" << link.position.x() << ", " << link.position.y() << ", " << link.position.z() << ")" << std::endl;
    }
}

void URDFModel::printCollisionCoordinates() const {
    for (const auto& [name, link] : worldLinks) {
        for (const auto& collision : link.collisions) {
            Eigen::Vector3d worldCollisionPos = link.position + link.rotation * collision.collisionPositions;
            std::cout << "Link: " << name << ", Collision Position: (" << worldCollisionPos.x() << ", " << worldCollisionPos.y() << ", " << worldCollisionPos.z() << ")" << std::endl;
        }
    }
}

void URDFModel::parseURDF(const std::string& filePath) {
    XMLDocument doc;
    if (doc.LoadFile(filePath.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load URDF file: " << filePath << std::endl;
        return;
    }

    XMLElement* robot = doc.FirstChildElement("robot");
    if (!robot) {
        std::cerr << "No robot element found in URDF file!" << std::endl;
        return;
    }

    for (XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
        const char* name = link->Attribute("name");
        if (name) {
            Link l;
            l.name = name;
            l.position = Eigen::Vector3d(0, 0, 0);
            l.rotation = Eigen::Quaterniond(1, 0, 0, 0);

            for (XMLElement* collision = link->FirstChildElement("collision"); collision; collision = collision->NextSiblingElement("collision")) {
                XMLElement* origin = collision->FirstChildElement("origin");
                Eigen::Vector3d collPos = Eigen::Vector3d(0, 0, 0);
                Eigen::Quaterniond collRot = Eigen::Quaterniond(1, 0, 0, 0);

                if (origin) {
                    const char* xyz = origin->Attribute("xyz");
                    if (xyz) {
                        sscanf(xyz, "%lf %lf %lf", &collPos.x(), &collPos.y(), &collPos.z());
                    }

                    const char* rpy = origin->Attribute("rpy");
                    if (rpy) {
                        double roll, pitch, yaw;
                        sscanf(rpy, "%lf %lf %lf", &roll, &pitch, &yaw);
                        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                        collRot = yawAngle * pitchAngle * rollAngle;
                    } 

                    XMLElement* geometry = collision->FirstChildElement("geometry");
                    if(geometry){
                        std::shared_ptr<bodies::Body> body;
                        CollisionGeom collisionGeom;
                        collisionGeom.collisionPositions = collPos;
                        collisionGeom.collisionRotations = collRot;

                        XMLElement* box = geometry->FirstChildElement("box");
                        if(box){
                            const char* size = box->Attribute("size");
                            if(size){
                                Eigen::Vector3d boxSize;
                                sscanf(size, "%lf %lf %lf", &boxSize.x(), &boxSize.y(), &boxSize.z());
                                shapes::Box shape(boxSize.x(), boxSize.y(), boxSize.z());
                                body = std::make_shared<bodies::Box>(&shape);
                                body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                                collisionGeom.type = CollisionGeom::BOX;
                                collisionGeom.body = body;
                                l.collisions.push_back(collisionGeom);
                            }
                        }
                        
                            
                        XMLElement* sphere = geometry->FirstChildElement("sphere");
                        if (sphere) {
                            const char* radius = sphere->Attribute("radius");
                            if (radius) {
                                double sphereRadius;
                                sscanf(radius, "%lf", &sphereRadius);
                                shapes::Sphere shape(sphereRadius);
                                body = std::make_shared<bodies::Sphere>(&shape);
                                body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                                collisionGeom.type = CollisionGeom::SPHERE;
                                collisionGeom.body = body;
                                l.collisions.push_back(collisionGeom);
                            }
                        }

                        XMLElement* cylinder = geometry->FirstChildElement("cylinder");
                        if (cylinder) {
                            const char* radius = cylinder->Attribute("radius");
                            const char* length = cylinder->Attribute("length");
                            if (radius && length) {
                                double cylinderRadius, cylinderLength;
                                sscanf(radius, "%lf", &cylinderRadius);
                                sscanf(length, "%lf", &cylinderLength);
                                shapes::Cylinder shape(cylinderRadius, cylinderLength);
                                body = std::make_shared<bodies::Cylinder>(&shape);
                                body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                                collisionGeom.type = CollisionGeom::CYLINDER;
                                collisionGeom.body = body;
                                l.collisions.push_back(collisionGeom);
                            }
                        }

                        XMLElement* mesh = geometry->FirstChildElement("mesh");
                        if (mesh) {
                            const char* filename = mesh->Attribute("filename");
                            const char* scale = mesh->Attribute("scale");
                            Eigen::Vector3d meshScale(1, 1, 1);
                            if (scale) {
                                sscanf(scale, "%lf %lf %lf", &meshScale.x(), &meshScale.y(), &meshScale.z());
                            }
                            if (filename) {
                                shapes::Mesh* shape = shapes::createMeshFromResource(filename, meshScale);
                                if (shape) {
                                    body = std::make_shared<bodies::ConvexMesh>(shape);
                                    body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                                    collisionGeom.type = CollisionGeom::MESH;
                                    collisionGeom.body = body;
                                    l.collisions.push_back(collisionGeom);
                                }
                            }
                        }    
                    }
                }
            }

            links[name] = l;
        }
    }

    for (XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
        const char* name = joint->Attribute("name");
        const char* parent = joint->FirstChildElement("parent")->Attribute("link");
        const char* child = joint->FirstChildElement("child")->Attribute("link");
        const char* type = joint->Attribute("type");
        if (parent && child && name && type) {
            Joint j;
            j.name = name;
            j.parent = parent;
            j.child = child;
            j.type = type;

            XMLElement* origin = joint->FirstChildElement("origin");
            if (origin) {
                const char* xyz = origin->Attribute("xyz");
                if (xyz) {
                    sscanf(xyz, "%lf %lf %lf", &j.position.x(), &j.position.y(), &j.position.z());
                } else {
                    j.position = Eigen::Vector3d(0, 0, 0);
                }

                const char* rpy = origin->Attribute("rpy");
                if (rpy) {
                    double roll, pitch, yaw;
                    sscanf(rpy, "%lf %lf %lf", &roll, &pitch, &yaw);
                    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                    j.rotation = yawAngle * pitchAngle * rollAngle;
                } else {
                    j.rotation = Eigen::Quaterniond(1, 0, 0, 0);
                }
            } else {
                j.position = Eigen::Vector3d(0, 0, 0);
                j.rotation = Eigen::Quaterniond(1, 0, 0, 0);
            }

            XMLElement* axis = joint->FirstChildElement("axis");
            if (axis) {
                const char* xyz = axis->Attribute("xyz");
                if (xyz) {
                    sscanf(xyz, "%lf %lf %lf", &j.axis.x(), &j.axis.y(), &j.axis.z());
                } else {
                    j.axis = Eigen::Vector3d(0, 0, 1);
                }
            } else {
                j.axis = Eigen::Vector3d(0, 0, 1);
            }

            joints.push_back(j);
        }
    }

    computeAABB();
    computeOBB();

}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot) {
    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot;

    worldLinks[linkName] = link;

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            Eigen::Quaterniond jointRot = joint.rotation;

            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) {
                    double angle = angleIt->second;
                    Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
                    jointRot = joint.rotation * jointAngleRotation;
                }
            }

            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Quaterniond childRot = parentRot * jointRot;
            computeWorldCoordinates(joint.child, childPos, childRot);
        }
    }
}

void URDFModel::computeAABB(){
        for(auto& linkPair : links){
        Link& link = linkPair.second;
        for (auto& collision : link.collisions) {
            configureCollisionAABB(collision);
        }
    }
}

void URDFModel::computeOBB(){
    for(auto& linkPair : links){
        Link& link = linkPair.second;
        for (auto& collision : link.collisions) {
            configureCollisionOBB(collision);
        }
    }
}

void URDFModel::configureCollisionOBB(CollisionGeom& collision){
    if(!collision.body){
        std::cerr << "Error: Collision body is not set." << std::endl;
        return;      
    }

    bodies::OBB obb;
    collision.body->computeBoundingBox(obb);

    collision.obb.extent = obb.getExtents();

    Eigen::Isometry3d obbPose = obb.getPose();
    collision.obb.axis = obbPose.rotation();
    collision.obb.To = obbPose.translation();

    return;
}

void URDFModel::configureCollisionAABB(CollisionGeom& collision){
    if(!collision.body){
        std::cerr << "Error: Collision body is not set." << std::endl;
        return;      
    }

    bodies::AABB aabb;
    collision.body->computeBoundingBox(aabb);


    collision.aabb.min_ = aabb.min();
    collision.aabb.max_ = aabb.max();

    return; 

}