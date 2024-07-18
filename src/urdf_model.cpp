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
        for (size_t i = 0; i < link.collisionPositions.size(); ++i) {
            Eigen::Vector3d worldCollisionPos = link.position + link.rotation * link.collisionPositions[i];
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
                if (origin) {
                    Eigen::Vector3d collPos;
                    Eigen::Quaterniond collRot;

                    const char* xyz = origin->Attribute("xyz");
                    if (xyz) {
                        sscanf(xyz, "%lf %lf %lf", &collPos.x(), &collPos.y(), &collPos.z());
                    } else {
                        collPos = Eigen::Vector3d(0, 0, 0);
                    }

                    const char* rpy = origin->Attribute("rpy");
                    if (rpy) {
                        double roll, pitch, yaw;
                        sscanf(rpy, "%lf %lf %lf", &roll, &pitch, &yaw);
                        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                        collRot = yawAngle * pitchAngle * rollAngle;
                    } else {
                        collRot = Eigen::Quaterniond(1, 0, 0, 0);
                    }

                    l.collisionPositions.push_back(collPos);
                    l.collisionRotations.push_back(collRot);
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
