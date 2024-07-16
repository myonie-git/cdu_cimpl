// #include <iostream>
// #include <string>
// #include <map>
// #include <vector>
// #include <tinyxml2.h>
// #include <Eigen/Dense>
// #include <sstream>
// #include <stack>
// #include <set>

// using namespace tinyxml2;
// using namespace Eigen;

// struct Collision
// {
//     Isometry3d transform = Isometry3d::Identity();
//     Isometry3d global_transform = Isometry3d::Identity(); // Global transform to be calculated
// };

// struct Link
// {
//     std::string name;
//     Isometry3d transform = Isometry3d::Identity();
//     std::vector<Collision> collisions;
// };

// struct Joint
// {
//     std::string name;
//     std::string type;
//     std::string parent;
//     std::string child;
//     Isometry3d transform = Isometry3d::Identity();
//     Vector3d axis = Vector3d::UnitZ(); // Default axis
//     double angle = 0.0; // Joint angle (radians)
//     Isometry3d global_transform = Isometry3d::Identity(); // Global transform to be calculated
// };

// class URDFModel
// {
// public:
//     bool loadURDF(const std::string& filePath);
//     void setJointAngles(const std::map<std::string, double>& joint_angles);
//     void calculateFK();
//     void printModel();

// private:
//     std::map<std::string, Link> links;
//     std::map<std::string, Joint> joints;

//     void applyTransform(const std::string& linkName, const Isometry3d& parentTransform);
//     std::vector<std::string> getTopologicalOrder();
// };

// bool URDFModel::loadURDF(const std::string& filePath)
// {
//     XMLDocument doc;
//     XMLError eResult = doc.LoadFile(filePath.c_str());
//     if (eResult != XML_SUCCESS)
//     {
//         std::cerr << "Failed to load URDF file: " << filePath << std::endl;
//         return false;
//     }

//     const XMLElement* robotElement = doc.FirstChildElement("robot");
//     if (!robotElement)
//     {
//         std::cerr << "No <robot> element found in URDF file." << std::endl;
//         return false;
//     }

//     // Parse links
//     for (const XMLElement* linkElement = robotElement->FirstChildElement("link"); linkElement; linkElement = linkElement->NextSiblingElement("link"))
//     {
//         Link link;
//         link.name = linkElement->Attribute("name");

//         // Parse collisions
//         for (const XMLElement* collisionElement = linkElement->FirstChildElement("collision"); collisionElement; collisionElement = collisionElement->NextSiblingElement("collision"))
//         {
//             Collision collision;
//             const XMLElement* originElement = collisionElement->FirstChildElement("origin");
//             if (originElement)
//             {
//                 double roll = 0.0, pitch = 0.0, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0;
//                 const char* rpyAttr = originElement->Attribute("rpy");
//                 const char* xyzAttr = originElement->Attribute("xyz");

//                 if (rpyAttr)
//                 {
//                     std::istringstream rpyStream(rpyAttr);
//                     rpyStream >> roll >> pitch >> yaw;
//                 }

//                 if (xyzAttr)
//                 {
//                     std::istringstream xyzStream(xyzAttr);
//                     xyzStream >> x >> y >> z;
//                 }

//                 collision.transform = Translation3d(x, y, z) *
//                                       AngleAxisd(roll, Vector3d::UnitX()) *
//                                       AngleAxisd(pitch, Vector3d::UnitY()) *
//                                       AngleAxisd(yaw, Vector3d::UnitZ());
//             }
//             link.collisions.push_back(collision);
//         }

//         links[link.name] = link;
//     }

//     // Parse joints
//     for (const XMLElement* jointElement = robotElement->FirstChildElement("joint"); jointElement; jointElement = jointElement->NextSiblingElement("joint"))
//     {
//         Joint joint;
//         joint.name = jointElement->Attribute("name");
//         joint.type = jointElement->Attribute("type");

//         const XMLElement* parentElement = jointElement->FirstChildElement("parent");
//         const XMLElement* childElement = jointElement->FirstChildElement("child");

//         if (parentElement && childElement)
//         {
//             joint.parent = parentElement->Attribute("link");
//             joint.child = childElement->Attribute("link");
//         }

//         const XMLElement* originElement = jointElement->FirstChildElement("origin");
//         if (originElement)
//         {
//             double roll = 0.0, pitch = 0.0, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0;
//             const char* rpyAttr = originElement->Attribute("rpy");
//             const char* xyzAttr = originElement->Attribute("xyz");

//             if (rpyAttr)
//             {
//                 std::istringstream rpyStream(rpyAttr);
//                 rpyStream >> roll >> pitch >> yaw;
//             }

//             if (xyzAttr)
//             {
//                 std::istringstream xyzStream(xyzAttr);
//                 xyzStream >> x >> y >> z;
//             }

//             joint.transform = Translation3d(x, y, z) *
//                               AngleAxisd(roll, Vector3d::UnitX()) *
//                               AngleAxisd(pitch, Vector3d::UnitY()) *
//                               AngleAxisd(yaw, Vector3d::UnitZ());
//         }

//         const XMLElement* axisElement = jointElement->FirstChildElement("axis");
//         if (axisElement)
//         {
//             double ax = 0.0, ay = 0.0, az = 0.0;
//             const char* axisAttr = axisElement->Attribute("xyz");

//             if (axisAttr)
//             {
//                 std::istringstream axisStream(axisAttr);
//                 axisStream >> ax >> ay >> az;
//             }

//             joint.axis = Vector3d(ax, ay, az);
//         }

//         joints[joint.name] = joint;
//     }

//     return true;
// }

// void URDFModel::setJointAngles(const std::map<std::string, double>& joint_angles)
// {
//     for (auto& jointPair : joints)
//     {
//         if (joint_angles.find(jointPair.first) != joint_angles.end())
//         {
//             jointPair.second.angle = joint_angles.at(jointPair.first);
//         }
//     }
// }

// std::vector<std::string> URDFModel::getTopologicalOrder()
// {
//     std::vector<std::string> order;
//     std::set<std::string> visited;
//     std::stack<std::string> stack;

//     // Find root link (a link that is not a child of any joint)
//     std::set<std::string> children;
//     for (const auto& jointPair : joints)
//     {
//         children.insert(jointPair.second.child);
//     }
//     std::string root;
//     for (const auto& linkPair : links)
//     {
//         if (children.find(linkPair.first) == children.end())
//         {
//             root = linkPair.first;
//             break;
//         }
//     }

//     stack.push(root);
//     while (!stack.empty())
//     {
//         std::string linkName = stack.top();
//         stack.pop();
//         if (visited.find(linkName) == visited.end())
//         {
//             visited.insert(linkName);
//             order.push_back(linkName);

//             // Add children to the stack
//             for (const auto& jointPair : joints)
//             {
//                 if (jointPair.second.parent == linkName)
//                 {
//                     stack.push(jointPair.second.child);
//                 }
//             }
//         }
//     }

//     return order;
// }

// void URDFModel::calculateFK()
// {
//     std::vector<std::string> order = getTopologicalOrder();
//     for (const auto& linkName : order)
//     {
//         if (linkName == order.front())  // Assuming the first element in order is the root
//         {
//             applyTransform(linkName, Isometry3d::Identity());
//         }
//         else
//         {
//             for (const auto& jointPair : joints)
//             {
//                 if (jointPair.second.child == linkName)
//                 {
//                     applyTransform(linkName, joints[jointPair.second.parent].global_transform);
//                     break;
//                 }
//             }
//         }
//     }
// }

// void URDFModel::applyTransform(const std::string& linkName, const Isometry3d& parentTransform)
// {
//     for (auto& jointPair : joints)
//     {
//         Joint& joint = jointPair.second;
//         if (joint.child == linkName)
//         {
//             Isometry3d joint_transform = Isometry3d::Identity();
//             if (joint.type == "revolute" || joint.type == "continuous")
//             {
//                 joint_transform = Translation3d(joint.transform.translation()) *
//                                   AngleAxisd(joint.angle, joint.axis); //TODO ：这里一定有问题
//             }
//             else if (joint.type == "fixed")
//             {
//                 joint_transform = joint.transform;
//             }
//             joint.global_transform = parentTransform * joint_transform;
//             links[linkName].transform = joint.global_transform;

//             for (auto& collision : links[linkName].collisions)
//             {
//                 collision.global_transform = links[linkName].transform * collision.transform;
//             }
//             break;
//         }
//     }
// }

// void URDFModel::printModel()
// {
//     std::vector<std::string> order = getTopologicalOrder();
    
//     std::cout << "Links:" << std::endl;
//     for (const auto& linkName : order)
//     {
//         const Link& link = links[linkName];
//         std::cout << "  " << link.name << std::endl;
//     }

//     std::cout << "Joints:" << std::endl;
//     for (const auto& linkName : order)
//     {
//         for (const auto& jointPair : joints)
//         {
//             const Joint& joint = jointPair.second;
//             if (joint.parent == linkName)
//             {
//                 std::cout << "  " << joint.name << " (" << joint.type << "): " << joint.parent << " -> " << joint.child << std::endl;
//             }
//         }
//     }

//     std::cout << "Transforms:" << std::endl;
//     for (const auto& linkName : order)
//     {
//         const Link& link = links[linkName];
//         std::cout << std::endl << "  Link: " << link.name << std::endl;
//         std::cout << link.transform.matrix() << std::endl << std::endl;

//         for (const auto& collision : link.collisions)
//         {
//             // std::cout << "  Collision: " << std::endl;
//             // std::cout << collision.global_transform.matrix() << std::endl;
//             Vector3d center = collision.global_transform.translation();
//             std::cout << "  Center position: (" << center.x() << ", " << center.y() << ", " << center.z() << ")" << std::endl;
//         }
//     }

//     std::cout << "Joint Transforms:" << std::endl;
//     for (const auto& linkName : order)
//     {
//         for (const auto& jointPair : joints)
//         {
//             const Joint& joint = jointPair.second;
//             if (joint.parent == linkName)
//             {
//                 Vector3d pos = joint.global_transform.translation();
//                 std::cout << "  Joint: " << joint.name << " Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
//                 std::cout << joint.global_transform.matrix() << std::endl;
//             }
//         }
//     }
// }

// int main()
// {
//     URDFModel model;
//     std::string urdfFilePath = "/data/ros/cdu_cimpl/urdf_load/prbt_description.urdf";

//     if (model.loadURDF(urdfFilePath))
//     {
//         std::cout << "Successfully loaded URDF file: " << urdfFilePath << std::endl;

//         // Set joint angles
//         std::map<std::string, double> joint_angles = {
//             {"prbt_joint_1", 0.0},
//             {"prbt_joint_2", 0.0},
//             {"prbt_joint_3", 0.0},
//             {"prbt_joint_4", 0.0},
//             {"prbt_joint_5", 0.0},
//             {"prbt_joint_6", 0.0}
//         };
//         model.setJointAngles(joint_angles);

//         model.calculateFK();
//         model.printModel();
//     }
//     else
//     {
//         std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
//         return -1;
//     }
//     return 0;
// }
#include <iostream>
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>

using namespace tinyxml2;

// 定义一个结构体来存储链接信息
struct Link {
    std::string name;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::vector<Eigen::Vector3d> collisionPositions; // 添加一个用于存储collision坐标的向量
    std::vector<Eigen::Quaterniond> collisionRotations; // 添加一个用于存储collision旋转的向量
};

// 定义一个结构体来存储关节信息
struct Joint {
    std::string name;
    std::string parent;
    std::string child;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::string type; // 增加关节类型
    Eigen::Vector3d axis; // 增加用于存储关节旋转轴
};

// 解析URDF文件并提取链接和关节信息
void parseURDF(const char* urdfFile, std::map<std::string, Link>& links, std::vector<Joint>& joints) {
    XMLDocument doc;
    if (doc.LoadFile(urdfFile) != XML_SUCCESS) {
        std::cerr << "Failed to load URDF file!" << std::endl;
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
                    j.axis = Eigen::Vector3d(0, 0, 1); // 默认旋转轴为Z轴
                }
            } else {
                j.axis = Eigen::Vector3d(0, 0, 1); // 默认旋转轴为Z轴
            }

            joints.push_back(j);
        }
    }
}

// 递归地计算每个链接的真实世界坐标
void computeWorldCoordinates(const std::string& linkName, const std::map<std::string, Link>& links, 
                             const std::vector<Joint>& joints, const std::map<std::string, double>& jointAngles, 
                             const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot, 
                             std::map<std::string, Link>& worldLinks) {
    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot;

    worldLinks[linkName] = link;

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            Eigen::Quaterniond jointRot = joint.rotation;

            // 应用关节角度（仅对旋转关节类型）
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) {
                    double angle = angleIt->second;
                    Eigen::AngleAxisd jointAngleRotation(angle, joint.axis); // 使用关节的旋转轴，构造了一个四元数
                    jointRot = joint.rotation * jointAngleRotation;
                }
            }

            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Quaterniond childRot = parentRot * jointRot;
            computeWorldCoordinates(joint.child, links, joints, jointAngles, childPos, childRot, worldLinks);
        }
    }
}

// 打印Collision的坐标
void printCollisionCoordinates(const std::map<std::string, Link>& worldLinks) {
    for (const auto& [name, link] : worldLinks) {
        for (size_t i = 0; i < link.collisionPositions.size(); ++i) {
            Eigen::Vector3d worldCollisionPos = link.position + link.rotation * link.collisionPositions[i];
            std::cout << "Link: " << name << ", Collision Position: (" << worldCollisionPos.x() << ", " << worldCollisionPos.y() << ", " << worldCollisionPos.z() << ")" << std::endl;
        }
    }
}

int main() {
    const char* urdfFile = "/data/ros/cdu_cimpl/urdf_load/prbt_description.urdf"; // 替换为您的URDF文件路径
    std::map<std::string, Link> links;
    std::vector<Joint> joints;
    std::map<std::string, Link> worldLinks;

    parseURDF(urdfFile, links, joints);

    // 设置关节角度
    std::map<std::string, double> jointAngles = {
        {"prbt_joint_1", 0.0},
        {"prbt_joint_2", 0.0},
        {"prbt_joint_3", 2.0},
        {"prbt_joint_4", 3.0},
        {"prbt_joint_5", 1.0},
        {"prbt_joint_6", 0.0}
    };

    // 计算世界坐标，假设根链接名称为"prbt_base_link"（请根据您的URDF文件修改）
    computeWorldCoordinates("prbt_base_link", links, joints, jointAngles, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0), worldLinks);

    // 打印计算的世界坐标
    for (const auto& [name, link] : worldLinks) {
        std::cout << "Link: " << name << ", Position: (" << link.position.x() << ", " << link.position.y() << ", " << link.position.z() << ")" << std::endl;
    }

    // 打印Collision的坐标
    printCollisionCoordinates(worldLinks);

    return 0;
}
