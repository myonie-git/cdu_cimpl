#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tinyxml2.h>
#include <Eigen/Dense>
#include <sstream>
#include <stack>
#include <set>

using namespace tinyxml2;
using namespace Eigen;

struct Collision
{
    Isometry3d transform = Isometry3d::Identity();
    Isometry3d global_transform = Isometry3d::Identity(); // Global transform to be calculated
};

struct Link
{
    std::string name;
    Isometry3d transform = Isometry3d::Identity();
    std::vector<Collision> collisions;
};

struct Joint
{
    std::string name;
    std::string type;
    std::string parent;
    std::string child;
    Isometry3d transform = Isometry3d::Identity();
    Vector3d axis = Vector3d::UnitZ(); // Default axis
    double angle = 0.0; // Joint angle (radians)
};

class URDFModel
{
public:
    bool loadURDF(const std::string& filePath);
    void setJointAngles(const std::map<std::string, double>& joint_angles);
    void calculateFK();
    void printModel();

private:
    std::map<std::string, Link> links;
    std::map<std::string, Joint> joints;

    void applyTransform(const std::string& linkName, const Isometry3d& parentTransform);
    std::vector<std::string> getTopologicalOrder();
};

bool URDFModel::loadURDF(const std::string& filePath)
{
    XMLDocument doc;
    XMLError eResult = doc.LoadFile(filePath.c_str());
    if (eResult != XML_SUCCESS)
    {
        std::cerr << "Failed to load URDF file: " << filePath << std::endl;
        return false;
    }

    const XMLElement* robotElement = doc.FirstChildElement("robot");
    if (!robotElement)
    {
        std::cerr << "No <robot> element found in URDF file." << std::endl;
        return false;
    }

    // Parse links
    for (const XMLElement* linkElement = robotElement->FirstChildElement("link"); linkElement; linkElement = linkElement->NextSiblingElement("link"))
    {
        Link link;
        link.name = linkElement->Attribute("name");

        // Parse collisions
        for (const XMLElement* collisionElement = linkElement->FirstChildElement("collision"); collisionElement; collisionElement = collisionElement->NextSiblingElement("collision"))
        {
            Collision collision;
            const XMLElement* originElement = collisionElement->FirstChildElement("origin");
            if (originElement)
            {
                double roll = 0.0, pitch = 0.0, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0;
                const char* rpyAttr = originElement->Attribute("rpy");
                const char* xyzAttr = originElement->Attribute("xyz");

                if (rpyAttr)
                {
                    std::istringstream rpyStream(rpyAttr);
                    rpyStream >> roll >> pitch >> yaw;
                }

                if (xyzAttr)
                {
                    std::istringstream xyzStream(xyzAttr);
                    xyzStream >> x >> y >> z;
                }

                collision.transform = Translation3d(x, y, z) *
                                      AngleAxisd(roll, Vector3d::UnitX()) *
                                      AngleAxisd(pitch, Vector3d::UnitY()) *
                                      AngleAxisd(yaw, Vector3d::UnitZ());
            }
            link.collisions.push_back(collision);
        }

        links[link.name] = link;
    }

    // Parse joints
    for (const XMLElement* jointElement = robotElement->FirstChildElement("joint"); jointElement; jointElement = jointElement->NextSiblingElement("joint"))
    {
        Joint joint;
        joint.name = jointElement->Attribute("name");
        joint.type = jointElement->Attribute("type");

        const XMLElement* parentElement = jointElement->FirstChildElement("parent");
        const XMLElement* childElement = jointElement->FirstChildElement("child");

        if (parentElement && childElement)
        {
            joint.parent = parentElement->Attribute("link");
            joint.child = childElement->Attribute("link");
        }

        const XMLElement* originElement = jointElement->FirstChildElement("origin");
        if (originElement)
        {
            double roll = 0.0, pitch = 0.0, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0;
            const char* rpyAttr = originElement->Attribute("rpy");
            const char* xyzAttr = originElement->Attribute("xyz");

            if (rpyAttr)
            {
                std::istringstream rpyStream(rpyAttr);
                rpyStream >> roll >> pitch >> yaw;
            }

            if (xyzAttr)
            {
                std::istringstream xyzStream(xyzAttr);
                xyzStream >> x >> y >> z;
            }

            joint.transform = Translation3d(x, y, z) *
                              AngleAxisd(roll, Vector3d::UnitX()) *
                              AngleAxisd(pitch, Vector3d::UnitY()) *
                              AngleAxisd(yaw, Vector3d::UnitZ());
        }

        const XMLElement* axisElement = jointElement->FirstChildElement("axis");
        if (axisElement)
        {
            double ax = 0.0, ay = 0.0, az = 0.0;
            const char* axisAttr = axisElement->Attribute("xyz");

            if (axisAttr)
            {
                std::istringstream axisStream(axisAttr);
                axisStream >> ax >> ay >> az;
            }

            joint.axis = Vector3d(ax, ay, az);
        }

        joints[joint.name] = joint;
    }

    return true;
}

void URDFModel::setJointAngles(const std::map<std::string, double>& joint_angles)
{
    for (auto& jointPair : joints)
    {
        if (joint_angles.find(jointPair.first) != joint_angles.end())
        {
            jointPair.second.angle = joint_angles.at(jointPair.first);
        }
    }
}

std::vector<std::string> URDFModel::getTopologicalOrder()
{
    std::vector<std::string> order;
    std::set<std::string> visited;
    std::stack<std::string> stack;

    // Find root link (a link that is not a child of any joint)
    std::set<std::string> children;
    for (const auto& jointPair : joints)
    {
        children.insert(jointPair.second.child);
    }
    std::string root;
    for (const auto& linkPair : links)
    {
        if (children.find(linkPair.first) == children.end())
        {
            root = linkPair.first;
            break;
        }
    }

    stack.push(root);
    while (!stack.empty())
    {
        std::string linkName = stack.top();
        stack.pop();
        if (visited.find(linkName) == visited.end())
        {
            visited.insert(linkName);
            order.push_back(linkName);

            // Add children to the stack
            for (const auto& jointPair : joints)
            {
                if (jointPair.second.parent == linkName)
                {
                    stack.push(jointPair.second.child);
                }
            }
        }
    }

    return order;
}

void URDFModel::calculateFK()
{
    std::vector<std::string> order = getTopologicalOrder();
    for (const auto& linkName : order)
    {
        if (links[linkName].transform.isApprox(Isometry3d::Identity()))
        {
            applyTransform(linkName, Isometry3d::Identity());
        }
    }
}

void URDFModel::applyTransform(const std::string& linkName, const Isometry3d& parentTransform)
{
    links[linkName].transform = parentTransform;

    for (auto& collision : links[linkName].collisions)
    {
        collision.global_transform = parentTransform * collision.transform;
    }

    for (const auto& jointPair : joints)
    {
        const Joint& joint = jointPair.second;
        if (joint.parent == linkName)
        {
            Isometry3d joint_transform = Isometry3d::Identity();
            if (joint.type == "revolute" || joint.type == "continuous")
            {
                joint_transform = Translation3d(joint.transform.translation()) *
                                  AngleAxisd(joint.angle, joint.axis);
            }
            else if (joint.type == "fixed")
            {
                joint_transform = joint.transform;
            }
            applyTransform(joint.child, links[linkName].transform * joint_transform);
        }
    }
}

void URDFModel::printModel()
{
    std::cout << "Links:" << std::endl;
    for (const auto& linkPair : links)
    {
        std::cout << "  " << linkPair.second.name << std::endl;
    }

    std::cout << "Joints:" << std::endl;
    for (const auto& jointPair : joints)
    {
        const Joint& joint = jointPair.second;
        std::cout << "  " << joint.name << " (" << joint.type << "): " << joint.parent << " -> " << joint.child << std::endl;
    }

    std::cout << "Transforms:" << std::endl;
    for (const auto& linkPair : links)
    {
        std::cout << "  Link: " << linkPair.second.name << std::endl;
        std::cout << linkPair.second.transform.matrix() << std::endl << std::endl;

        for (const auto& collision : linkPair.second.collisions)
        {
            std::cout << "  Collision: " << std::endl;
            std::cout << collision.global_transform.matrix() << std::endl;
            Vector3d center = collision.global_transform.translation();
            std::cout << "  Center position: (" << center.x() << ", " << center.y() << ", " << center.z() << ")" << std::endl << std::endl;
        }
    }
}

int main()
{
    URDFModel model;
    std::string urdfFilePath = "/data/ros/urdf_load/prbt_description.urdf";

    if (model.loadURDF(urdfFilePath))
    {
        std::cout << "Successfully loaded URDF file: " << urdfFilePath << std::endl;

        // Set joint angles
        std::map<std::string, double> joint_angles = {
            {"prbt_joint_1", 0.0},
            {"prbt_joint_2", 0.0},
            {"prbt_joint_3", 0.0},
            {"prbt_joint_4", 0.0},
            {"prbt_joint_5", 0.0},
            {"prbt_joint_6", 0.0}
        };
        model.setJointAngles(joint_angles);

        model.calculateFK();
        model.printModel();
    }
    else
    {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        return -1;
    }

    return 0;
}
