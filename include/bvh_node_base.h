#ifndef BVH_NODE_BASE_H
#define BVH_NODE_BASE_H

template<typename BV>
class NodeBase{
public:
    BV bv;
    NodeBase<BV>* parent;

    union
    {
        /// @brief for leaf node, children nodes
        NodeBase<BV>* children[2];
        void* data;
    };

    bool isLeaf() const;
    bool isInternal() const;

    NodeBase();
    NodeBase(const BV& bv_);
};

#endif