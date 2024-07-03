#ifndef BVH_TREE_H
#define BVH_TREE_H

#include <vector>
#include <map>
#include <iostream>

#include "aabb.h"
#include "bvh_node_base.h"

template <typename BV>
class BvhTree{
public:
    typedef NodeBase<AABB<double>> NodeType;
    BvhTree();
    ~BvhTree();
    void init(std::vector<NodeType*>& leaves); //初始化bvh树
    NodeType* getRoot() const;
    NodeType&* getRoot();
    void clear();

    int bu_threshold;
    size_t n_leaves;
    NodeType* root_node;

private:
    
    void recurseDeleteNode(NodeType* Node);
    void deleteNode(NodeType* node);

    typedef typename std::vector<NodeType*>::iterator NodeVecIterator;
    NodeType* topdown(const NodeVecIterator lbeg, const NodeVecIterator lend);

    

};


#endif