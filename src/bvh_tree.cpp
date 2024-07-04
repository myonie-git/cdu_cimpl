#include "bvh_tree.h"

template<typename BV>
BvhTree<BV>::BvhTree(){
    root_node = nullptr;
    n_leaves = 0;
    bu_threshold = 1;
}

template<typename BV>
BvhTree<BV>::~BvhTree(){
    clear();
}

template<typename BV>
void BvhTree<BV>::clear(){
    if(root_node){
        recurseDeleteNode(root_node);
    }
    n_leaves = 0;

}

template<typename BV>
void BvhTree<BV>::recurseDeleteNode(NodeType* node){
    if(!node->isLeaf()){
        recurseDeleteNode(node->children[0]);
        recurseDeleteNode(node->children[1]);
    }

    if(node == root_node) root_node = nullptr;
    delete(node);
}

template<typename BV>
void BvhTree<BV>::deleteNode(NodeType* node){
    delete node;
}

template<typename BV>
typename BvhTree<BV>::NodeType* BvhTree<BV>::getRoot() const{
    return root_node;
}

template<typename BV>
typename BvhTree<BV>::NodeType*& BvhTree<BV>::getRoot(){
    return root_node;
}


template<typename BV>
void BvhTree<BV>::init(std::vector<NodeType*>& leaves){
    clear();
    root_node = topdown(leaves.begin(), leaves.end());
    n_leaves = leaves.size();
}

//==============================================================================
template<typename BV>
bool nodeBaseLess(NodeBase<BV>* a, NodeBase<BV>* b, int d)
{
  if(a->bv.center()[d] < b->bv.center()[d]) return true;
  return false;
}

template<typename BV>
typename BvhTree<BV>::NodeType* BvhTree<BV>::topdown(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
    int num_leaves = lend - lbeg;
    if(num_leaves > 1){
        if(num_leaves > bu_threshold){
            BV vol = (*lbeg)->bv;
            for(NodeVecIterator it = lbeg + 1; it < lend; ++it)
                vol += (*it)->bv;

            int best_axis = 0;
            S extent[3] = {vol.width(), vol.height(), vol.depth()};
            if(extent[1] > extent[0]) best_axis = 1;
            if(extent[2] > extent[best_axis]) best_axis = 2;

            // compute median
            NodeVecIterator lcenter = lbeg + num_leaves / 2;
            std::nth_element(lbeg, lcenter, lend, std::bind(&nodeBaseLess<BV>, std::placeholders::_1, std::placeholders::_2, std::ref(best_axis))); //对节点范围进行部分排序，并选择一个中间点作为切割点

            NodeType* node = createNode(nullptr, vol, nullptr);
            node->children[0] = topdown(lbeg, lcenter);
            node->children[1] = topdown(lcenter, lend);
            node->children[0]->parent = node;
            node->children[1]->parent = node;
            return node;
        }
        else{
            assert(0);
            return *lbeg;
        }
    }
    return *lbeg;
}