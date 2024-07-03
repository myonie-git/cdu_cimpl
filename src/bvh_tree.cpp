#include "bvh_tree.h"

template<typename BV>
BvhTree<BV>::BvhTree(){
    root_node = nullptr;
    n_leaves = 0;
    bu_threshold = 2;
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
void BvhTree<BV>::recurseDeleteNode(NodeType* Node){
    if(!node->isLeaf()){
        recurseDeleteNode(node->children[0]);
        recurseDeleteNode(node->children[1]);
    }

    if(node == root_node) root_node = nullptr;
    delete(node);
}

template<typename BV>
void BvhTree<BV>::deleteNode(NodeType* Node){
    delete node;
}

template<typename BV>
void BvhTree<BV>::NodeType* BvhTree<BV>::getRoot() const{
    return root_node;
}

template<typename BV>
void BvhTree<BV>::NodeType*& BvhTree<BV>::getRoot() const{
    return root_node;
}


template<typename BV>
void BvhTree<BV>::init(std::vector<NodeType*>& leaves){
    clear();
    root_node = topdown(leaves.begin(), leaves.end());
    n_leaves = leaves.size();
}

template<typename BV>
typename BvhTree<BV>::NodeType* HierarchyTree<BV>::topdown(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
    int num_leaves = lend - lbeg;
    if(num_leaves > 1){
        if(num_leaves > bu_threshold){
            BV vol = (*lbeg)->bv;
            for(NodeVecIterator it = lbeg + 1; it < lend; ++it)
                vol += (*it)->bv;

            int best_axis = 0;
            


        }
        else{

        }
    }

    return *lbeg;
}