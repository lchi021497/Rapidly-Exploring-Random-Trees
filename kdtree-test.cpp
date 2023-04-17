#include "kdtree.hpp"
#include <stdio.h>
#include <iostream>
#include <string.h>


int main() {
    Kdtree::CoordPoint min_point({0, 0});
    Kdtree::CoordPoint max_point({500, 500});
    size_t num_dim = 2;

    // Kdtree::CoordPointVec allnodes;

    auto p1 = Kdtree::CoordPoint({51, 50});
    auto p2 = Kdtree::CoordPoint({53, 50});
    auto p3 = Kdtree::CoordPoint({51, 53});
    auto p4 = Kdtree::CoordPoint({50, 52});
    auto p5 = Kdtree::CoordPoint({49, 48});
    // allnodes.push_back(p1);
    // allnodes.push_back(p2);
    // allnodes.push_back(p3);
    // allnodes.push_back(p4);
    // allnodes.push_back(p5);
    
    // Kdtree::KdTree kdtree(allnodes, min_point, max_point, num_dim);

    // Kdtree::CoordPoint queryPoint({0, 0});
    // Kdtree::KdTreeNodeVec res;
    // kdtree.k_nearest_neighbors(queryPoint, 3, &res);

    // for (auto kdnode: res) {
    //     std::cout << "y: " << kdnode->point[0] << ", x: " << kdnode->point[1] << std::endl;
    // }

    // use push instead of taking in vector
    Kdtree::KdTree kdtree(min_point, max_point, num_dim);
    kdtree.insert(p1);
    kdtree.insert(p2);
    kdtree.insert(p3);
    kdtree.insert(p4);
    kdtree.insert(p5);

    Kdtree::CoordPoint queryPoint({0, 0});
    Kdtree::KdTreeNodeVec res;
    kdtree.k_nearest_neighbors(queryPoint, 3, &res);

    for (auto kdnode: res) {
        std::cout << "y: " << kdnode->point[0] << ", x: " << kdnode->point[1] << " distance: " << kdnode->point[0] * kdnode->point[0]  + kdnode->point[1] * kdnode->point[1] << std::endl;
    }
    
}