#include "kdtree.hpp"
#include <stdio.h>
#include <iostream>
#include <string.h>


int main() {


    Kdtree::CoordPoint min_point({0, 0});
    Kdtree::CoordPoint max_point({500, 500});
    size_t num_dim = 2;

    Kdtree::CoordPointVec allnodes;

    auto p1 = Kdtree::CoordPoint({51.0, 50.0});
    auto p2 = Kdtree::CoordPoint({52.0, 0.0});
    auto p3 = Kdtree::CoordPoint({51.0, 53.0});
    auto p4 = Kdtree::CoordPoint({50.0, 52.0});
    auto p5 = Kdtree::CoordPoint({49.0, 48.0});
    allnodes.push_back(p1);
    allnodes.push_back(p2);
    allnodes.push_back(p3);
    allnodes.push_back(p4);
    allnodes.push_back(p5);
 
    
    Kdtree::KdTree kdtree2(allnodes, min_point, max_point, num_dim);

    Kdtree::CoordPoint queryPoint2({0, 0});
    Kdtree::KdTreeNodeVec res2;
    kdtree2.k_nearest_neighbors(queryPoint2, 3, &res2);

    for (auto kdnode: res2) {
        std::cout << "x: " << kdnode->point[0] << ", y: " << kdnode->point[1] << " distance: " << kdnode->point[0] * kdnode->point[0]  + kdnode->point[1] * kdnode->point[1] << std::endl;

    }

    // use push instead of taking in vector
    Kdtree::KdTree kdtree(min_point, max_point, num_dim);
    kdtree.insert(p1, 0, 0, nullptr);
    kdtree.insert(p2, 1, 0, nullptr);
    kdtree.insert(p3, 2, 0, nullptr);
    kdtree.insert(p4, 3, 0, nullptr);
    kdtree.insert(p5, 4, 0, nullptr);

    Kdtree::CoordPoint queryPoint({0, 0});
    Kdtree::KdTreeNodeVec res;
    kdtree.k_nearest_neighbors(queryPoint, 2, &res);

    for (auto kdnode: res) {
        std::cout << "x: " << kdnode->point[0] << ", y: " << kdnode->point[1] << " distance: " << kdnode->point[0] * kdnode->point[0]  + kdnode->point[1] * kdnode->point[1] << std::endl;
    }
    
}