#include "kdtree.hpp"
#include <stdio.h>
#include <iostream>
#include <string.h>


int main() {
    Kdtree::KdNodeVector allnodes;

    auto coordA = Kdtree::KdNode(Kdtree::CoordPoint({1, 0}));
    allnodes.push_back(coordA);
    allnodes.push_back(Kdtree::KdNode(Kdtree::CoordPoint({3, 0})));
    allnodes.push_back(Kdtree::KdNode(Kdtree::CoordPoint({1, 3})));
    allnodes.push_back(Kdtree::KdNode(Kdtree::CoordPoint({0, 2})));
    allnodes.push_back(Kdtree::KdNode{Kdtree::CoordPoint({-1, -2})});

    Kdtree::KdTree kdtree(&allnodes);

    Kdtree::CoordPoint queryPoint({0, 0});
    Kdtree::KdNodeVector res;
    kdtree.k_nearest_neighbors(queryPoint, 2, &res);

    for (auto &kdnode: res) {
        std::cout << "y: " << kdnode.point[0] << ", x: " << kdnode.point[1] << std::endl;
    }
}