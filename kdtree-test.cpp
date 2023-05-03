#include "kdtree.hpp"
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <climits>
#include "assert.h"

void BasicTest() {
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
    kdtree.insert(p1, 0, 0, NULL);
    kdtree.insert(p2, 1, 0, NULL);
    kdtree.insert(p3, 2, 0, NULL);
    kdtree.insert(p4, 3, 0, NULL);
    kdtree.insert(p5, 4, 0, NULL);

    Kdtree::CoordPoint queryPoint({0, 0});
    Kdtree::KdTreeNodeVec res;
    kdtree.k_nearest_neighbors(queryPoint, 3, &res);

    for (auto kdnode: res) {
        std::cout << "y: " << kdnode->point[0] << ", x: " << kdnode->point[1] << " distance: " << kdnode->point[0] * kdnode->point[0]  + kdnode->point[1] * kdnode->point[1] << std::endl;
    }
}

std::pair<float, float> rand_point(float lo_x, float hi_x, float lo_y, float hi_y) {
    float rand_x = lo_x + static_cast <float> (rand()) /( static_cast <float> (static_cast<float>(RAND_MAX)/(hi_x-lo_x)));
    float rand_y = lo_y + static_cast <float> (rand()) /( static_cast <float> (static_cast<float>(RAND_MAX)/(hi_y-lo_y))); 

    return std::make_pair(rand_x, rand_y);
}

void RandomTest() {
    float lo_x = 0;
    float hi_x = 500;
    float lo_y = 0;
    float hi_y = 500;
    Kdtree::CoordPoint min_point({0, 0});
    Kdtree::CoordPoint max_point({500, 500});
    size_t num_dim = 2;

    int num_points = 1000;

    Kdtree::KdTree kdtree(min_point, max_point, num_dim); 
    std::vector<Kdtree::CoordPoint> node_arr;
    
    Kdtree::DistanceL2 measure;
    // generate num_points randomly
    for (int iter = 0; iter < num_points; iter++) {
        
        auto query_coord = rand_point(lo_x, hi_x, lo_y, hi_y);
        Kdtree::CoordPoint query_point;
        query_point.push_back(query_coord.first);
        query_point.push_back(query_coord.second);

        auto _rand_point = rand_point(lo_x, hi_x, lo_y, hi_y); 

        auto new_point = Kdtree::CoordPoint({_rand_point.first, _rand_point.second});
        node_arr.push_back(new_point);
        kdtree.insert(new_point, iter, 0, nullptr);

        int k = 1;
        if (iter < k) {
            continue;
        }

        
        // find closest point to query point
        Kdtree::KdTreeNodeVec res;
        kdtree.k_nearest_neighbors(query_point, k, &res);

        // sort the node_arr by distance wrt query point
        std::sort(node_arr.begin(), node_arr.end(), [query_point, &measure](const Kdtree::CoordPoint &p1, const Kdtree::CoordPoint &p2) {
            return measure.distance(query_point, p1) < measure.distance(query_point, p2); 
        });

        printf("iteration %d:\n", iter);
        for (int i = 0; i < k; i++) {

            // compare result from kd tree with vector
            printf("i: %d\n", i);  
            printf("query point: %f, %f\n", query_coord.first, query_coord.second); 
            printf("kdtree: \n");
            std::cout << res[i]->point[0] << ", " << res[i]->point[1] << std::endl;
            printf("vec: \n");
            std::cout << node_arr[i][0] << ", " << node_arr[i][1] << std::endl;
            std::cout << std::endl;
            kdtree.print_tree();

                    
            assert(abs(res[i]->point[0] - node_arr[i][0]) < 0.001 && abs(res[i]->point[1] - node_arr[i][1]) < 0.001);
        }
    }
}

void RandomTest2() {
    float lo_x = 0;
    float hi_x = 500;
    float lo_y = 0;
    float hi_y = 500;
    Kdtree::CoordPoint min_point({0, 0});
    Kdtree::CoordPoint max_point({500, 500});
    size_t num_dim = 2;

    int num_points = 100;

    Kdtree::KdTree kdtree(min_point, max_point, num_dim); 
    std::vector<Kdtree::CoordPoint> node_arr;
    // generate num_points randomly
    for (int i = 0; i < num_points; i++) {
        auto _rand_point = rand_point(lo_x, hi_x, lo_y, hi_y); 

        auto new_point = Kdtree::CoordPoint({_rand_point.first, _rand_point.second});
        node_arr.push_back(new_point);
        kdtree.insert(new_point, i, 0, NULL);
    }

    auto query_coord = rand_point(lo_x, hi_x, lo_y, hi_y);
    Kdtree::CoordPoint query_point;
    query_point.push_back(query_coord.first);
    query_point.push_back(query_coord.second); 

    int k = 10;
    // find closest point to query point
    Kdtree::KdTreeNodeVec res;
    kdtree.k_nearest_neighbors(query_point, k, &res);
    
    Kdtree::DistanceL2 measure;

    // sort the node_arr by distance wrt query point
    std::sort(node_arr.begin(), node_arr.end(), [query_point, &measure](const Kdtree::CoordPoint &p1, const Kdtree::CoordPoint &p2) {
        return measure.distance(query_point, p1) < measure.distance(query_point, p2); 
    });

    // compare result from kd tree with vector
    for (int i = 0; i < k; i++) {
        printf("kdtree: \n");
        std::cout << *(res[i]) << std::endl;
        printf("vec: \n");
        std::cout << node_arr[i][0] << ", " << node_arr[i][1] << std::endl;
    }
}

int main() {
    RandomTest();
    
}