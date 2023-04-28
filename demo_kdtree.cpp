//
// Demonstration of typical use cases of the kd-tree library
// Author:  Christoph Dalitz, 2018-02-04
// License: BSD style license (see the file LICENSE)
//

#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "kdtree.hpp"

using namespace std;

//
// helper function for printing points
//
void print_nodes(const Kdtree::KdNodeVector &nodes) {
  size_t i,j;
  for (i = 0; i < nodes.size(); ++i) {
    if (i > 0)
      cout << " ";
    cout << nodes[i].index << "(";
    for (j = 0; j < nodes[i].point.size(); j++) {
      if (j > 0)
        cout << ",";
      cout << nodes[i].point[j];
    }
    cout << ")" << nodes[i].point[0]*nodes[i].point[0] + nodes[i].point[1]*nodes[i].point[1] ;
  }
  cout << endl;
}

void print_nodes(const Kdtree::KdTreeNodeVec &nodes) {
  size_t i,j;
  for (i = 0; i < nodes.size(); ++i) {
    if (i > 0)
      cout << " ";
    // cout << nodes[i]->index << "(";
    cout << "(";
    for (j = 0; j < nodes[i]->point.size(); j++) {
      if (j > 0)
        cout << ",";
      cout << nodes[i]->point[j];
    }
    // cout << ")" << nodes[i]->point[0]*nodes[i]->point[0] + nodes[i]->point[1]*nodes[i]->point[1] ;
    cout << ")";
    // cout << nodes[i]->par_cost->cost;

  }
  cout << endl;
}

//
// main program demonstrating typical use cases
//
int main(int argc, char** argv) {

   size_t N = 1;

  srand (time(NULL));
  Kdtree::CoordPoint min_point({0, 0});
  Kdtree::CoordPoint max_point({500, 500});
  size_t num_dim = 2;
  Kdtree::KdTree kdtree(min_point, max_point, num_dim);

  for (size_t i = 0; i < N; ++i) {
    std::vector<double> point(2);
    // point[0] = (double)((rand() / RAND_MAX));
    // point[1] = (double)((rand() / RAND_MAX));
    point[0] = (double)((rand()/(RAND_MAX*1.0)) * 500.0);
    point[1] = (double)((rand()/(RAND_MAX*1.0)) * 500.0);

    std::cout << point[0] << " " << point[1] << std::endl;
    kdtree.insert(point, i, 0, nullptr);

  }
  int num_neighbors = 1;

  Kdtree::CoordPoint queryPoint({(double)((rand()/(RAND_MAX*1.0)) * 500.0), (double)((rand()/(RAND_MAX*1.0)) * 500.0)});

  for (int i = 0; i < 2; i++) {
    printf("q: %f %f\n", queryPoint[0], queryPoint[1]);
    Kdtree::KdTreeNodeVec result2;

    kdtree.k_nearest_neighbors(queryPoint, num_neighbors, &result2);

    print_nodes(result2);

    printf("%f %f\n",result2[0]->point[0] + 5.0, result2[0]->point[1] + 5.0 );
    Kdtree::CoordPoint pnt= {result2[0]->point[0] + 5.0, result2[0]->point[1] + 5.0};
    printf("%f %f\n", pnt[0], pnt[1] );

    kdtree.insert(pnt, i, 2.0, nullptr);
    queryPoint[0] = pnt[0];
    queryPoint[1] = pnt[1];

  }
  
  // cout << "3NNs of (" << queryPoint[0] << "," << queryPoint[1] << "):\n  ";
  print_nodes(kdtree.allTreeNodes);


  // double why= result2[10]->par_cost.load()->cost;

  // printf("%f \n",why);




  return 0;
}
