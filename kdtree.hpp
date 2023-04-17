#ifndef __kdtree_HPP
#define __kdtree_HPP

//
// Kd-Tree implementation.
//
// Copyright: Christoph Dalitz, 2018-2022
//            Jens Wilberg, 2018
// Version:   1.2
// License:   BSD style license
//            (see the file LICENSE for details)
//

#include <cstdlib>
#include <queue>
#include <vector>

namespace Kdtree {

typedef std::vector<double> CoordPoint;
typedef std::vector<double> DoubleVector;

// for passing points to the constructor of kdtree
// struct KdNode {
//   CoordPoint point;
//   void* data;
//   int index;
//   KdNode(const CoordPoint p, void* d = NULL, int i = -1) {
//     point = p;
//     data = d;
//     index = i;
//   }
//   KdNode() { data = NULL; }
// };
// typedef std::vector<KdNode> KdNodeVector;
class kdtree_node;
typedef std::vector<CoordPoint> CoordPointVec;
typedef std::vector<kdtree_node *> KdTreeNodeVec;
// base function object for search predicate in knn search
// returns true when the given KdNode is an admissible neighbor
// To define an own search predicate, derive from this class
// and overwrite the call operator operator()
struct KdNodePredicate {
  virtual ~KdNodePredicate() {}
  virtual bool operator()(const kdtree_node*) const { return true; }
};

//--------------------------------------------------------
// private helper classes used internally by KdTree
//
// the internal node structure used by kdtree
class kdtree_node {
 public:
  kdtree_node() {
    cutdim = 0;
    loson = hison = nullptr;
  }

  bool is_leaf() {
    return loson == nullptr && hison == nullptr;
  }
  ~kdtree_node() {
    if (loson) delete loson;
    if (hison) delete hison;
  }

  // friend std::ostream& operator<< (std::ostream& s, const kdtree_node& node) {
  //   s << node.point[0] << ", " << node.point[1];
  //   s << "\n";
  //   return s;
  // }
  // cutting dimension
  size_t cutdim;
  // value of point
  // double cutval; // == point[cutdim]
  CoordPoint point;
  //  roots of the two subtrees
  kdtree_node *loson, *hison;
  void *data;
  // bounding rectangle of this node's subtree
  CoordPoint lobound, upbound;

  int index; 

};

// base class for different distance computations
class DistanceMeasure;
// helper class for priority queue in k nearest neighbor search
class nn4heap {
 public:
  kdtree_node *node_;  // index of actual kdnode in *allnodes*
  double distance; 
    // distance of this neighbor from *point*
  nn4heap(kdtree_node *node, double d) {
    node_ = node;
    distance = d;
  }
};
class compare_nn4heap {
 public:
  bool operator()(const nn4heap& n, const nn4heap& m) {
    return (n.distance < m.distance);
  }
};
  typedef std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap> SearchQueue;
//--------------------------------------------------------

// kdtree class
class KdTree {
 private:
  // recursive build of tree
  kdtree_node* build_tree(size_t depth, size_t a, size_t b);
  // helper variable for keeping track of subtree bounding box
  CoordPoint lobound_, upbound_;
  // helper variable to check the distance method
  int distance_type;
  bool neighbor_search(const CoordPoint& point, kdtree_node* node, size_t k, SearchQueue* neighborheap, int depth = 0);
  void range_search(const CoordPoint& point, kdtree_node* node, double r, std::vector<kdtree_node *>& range_result);
  bool bounds_overlap_ball(const CoordPoint& point, double dist,
                           kdtree_node* node);
  bool ball_within_bounds(const CoordPoint& point, double dist,
                          kdtree_node* node); 
  // class implementing the distance computation
  DistanceMeasure* distance;
  // search predicate in knn searches
  KdNodePredicate* searchpredicate;

 public:
  CoordPointVec allnodes_;
  size_t dimension_;
  kdtree_node* root;
  // distance_type can be 0 (max), 1 (city block), or 2 (euklid [squared])
  KdTree(CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type = 2);
  KdTree(const CoordPointVec nodes, CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type = 2);
  ~KdTree();
  void set_distance(int distance_type, const DoubleVector* weights = NULL);
  void k_nearest_neighbors(const CoordPoint& point, size_t k,
                           KdTreeNodeVec* result, KdNodePredicate* pred = NULL);
  void range_nearest_neighbors(const CoordPoint& point, double r,
                               KdTreeNodeVec* result);
  kdtree_node* insert(const CoordPoint& point, int index);
};

}  // end namespace Kdtree

#endif