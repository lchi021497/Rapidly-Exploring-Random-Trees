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
#include <atomic> 

namespace Kdtree {

typedef std::vector<double> CoordPoint;
typedef std::vector<double> DoubleVector;

// for passing points to the constructor of kdtree
struct KdNode {
  CoordPoint point;
  void* data;
  int index;
  KdNode(const CoordPoint& p, void* d = NULL, int i = -1) {
    point = p;
    data = d;
    index = i;
  }
  KdNode() { data = NULL; }
};
typedef std::vector<KdNode> KdNodeVector;

// base function object for search predicate in knn search
// returns true when the given KdNode is an admissible neighbor
// To define an own search predicate, derive from this class
// and overwrite the call operator operator()
struct KdNodePredicate {
  virtual ~KdNodePredicate() {}
  virtual bool operator()(const KdNode&) const { return true; }
};

//--------------------------------------------------------
// private helper classes used internally by KdTree
//
// the internal node structure used by kdtree


class kdtree_node;
typedef std::vector<kdtree_node *> KdTreeNodeVec;

struct rewire_par_cost {
  double cost;
  kdtree_node *parent;
};

class kdtree_node {
 public:
  kdtree_node() {
    dataindex = cutdim = 0;
    loson = hison = (kdtree_node*)NULL;
  }
  ~kdtree_node() {
    if (loson) delete loson;
    if (hison) delete hison;
    if (par_cost) delete par_cost;

  }
  // index of node data in kdtree array "allnodes"
  size_t dataindex;
  // cutting dimension
  size_t cutdim;
  // value of point
  // double cutval; // == point[cutdim]
  CoordPoint point;
  //  roots of the two subtrees
  // kdtree_node *loson, *hison;
  std::atomic<kdtree_node*> loson, hison;

  // bounding rectangle of this node's subtree
  CoordPoint lobound, upbound;

  int index; 

  // std::atomic<rewire_par_cost*> par_cost;
  rewire_par_cost* par_cost;
};

















// base class for different distance computations
class DistanceMeasure;
// helper class for priority queue in k nearest neighbor search
class nn4heap {
 public:
  size_t dataindex;  // index of actual kdnode in *allnodes*
  double distance;   // distance of this neighbor from *point*
  nn4heap(size_t i, double d) {
    dataindex = i;
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
  CoordPoint lobound, upbound;
  // helper variable to check the distance method
  int distance_type;
  bool neighbor_search(const CoordPoint& point, kdtree_node* node, size_t k, SearchQueue* neighborheap);
  void range_search(const CoordPoint& point, kdtree_node* node, double r, std::vector<size_t>* range_result);
  bool bounds_overlap_ball(const CoordPoint& point, double dist,
                           kdtree_node* node);
  bool ball_within_bounds(const CoordPoint& point, double dist,
                          kdtree_node* node);
  // class implementing the distance computation
  DistanceMeasure* distance;
  // search predicate in knn searches
  KdNodePredicate* searchpredicate;

 public:
  KdNodeVector allnodes;
  KdTreeNodeVec allTreeNodes;

  size_t dimension;
  kdtree_node* root;
  // distance_type can be 0 (max), 1 (city block), or 2 (euklid [squared])
  KdTree(const KdNodeVector* nodes, int distance_type = 2);
  KdTree(CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type = 2);
  ~KdTree();
  void set_distance(int distance_type, const DoubleVector* weights = NULL);
  void k_nearest_neighbors(const CoordPoint& point, size_t k,
                           KdNodeVector* result, KdNodePredicate* pred = NULL);

  void k_nearest_neighbors(const CoordPoint& point, size_t k,
                           KdTreeNodeVec* result, KdNodePredicate* pred = NULL);
  void range_nearest_neighbors(const CoordPoint& point, double r,
                               KdNodeVector* result);

    void range_nearest_neighbors(const CoordPoint& point, double r,
                               KdTreeNodeVec* result);

  kdtree_node* insert(CoordPoint point, int index, double cost, kdtree_node *parent);
};

}  // end namespace Kdtree

#endif
