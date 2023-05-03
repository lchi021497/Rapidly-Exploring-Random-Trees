//
// Kd-Tree implementation.
//
// Copyright: Christoph Dalitz, 2018
//            Jens Wilberg, 2018
// Version:   1.2
// License:   BSD style license
//            (see the file LICENSE for details)
//

#include "kdtree.hpp"
#include <math.h>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <cassert>

namespace Kdtree {

//--------------------------------------------------------------
// function object for comparing only dimension d of two vecotrs
//--------------------------------------------------------------
class compare_dimension {
 public:
  compare_dimension(size_t dim) { d = dim; }
  bool operator()(const CoordPoint& p, const CoordPoint& q) {
    return (p[d] < q[d]);
  }
  size_t d;
};


//--------------------------------------------------------------
// destructor and constructor of kdtree
//--------------------------------------------------------------
KdTree::~KdTree() {
  if (root) delete root;
  delete distance;
}
// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
KdTree::KdTree(CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type /*=2*/) : lobound_(std::move(min_point)), upbound_(std::move(max_point)), dimension_(dimension) {
    // initialize distance values
  distance = NULL;
  this->distance_type = -1;
  set_distance(distance_type);
  root = NULL;
}

KdTree::KdTree(const CoordPointVec nodes, CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type /*=2*/) : KdTree(min_point, max_point, dimension, distance_type) {
  allnodes_ = nodes;
  // build tree recursively
  root = build_tree(0, 0, allnodes_.size());
}

// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
void KdTree::set_distance(int distance_type,
                          const DoubleVector* weights /*=NULL*/) {
  if (distance) delete distance;
  this->distance_type = distance_type;
  if (distance_type == 0) {
    distance = (DistanceMeasure*)new DistanceL0(weights);
  } else if (distance_type == 1) {
    distance = (DistanceMeasure*)new DistanceL1(weights);
  } else {
    distance = (DistanceMeasure*)new DistanceL2(weights);
  }
}

//--------------------------------------------------------------
// recursive build of tree
// "a" and "b"-1 are the lower and upper indices
// from "allnodes" from which the subtree is to be built
//--------------------------------------------------------------
kdtree_node* KdTree::build_tree(size_t depth, size_t a, size_t b) {
  size_t m;
  double temp, cutval;
  kdtree_node* node = new kdtree_node();
  node->lobound = lobound_;
  node->upbound = upbound_;
  node->cutdim = depth % dimension_;
  if (b - a <= 1) {
    node->point = allnodes_[a];
  } else {
    m = (a + b) / 2;
    std::nth_element(allnodes_.begin() + a, allnodes_.begin() + m,
                     allnodes_.begin() + b, compare_dimension(node->cutdim));
    node->point = allnodes_[m];
    printf("build node %f, %f\n", node->point[0], node->point[1]);
    cutval = allnodes_[m][node->cutdim];
    if (m - a > 0) {
      temp = upbound_[node->cutdim];
      upbound_[node->cutdim] = cutval;
      node->loson = build_tree(depth + 1, a, m);
      upbound_[node->cutdim] = temp;
    }
    if (b - m > 1) {
      temp = lobound_[node->cutdim];
      lobound_[node->cutdim] = cutval;
      node->hison = build_tree(depth + 1, m + 1, b);
      lobound_[node->cutdim] = temp;
    }
  }
  return node;
}

kdtree_node* KdTree::insert(const CoordPoint& point, int index, double cost, kdtree_node *parent) {
  size_t m;
  double temp, cutval;
  kdtree_node *ptr = root;

  kdtree_node* node = new kdtree_node();
  node->lobound = lobound_;
  node->upbound = upbound_;
  node->point = point;
  node->index = index;
  node->par_cost = new rewire_par_cost();

  node->par_cost.load()->cost = cost; 
  node->par_cost.load()->parent = parent;

  if (root == nullptr) {
    root = node;
    return node;
  }

  assert(root != nullptr);
  int depth = 0;
  // iterate through the kd tree to find position to insert
  while (true) {
    node->cutdim = depth % dimension_;

    cutval = ptr->point[node->cutdim];

    int try_again = 0;
    if (node->point[node->cutdim] < cutval) {
      // go left

      node->upbound[node->cutdim] = cutval;
      if (ptr->loson == nullptr) {
        // ptr->loson = node;

        kdtree_node *tmp = nullptr;
        if ((ptr->loson).compare_exchange_weak(tmp, node, std::memory_order_release, std::memory_order_relaxed)) {
          break;
        }        
        else {
          try_again = 1;
        }
      }
      ptr = ptr->loson;
      // printf("left\n");
    } else {
      node->lobound[node->cutdim] = cutval;
      if (ptr->hison == nullptr) {
        // ptr->hison = node;

        kdtree_node *tmp = nullptr;
        if ((ptr->hison).compare_exchange_weak(tmp, node, std::memory_order_release, std::memory_order_relaxed)) {
          break;
        }
        else {
          try_again = 1;
        }
        
      }
      ptr = ptr->hison;
      // printf("right\n");
    }
    if (try_again)
      try_again = 0;
    else 
      depth++;
  }
  
  return node;
}



//--------------------------------------------------------------
// k nearest neighbor search
// returns the *k* nearest neighbors of *point* in O(log(n))
// time. The result is returned in *result* and is sorted by
// distance from *point*.
// The optional search predicate is a callable class (aka "functor")
// derived from KdNodePredicate. When Null (default, no search
// predicate is applied).
//--------------------------------------------------------------
void KdTree::k_nearest_neighbors(const CoordPoint& point, size_t k,
                                 KdTreeNodeVec* result,
                                 KdNodePredicate* pred /*=NULL*/) {
  size_t i;
  searchpredicate = pred;

  result->clear();
  if (k < 1) return;
  if (point.size() != dimension_)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");

  // collect result of k values in neighborheap
  //std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>*
  //neighborheap = new std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>();
  SearchQueue* neighborheap = new SearchQueue();
  
  neighbor_search(point, root, k, neighborheap);

  // copy over result sorted by distance
  // (we must revert the vector for ascending order)
  while (!neighborheap->empty()) {
    auto nn = neighborheap->top();
    neighborheap->pop();
    result->push_back(nn.node_); 
  }
  std::reverse(result->begin(), result->end());
}

//--------------------------------------------------------------
// range nearest neighbor search
// returns the nearest neighbors of *point* in the given range
// *r*. The result is returned in *result* and is sorted by
// distance from *point*.
//--------------------------------------------------------------
void KdTree::range_nearest_neighbors(const CoordPoint& point, double r,
                                     KdTreeNodeVec* result) {
  result->clear();
  if (point.size() != dimension_)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");
  if (this->distance_type == 2) {
    // if euclidien distance is used the range must be squared because we
    // get squared distances from this implementation
    r *= r;
  }

  // collect result in range_result
  range_search(point, root, r, *result);
}

//--------------------------------------------------------------
// recursive function for nearest neighbor search in subtree
// under *node*. Stores result in *neighborheap*.
// returns "true" when no nearer neighbor elsewhere possible
//--------------------------------------------------------------
bool KdTree::neighbor_search(const CoordPoint& point, kdtree_node* node,
                             size_t k, SearchQueue* neighborheap, int depth) {
  double curdist, dist;
  // printf("depth: %d\n", depth);
  // printf("node: %f, %f\n", node->point[0], node->point[1]);

  curdist = distance->distance(point, node->point);
  if (!(searchpredicate && !(*searchpredicate)(node))) {
    if (neighborheap->size() < k) {
      neighborheap->push(nn4heap(node, curdist));
    } else if (curdist < neighborheap->top().distance) {
      neighborheap->pop();
      neighborheap->push(nn4heap(node, curdist));
    }
  }
  // first search on side closer to point
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->loson)
      if (neighbor_search(point, node->loson, k, neighborheap, depth+ 1)) return true;
  } else {
    if (node->hison)
      if (neighbor_search(point, node->hison, k, neighborheap, depth + 1)) return true;
  }
  // second search on farther side, if necessary
  if (neighborheap->size() < k) {
    dist = std::numeric_limits<double>::max();
  } else {
    dist = neighborheap->top().distance;
  }
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->hison && bounds_overlap_ball(point, dist, node->hison))
      if (neighbor_search(point, node->hison, k, neighborheap, depth + 1)) return true;
  } else {
    if (node->loson && bounds_overlap_ball(point, dist, node->loson))
      if (neighbor_search(point, node->loson, k, neighborheap, depth + 1)) return true;
  }

  if (neighborheap->size() == k) dist = neighborheap->top().distance;
  return ball_within_bounds(point, dist, node);
}

//--------------------------------------------------------------
// recursive function for range search in subtree under *node*.
// Stores result in *range_result*.
//--------------------------------------------------------------
void KdTree::range_search(const CoordPoint& point, kdtree_node* node,
                          double r, std::vector<kdtree_node *>& range_result) {
  double curdist = distance->distance(point, node->point);
  if (curdist <= r) {
    range_result.push_back(node);
  }
  if (node->loson != NULL && this->bounds_overlap_ball(point, r, node->loson)) {
    range_search(point, node->loson, r, range_result);
  }
  if (node->hison != NULL && this->bounds_overlap_ball(point, r, node->hison)) {
    range_search(point, node->hison, r, range_result);
  }
}

// returns true when the bounds of *node* overlap with the
// ball with radius *dist* around *point*
bool KdTree::bounds_overlap_ball(const CoordPoint& point, double dist,
                                 kdtree_node* node) {
  double distsum = 0.0;
  size_t i;
  for (i = 0; i < dimension_; i++) {
    if (point[i] < node->lobound[i]) {  // lower than low boundary
      distsum += distance->coordinate_distance(point[i], node->lobound[i], i);
      if (distsum > dist) return false;
    } else if (point[i] > node->upbound[i]) {  // higher than high boundary
      distsum += distance->coordinate_distance(point[i], node->upbound[i], i);
      if (distsum > dist) return false;
    }
  }
  return true;
}

// returns true when the bounds of *node* completely contain the
// ball with radius *dist* around *point*
bool KdTree::ball_within_bounds(const CoordPoint& point, double dist,
                                kdtree_node* node) {
  size_t i;
  for (i = 0; i < dimension_; i++)
    if (distance->coordinate_distance(point[i], node->lobound[i], i) <= dist ||
        distance->coordinate_distance(point[i], node->upbound[i], i) <= dist)
      return false;
  return true;
}

}  // namespace Kdtree