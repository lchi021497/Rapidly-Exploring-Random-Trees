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
  bool operator()(const KdNode& p, const KdNode& q) {
    return (p.point[d] < q.point[d]);
  }
  size_t d;
};

//--------------------------------------------------------------
// internal node structure used by kdtree
//--------------------------------------------------------------
// class kdtree_node {
//  public:
//   kdtree_node() {
//     dataindex = cutdim = 0;
//     loson = hison = (kdtree_node*)NULL;
//   }
//   ~kdtree_node() {
//     if (loson) delete loson;
//     if (hison) delete hison;
//   }
//   // index of node data in kdtree array "allnodes"
//   size_t dataindex;
//   // cutting dimension
//   size_t cutdim;
//   // value of point
//   // double cutval; // == point[cutdim]
//   CoordPoint point;
//   //  roots of the two subtrees
//   kdtree_node *loson, *hison;
//   // bounding rectangle of this node's subtree
//   CoordPoint lobound, upbound;
// };

//--------------------------------------------------------------
// different distance metrics
//--------------------------------------------------------------
class DistanceMeasure {
 public:
  DistanceMeasure() {}
  virtual ~DistanceMeasure() {}
  virtual double distance(const CoordPoint& p, const CoordPoint& q) = 0;
  virtual double coordinate_distance(double x, double y, size_t dim) = 0;
};
// Maximum distance (Linfinite norm)
class DistanceL0 : virtual public DistanceMeasure {
  DoubleVector* w;

 public:
  DistanceL0(const DoubleVector* weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector*)NULL;
  }
  ~DistanceL0() {
    if (w) delete w;
  }
  double distance(const CoordPoint& p, const CoordPoint& q) {
    size_t i;
    double dist, test;
    if (w) {
      dist = (*w)[0] * fabs(p[0] - q[0]);
      for (i = 1; i < p.size(); i++) {
        test = (*w)[i] * fabs(p[i] - q[i]);
        if (test > dist) dist = test;
      }
    } else {
      dist = fabs(p[0] - q[0]);
      for (i = 1; i < p.size(); i++) {
        test = fabs(p[i] - q[i]);
        if (test > dist) dist = test;
      }
    }
    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) {
    if (w) {
      return (*w)[dim] * fabs(x - y);
    }
    else {
      return fabs(x - y);
    }
  }
};
// Manhatten distance (L1 norm)
class DistanceL1 : virtual public DistanceMeasure {
  DoubleVector* w;

 public:
  DistanceL1(const DoubleVector* weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector*)NULL;
  }
  ~DistanceL1() {
    if (w) delete w;
  }
  double distance(const CoordPoint& p, const CoordPoint& q) {
    size_t i;
    double dist = 0.0;
    if (w) {
      for (i = 0; i < p.size(); i++) dist += (*w)[i] * fabs(p[i] - q[i]);
    } else {
      for (i = 0; i < p.size(); i++) dist += fabs(p[i] - q[i]);
    }
    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) {
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
  }
};
// Euklidean distance (L2 norm) (squared)
class DistanceL2 : virtual public DistanceMeasure {
  DoubleVector* w;

 public:
  DistanceL2(const DoubleVector* weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector*)NULL;
  }
  ~DistanceL2() {
    if (w) delete w;
  }
  double distance(const CoordPoint& p, const CoordPoint& q) {
    size_t i;
    double dist = 0.0;
    if (w) {
      for (i = 0; i < p.size(); i++)
        dist += (*w)[i] * (p[i] - q[i]) * (p[i] - q[i]);
    } else {
      for (i = 0; i < p.size(); i++) dist += (p[i] - q[i]) * (p[i] - q[i]);
    }

    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) {
    if (w) {
      return (*w)[dim] * (x - y) * (x - y);
    }
    else {
      return (x - y) * (x - y);
    }
  }
};

//--------------------------------------------------------------
// destructor and constructor of kdtree
//--------------------------------------------------------------
KdTree::~KdTree() {
  if (root) delete root;
  delete distance;
}

KdTree::KdTree(CoordPoint min_point, CoordPoint max_point, size_t dimension, int distance_type /*=2*/) {
  
  root = nullptr;
  allnodes = {};

  // this->lobound = min_point;
  // this->upbound = max_point;
  this->dimension = dimension;

  distance = NULL;
  this->distance_type = -1;
  set_distance(distance_type);
}

// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
KdTree::KdTree(const KdNodeVector* nodes, int distance_type /*=2*/) {
  size_t i, j;
  double val;
  // copy over input data
  if (!nodes || nodes->empty())
    throw std::invalid_argument(
        "kdtree::KdTree(): argument nodes must not be empty");
  dimension = nodes->begin()->point.size();
  allnodes = *nodes;
  // initialize distance values
  distance = NULL;
  this->distance_type = -1;
  set_distance(distance_type);
  // compute global bounding box
  lobound = nodes->begin()->point;
  upbound = nodes->begin()->point;
  for (i = 1; i < nodes->size(); i++) {
    for (j = 0; j < dimension; j++) {
      val = allnodes[i].point[j];
      if (lobound[j] > val) lobound[j] = val;
      if (upbound[j] < val) upbound[j] = val;
    }
  }
  // build tree recursively
  root = build_tree(0, 0, allnodes.size());
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
  node->lobound = lobound;
  node->upbound = upbound;
  node->cutdim = depth % dimension;
  if (b - a <= 1) {
    node->dataindex = a;
    node->point = allnodes[a].point;

    allnodes[a].upbound = upbound;
    allnodes[a].lobound = lobound;
  } else {
    m = (a + b) / 2;
    std::nth_element(allnodes.begin() + a, allnodes.begin() + m,
                     allnodes.begin() + b, compare_dimension(node->cutdim));
    node->point = allnodes[m].point;
    cutval = allnodes[m].point[node->cutdim];
    allnodes[m].upbound = upbound;
    allnodes[m].lobound = lobound;
    node->dataindex = m;
    if (m - a > 0) {
      temp = upbound[node->cutdim];
      upbound[node->cutdim] = cutval;
      node->loson = build_tree(depth + 1, a, m);
      upbound[node->cutdim] = temp;
    }
    if (b - m > 1) {
      temp = lobound[node->cutdim];
      lobound[node->cutdim] = cutval;
      node->hison = build_tree(depth + 1, m + 1, b);
      lobound[node->cutdim] = temp;
    }
  }
  return node;
}

kdtree_node* KdTree::insert(CoordPoint point, int index, double cost, kdtree_node *parent) {
  double cutval;
  kdtree_node *ptr = root;

  kdtree_node* node = new kdtree_node();
  node->upbound = upbound;
  node->lobound = lobound;

  node->point = point;
  node->index = index;
  node->dataindex = index;
  node->par_cost = new rewire_par_cost();

  node->par_cost.load()->cost = cost; 
  node->par_cost.load()->parent = parent;

  if (root == nullptr) {

    lobound = {0,0};
    upbound = {800,600};
    node->lobound = lobound;
    node->upbound = upbound;
    // printf("%f %f %f %f", lobound[0], lobound[1], upbound[0], upbound[1]);

  
    node->cutdim = 0;
    root = node;
    allnodes.push_back(Kdtree::KdNode(point, NULL, index));
    allTreeNodes.push_back(node);
    // printf("null");
    return node;
  }

  assert(root != nullptr);
  int depth = 1;
  // iterate through the kd tree to find position to insert
  while (true) {

    size_t cutdim = depth%dimension;
    cutval = ptr->point[cutdim];

    if (lobound[cutdim] > cutval)
      lobound[cutdim] = cutval;
    if (upbound[cutdim] < cutval)
      upbound[cutdim] = cutval;

    int try_again = 0;
    if (node->point[cutdim] < cutval) {
      // go left

      node->upbound = ptr->upbound;
      node->lobound = ptr->lobound;


      node->upbound[ptr->cutdim] = cutval;

      if (ptr->loson == nullptr) {
        // ptr->loson = node;
        node->cutdim = depth%dimension;
        kdtree_node *tmp = nullptr;
        if ((ptr->loson).compare_exchange_weak(tmp, node)) {
          break;
        }        
        else {
          try_again = 1;
        }
        // break;
      }
      ptr = ptr->loson;
      // printf("left\n");
    } else {

      node->upbound = ptr->upbound;
      node->lobound = ptr->lobound;

      node->lobound[ptr->cutdim] = cutval;
      // node->upbound[cutdim] = point[cutdim];

      if (ptr->hison == nullptr) {
        // ptr->hison = node;
        node->cutdim = depth%dimension;
        kdtree_node *tmp = nullptr;
        if ((ptr->hison).compare_exchange_weak(tmp, node)) {
          break;
        }
        else {
          try_again = 1;
        }
        // break;
      }
      ptr = ptr->hison;
      // printf("right\n");
    }
    if (try_again)
      try_again = 0;
    else 
      depth++;
  }

  allnodes.push_back(Kdtree::KdNode(point, NULL, index));
  allTreeNodes.push_back(node);
  
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
                                 KdNodeVector* result,
                                 KdNodePredicate* pred /*=NULL*/) {
  size_t i;
  KdNode temp;
  searchpredicate = pred;

  result->clear();
  if (k < 1) return;
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");

  // collect result of k values in neighborheap
  //std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>*
  //neighborheap = new std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>();
  SearchQueue* neighborheap = new SearchQueue();
  if (k > allnodes.size()) {
    // when more neighbors asked than nodes in tree, return everything
    k = allnodes.size();
    for (i = 0; i < k; i++) {
      if (!(searchpredicate && !(*searchpredicate)(allnodes[i])))
        neighborheap->push(
            nn4heap(i, distance->distance(allnodes[i].point, point)));
    }
  } else {
    neighbor_search(point, root, k, neighborheap);
  }

  // copy over result sorted by distance
  // (we must revert the vector for ascending order)
  while (!neighborheap->empty()) {
    i = neighborheap->top().dataindex;
    neighborheap->pop();
    result->push_back(allnodes[i]);
  }
  // beware that less than k results might have been returned
  k = result->size();
  for (i = 0; i < k / 2; i++) {
    temp = (*result)[i];
    (*result)[i] = (*result)[k - 1 - i];
    (*result)[k - 1 - i] = temp;
  }
  delete neighborheap;
}

void KdTree::k_nearest_neighbors(const CoordPoint& point, size_t k,
                                 KdTreeNodeVec* result,
                                 KdNodePredicate* pred /*=NULL*/) {
  size_t i;
  kdtree_node* temp;
  searchpredicate = pred;

  result->clear();
  if (k < 1) return;
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");

  // collect result of k values in neighborheap
  //std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>*
  //neighborheap = new std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>();
  SearchQueue* neighborheap = new SearchQueue();
  if (k > allnodes.size()) {
    // when more neighbors asked than nodes in tree, return everything
    k = allnodes.size();
    for (i = 0; i < k; i++) {
      if (!(searchpredicate && !(*searchpredicate)(allnodes[i])))
        neighborheap->push(
            nn4heap(i, distance->distance(allnodes[i].point, point)));
    }
  } else {
    neighbor_search(point, root, k, neighborheap);
  }

  // copy over result sorted by distance
  // (we must revert the vector for ascending order)
  while (!neighborheap->empty()) {
    i = neighborheap->top().dataindex;
    neighborheap->pop();
    result->push_back(allTreeNodes[i]);
    // printf("%d\n", i);
  }
  // beware that less than k results might have been returned
  // k = result->size();
  // for (i = 0; i < k / 2; i++) {
  //   temp = (*result)[i];
  //   (*result)[i] = (*result)[k - 1 - i];
  //   (*result)[k - 1 - i] = temp;
  // }
  delete neighborheap;
}


//--------------------------------------------------------------
// range nearest neighbor search
// returns the nearest neighbors of *point* in the given range
// *r*. The result is returned in *result* and is sorted by
// distance from *point*.
//--------------------------------------------------------------
void KdTree::range_nearest_neighbors(const CoordPoint& point, double r,
                                     KdNodeVector* result) {
  KdNode temp;

  result->clear();
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");
  if (this->distance_type == 2) {
    // if euclidien distance is used the range must be squared because we
    // get squared distances from this implementation
    r *= r;
  }

  // collect result in range_result
  std::vector<size_t> range_result;
  range_search(point, root, r, &range_result);

  // copy over result
  for (std::vector<size_t>::iterator i = range_result.begin();
       i != range_result.end(); ++i) {
    result->push_back(allnodes[*i]);
  }

  // clear vector
  range_result.clear();
}

void KdTree::range_nearest_neighbors(const CoordPoint& point, double r,
                                     KdTreeNodeVec* result) {
  KdNode temp;

  result->clear();
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");
  if (this->distance_type == 2) {
    // if euclidien distance is used the range must be squared because we
    // get squared distances from this implementation
    r *= r;
  }

  // collect result in range_result
  std::vector<size_t> range_result;
  range_search(point, root, r, &range_result);

  // copy over result
  for (std::vector<size_t>::iterator i = range_result.begin();
       i != range_result.end(); ++i) {
    result->push_back(allTreeNodes[*i]);
  }

  // clear vector
  range_result.clear();
}

//--------------------------------------------------------------
// recursive function for nearest neighbor search in subtree
// under *node*. Stores result in *neighborheap*.
// returns "true" when no nearer neighbor elsewhere possible
//--------------------------------------------------------------
bool KdTree::neighbor_search(const CoordPoint& point, kdtree_node* node,
                             size_t k, SearchQueue* neighborheap) {
  double curdist, dist;


  curdist = distance->distance(point, node->point);

  // printf("%f %f %f %f %f \n", point[0], point[1], node->point[0], node->point[1], curdist);
  if (!(searchpredicate && !(*searchpredicate)(allnodes[node->dataindex]))) {
    if (neighborheap->size() < k) {
      // printf("%d %f\n", k, curdist);

      neighborheap->push(nn4heap(node->dataindex, curdist));
      // printf("%f %f\n", curdist, neighborheap->top().distance);

    } else if (curdist < neighborheap->top().distance) {
      // printf("here3");
      
      neighborheap->pop();
      neighborheap->push(nn4heap(node->dataindex, curdist));
      // printf("%f %f\n", curdist, neighborheap->top().distance);
    }
  }
  // first search on side closer to point
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->loson)
      if (neighbor_search(point, node->loson, k, neighborheap)) return true;
  } else {
    if (node->hison)
      if (neighbor_search(point, node->hison, k, neighborheap)) return true;
  }
  // second search on farther side, if necessary
  if (neighborheap->size() < k) {
    dist = std::numeric_limits<double>::max();
  } else {
    dist = neighborheap->top().distance;
  }
  // printf("%f\n", dist);
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->hison)// && bounds_overlap_ball(point, dist, node->hison))
      if (neighbor_search(point, node->hison, k, neighborheap)) return true;
  } else {
    if (node->loson)// && bounds_overlap_ball(point, dist, node->loson))
      if (neighbor_search(point, node->loson, k, neighborheap)) return true;
  }
  return false;
  // if (neighborheap->size() == k) dist = neighborheap->top().distance;
  // return ball_within_bounds(point, dist, node);

}

//--------------------------------------------------------------
// recursive function for range search in subtree under *node*.
// Stores result in *range_result*.
//--------------------------------------------------------------
void KdTree::range_search(const CoordPoint& point, kdtree_node* node,
                          double r, std::vector<size_t>* range_result) {
  double curdist = distance->distance(point, node->point);
  if (curdist <= r) {
    range_result->push_back(node->dataindex);
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
  for (i = 0; i < dimension; i++) {
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
  for (i = 0; i < dimension; i++) {
    // printf("this %f %f %f %f %f %f\n", point[i], node->lobound[i], node->upbound[i], distance->coordinate_distance(point[i], node->lobound[i], i), distance->coordinate_distance(point[i], node->upbound[i], i), dist );

    if (distance->coordinate_distance(point[i], node->lobound[i], i) <= dist ||
        distance->coordinate_distance(point[i], node->upbound[i], i) <= dist)
      return false;
  }



  // printf("here");
  return true;
}

}  // namespace Kdtree
