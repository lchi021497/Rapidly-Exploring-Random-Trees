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

#include <cstdio>
#include <cstdlib>
#include <queue>
#include <vector>
#include <ostream>

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
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
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

  // std::ostream &operator<<(std::ostream &output_stream) {
  //   output_stream << "Node(x=" << point[0] << ",y=" << point[1];
  //   return output_stream;
  // }

  friend std::ostream& operator<< (std::ostream& s, const kdtree_node& node) {
    s << node.point[0] << ", " << node.point[1];
    s << "\n";
    return s;
  }
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
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
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
    if (w)
      return (*w)[dim] * (x - y) * (x - y);
    else
      return (x - y) * (x - y);
  }
};


}  // end namespace Kdtree

#endif