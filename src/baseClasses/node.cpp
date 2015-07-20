/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>


#ifdef DOVRPLOG
#include "./logger.h"
#endif

#include "./node.h"
// for distance_to_segment
#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>



/*!
 * \brief Compute the Haversine spherical distance between two nodes.
 *
 * Haversine spherical distance between two nodes with lat/lon values when
 * the nodes x,y is loaded with longitude,latitude values.
 *
 */


double Node::haversineDistance(const Node &other) const {
  // TODO (cvvc) not completly sure this is the correct call for harversine, but I am leaving it for later
  return boost::geometry::distance(this->coordinates_, other.coordinates_);
}

double Node::distance(const Node &other) const {
  assert(isLatLon() && other.isLatLon());
  return haversineDistance(other);
}




/*!  * \brief Print the contents of this node.  */
void Node::dump() const {
#ifdef DOVRPLOG
  DLOG(INFO) << nid()
             << ", " << x()
             << ", " << y()
             << ", " << hint();
#endif
}

// Vector Operations

/*! \brief Create a new Node by performing vector addition.  */
Node Node::operator+(const Node &other) const {
     Node node(*this);
     boost::geometry::add_point(node.coordinates_, other.coordinates_);
     return node;
}

/*! \brief Create a new Node by performing vector subtraction.  */
Node  Node::operator-(const Node &other) const {
     Node node(*this);
     boost::geometry::subtract_point(node.coordinates_, other.coordinates_);
     return node;
}

/*! \brief Create a new Node by scaling and existing node by a factor \c f.*/
  Node Node::operator*(double val) const {
     Node node(*this);
     boost::geometry::multiply_value(node.coordinates_, val);
     return node;
  }

/*! \brief Compute the vector dot product between two Nodes.*/
  double Node::dotProduct( const Node &other ) const {
    return boost::geometry::dot_product(this->coordinates_, other.coordinates_);
  }



/*!  \brief Compute the Euclidean distance squared between two Nodes.
 *
 * \sa Node::length, Node::distanceTo, Node::distance
 */
double Node::distanceToSquared(const Node &other) const {
  return boost::geometry::comparable_distance(this->coordinates_, other.coordinates_);
}

/*! \brief Compute the shortest distance from a Node to a line segment*/
double Node::distanceToSegment(const Node &v, const Node &w) const {
#if 0  // TODO: Not working yet
  typedef boost::geometry::strategy::distance::projected_point< double, haversine<double> > strategy_type;
  strategy_type strategy;
  strategy_type::calculation_type d2 = strategy.apply(this->coordinates_, v.coordinates_, w.coordinates_);
  return d2;
  return boost::geometry::distance(this->coordinates_, v.coordinates_, w.coordinates_);
#endif 
  Node q;
  return distanceToSegment(v, w, q);
}

/*! \brief Compute the shortest distance from a Node to a line segment */
double Node::distanceToSegment(const Node &v, const Node &w, Node &q) const {
  // i.e. |w-v|^2 ... avoid a sqrt
  double distSq = v.distanceToSquared(w);

  if (distSq == 0.0) {  // v == w case
    q = v;
    return distance(v);
  }

  // consider the line extending the segment, parameterized as v + t (w - v)
  // we find projection of point p onto the line
  // it falls where t = [(p-v) . (w-v)] / |w-v|^2

  double t = ((*this) - v).dotProduct(w - v) / distSq;

  if ( t < 0.0 ) {  // beyond the v end of the segment
    q = v;
    return distance(v);
  }

  if ( t > 1.0 ) {  // beyond the w end of the segment
    q = w;
    return distance(w);
  }

  // projection falls on the segment
  Node projection = v + ((w - v) * t);

  q = projection;

  return distance(projection);
}

/*! \bref Compute position along the line segment
 *  Check if the node is on the line segment and return -1.0 if distance
 *  exceeds tol. Otherwise project the node onto the line segment and
 *  return is position as a percentage.
 *  \param[in] v start of segment
 *  \param[in] w end of segment
 *  \param[in] tol tolerance for distance test
 *  \return position 0.0 to 1.0 along the segment of -1.0 if it is not within tolerance
*/
double Node::positionAlongSegment(const Node &v, const Node &w, double tol) const {
  double tolSq = tol * tol;

  // i.e. |w-v|^2 ... avoid a sqrt
  double distSq = v.distanceToSquared(w);

  if (distSq == 0.0) {  // v == w case
    if (distanceToSquared(v) < tolSq)
      return 0.0;       // node == v == w case
    else
      return -1.0;      // node is not within tol
  }

  // consider the line extending the segment, parameterized as v + t (w - v)
  // we find projection of point p onto the line
  // it falls where t = [(p-v) . (w-v)] / |w-v|^2

  double t = ((*this) - v).dotProduct(w - v) / distSq;

  // beyond the v end of the segment
  if ( t < 0.0 ) {
      return -1.0;
  }

  // beyond the w end of the segment
  if ( t > 1.0 ) {
      return -1.0;
  }

  // projection falls on the segment
  Node projection = v + ((w - v) * t);

  if (distanceToSquared(projection) > tolSq )
    return -1.0;

  return t;
}

#if 0
#endif

// Constructors

/*! \brief Construct a new Node that needs the user to set its attributes.  */
#if 0
Node::Node()
  : nid_(0), id_(0), x_(0.0), y_(0.0), hint_(""), valid_(false) {
}
#endif

/*! \brief Construct a new Node and assign it \c x and \c y values.  */
Node::Node(double x, double y)
  : nid_(0), id_(0),
    hint_(""), valid_(false), coordinates_(x, y){
}

