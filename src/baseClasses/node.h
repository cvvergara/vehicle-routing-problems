/*VRP*********************************************************************
 *
 * vehicle routing problems
 *        A collection of C++ classes for developing VRP solutions
 *        and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef SRC_BASECLASSES_NODE_H_
#define SRC_BASECLASSES_NODE_H_

#include <string>
#include <basictypes.h>
#include <boost/geometry/geometry.hpp>


/*! \class Node
 * \brief The Node class defines a point in 2D space with an id.
 *
 * A Node is a point that defines a location in 2D space. It maintains
 * a user \c id and an internal \c nid along with its \c x, \c y location.
 * This is the base object that things like depots, customer locations, etc.
 * are built upon.
 *
 */

class Node {
    typedef boost::geometry::model::point
      <
          double, 2, boost::geometry::cs::spherical_equatorial
             <
                boost::geometry::degree
             >
      > spherical_point;

 public:
    /** @name accessors */
    ///@{
    inline UID nid() const {return nid_;}
    inline int id() const {return id_;}
    inline double x() const {return coordinates_.get<0>();}
    inline double y() const {return coordinates_.get<1>();}
    inline spherical_point coordinates() const {return coordinates_;}
    inline std::string hint() const {return hint_;}
    ///@}



    /** @name state */
    ///@{
    inline bool isLatLon() const {
      return (x() < 180) && (x() > -180)
             && (y() < 180) && (y() > -180);
    }
    inline bool isValid() const {return  valid_ > 0;}
    inline bool isSamePos(const Node &other) const {
       return equals(coordinates(), other.coordinates());
    } // distance(other) == 0;}
    inline bool isSamePos(const Node &other, double tolerance) const {
      return distanceToSquared(other) < tolerance;
    }
    inline bool hasHint() const {return !(hint_ == "" );}
    ///@}

    /** @name mutators */
    ///@{
    void set_nid(UID nid) {nid_ = nid;}
    void set_id(int id) {
      id_ = id;
      valid_ = true;
    }
    void set_x(double x) {coordinates_.set<0>(x);}
    void set_y(double y) {coordinates_.set<1>(y);}
    void set(double x, double y) {coordinates_ = spherical_point(x, y);}
    void set(const spherical_point &point) {coordinates_ = point;}
    void set_hint(const std::string &hint) {hint_ = hint;}
    ///@}

    /** @name operators */
    ///@{
    bool operator<(const Node &n) const {return nid() < n.nid();}
    bool operator==(const Node &node) const {
      return nid() == node.nid()
             && equals(coordinates(), node.coordinates()); //x_ == n.x_ && y_ == n.y_;
    }
    bool operator!=(const Node &n) const {return !(*this == n);}
    bool operator>(const Node &n) const {return nid() > n.nid();}
    ///@}

    /** @name vector operators */
    ///@{
    Node operator+(const Node &v) const;
    Node operator-(const Node &v) const;
    Node operator*(double f) const;
    double dotProduct(const Node &p) const;
    ///@}

    /** @name distances */
    ///@{
    double distance(const Node &n) const;
    double haversineDistance(const Node &n) const;
    double distanceToSquared(const Node &p) const;
    ///@}

    /** @name distanceToSegment */
    /* Shortest distnace from point to a segment (v,w) */
    ///@{
    double distanceToSegment(const Node &v, const Node &w) const;
    double distanceToSegment(const Node &v, const Node &w, Node &q) const;

    ///@}

    double positionAlongSegment(const Node &v, const Node &w, double tol) const;

    // dump
    void dump() const;

    // constructors
    Node() = default;
    Node(const Node&) = default;
    Node(double x, double y);
 private:
    UID nid_;    ///< internal node number
    UID id_;     ///< user supplied node number
    std::string hint_;  ///< orsrm's hint
    bool valid_;  ///< true when id_ has being assigned
    spherical_point coordinates_;  ///< for earth
};

#endif  // SRC_BASECLASSES_NODE_H_
