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
#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

/*! \class Twnode
 * \brief Extends the \ref Node class to create a Node with time window attributes.
 *
 * A Time Window node is a Node with addition attributes and methods to
 * to support Time Windows and to model a more complex Node need in many
 * vehicle routing problems.
 *
 * Most application specific code will extend this class and define the specific
 * values and requirements for \c type and \c streetid.
 *
 * Currently the trash collection problem is using node type values of:
 * - -1: Invalid
 * - 0: Depot or Start location
 * - 1: Dump site
 * - 2: Pickup location
 * - 3: End site
 * Currently the pick & delivery problem is using node type values of:
 * - -1: Invalid
 * - 0: Depot or Start location also as ending site
 * - was using 1 as delivery, but the trash collection problem is more urgent
 * \todo use something else as deliver TODO
 * - 2: Pickup location
 */
class Twnode: public Node {
  protected:
    int    type;        ///< Defines what type of Twnode
    double demand;      ///< The demand for the Node
    double tw_open;     ///< When the time window opens (earliest arrival time)
    double tw_close;    ///< When the time window closes (latest arrival time)
    double serviceTime; ///< The length of time it takes to service the Node
    int streetid;       ///< The street id that the node is on (might be optional)

  public:
    // accessors

    /*!
     * \brief Get the time window open time.
     * \return The earliest arrival time.
     */
    double opens() const {return tw_open;};

    /*!
     * \brief Get the time window close time.
     * \return The latest arrival time.
     */
    double closes() const {return tw_close;};

    /*!
     * \brief Get the demand associated with this node.
     * \return The demand for this node.
     */
    double getDemand() const {return demand;};

    /*!
     * \brief Get the service time for this node.
     * \return The service time for this node.
     */
    double getServiceTime() const { return serviceTime;};

    /*!
     * \brief Get the length of time between the time window open and close.
     * \return the length of time that the time window is open.
     */
    double windowLength() const { return  tw_close - tw_open; };

    /*!
     * \brief Get the type of node this is. -1 is Invalid or undefined. Other values are defined by the application.
     * \return The type of node.
     */
    int ntype() const {return type;};

    /*!
     * \brief Get the street id or -1 if it is not defined.
     * \return The street id.
     */
    int streetId() const {return streetid;};


    /*! \brief True when node is depot*/
    bool isDepot() const {return type == 0;};
    bool isStarting() const {return type == 0;};
    bool isDump() const {return type == 1;};
    bool isPickup() const {return type == 2;};
    bool isEnding() const {return type == 3;};
    bool isValid() const;


    // doc in cpp
    void dump() const;

    // state

    // doc in cpp
    bool isvalid() const;

    /*!
     * \brief Determine whether or not the \ref Twnode has demand.
     * \return true if the node has positive demand.
     */
    bool hasDemand() const { return demand > 0; };

    /*!
     * \brief Determine whether or not the \ref Twnode as supply (ie: negative demand).
     * \return true if the node has negative demand
     */
    bool hasSupply() const { return demand < 0; };

    /*!
     * \brief Determine whether or not the \ref Twnode is empty (ie: demand equals zero).
     * \return true if the node has zero demand.
     */
    bool hasNoGoods() const { return demand == 0; };

    /*!
     * \brief Determine if \b arrivalTime is before \b tw_open
     * \param[in] arrivalTime the time we expect to arrive at this node
     * \return true if \b arrivalTime is before \b tw_open
     */
    bool earlyArrival( const double arrivalTime ) const { return arrivalTime < tw_open; };

    /*!
     * \brief Determine if \b arrivalTime is after \b tw_close
     * \param[in] arrivalTime the time we expect to arrive at this node
     * \return true if \b arrivalTime is after \b tw_close
     */
    bool lateArrival( const double arrivalTime ) const { return arrivalTime > tw_close; };

    /*!
     * \brief Check if this node is on the same street as another node
     * \param[in] other The other node we want to chack against
     * \return true if the street ids match
     */
    bool sameStreet ( const Twnode &other ) const {return streetid == other.streetid; };

    // mutators

    // doc in cpp
    void set( int _nid, int _id, double _x, double _y, int _demand,
              int _tw_open, int _tw_close, int _service );

    /*!
     * \brief Set the demand for this node.
     * \param[in] _demand The demand to assign to this node.
     */
    void setDemand( int _demand ) { demand = _demand; };

    /*!
     * \brief Set the type of this node. -1 is Invalid or undefined, other values are established by the application.
     * \param[in] _type The type code for this node.
     */
    void setType( int _type ) { type = _type; };

    /*!
     * \brief Set the time that the time window opens.
     * \param[in] _tw_open The earliest arrival time for this node.
     */
    void setOpens( int _tw_open ) { tw_open = _tw_open; };

    /*!
     * \brief Set the time that the time window closes.
     * \param[in] _tw_close The latest arrival time for this node.
     */
    void setCloses( int _tw_close ) { tw_close = _tw_close; };

    /*!
     * \brief Set the service time for this node.
     * \param[in] _service The service time to assign to this node.
     */
    void setServiceTime( int _service ) { serviceTime = _service; };

    /*!
     * \brief Set the street id for this node.
     * \param[in] _sid The street id to assign to this node.
     */
    void setStreetId( int _sid ) { streetid = _sid; };

    // structors

    /*!
     * \brief Construct an undefined Twnode object.
     */
    Twnode() {
        Node();
        type = -1;
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        serviceTime = 0;
    };

    // doc in cpp
    Twnode( std::string line );

    ~Twnode() {};

};
#endif