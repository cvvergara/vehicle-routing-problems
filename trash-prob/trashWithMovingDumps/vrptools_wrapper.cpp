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

#include <exception>
#include <string.h>

#include "signalhandler.h"
#include "vrptools.h"

#ifdef DOSTATS
#include "timer.h"
#endif

#ifdef DOVRPLOG
#include "logger.h"
#endif

#include "trashprob.h"
#include "feasableSol.h"
#include "tabuopt.h"

int vrp_trash_collection( container_t *containers, unsigned int container_count,
                          otherloc_t *otherlocs, unsigned int otherloc_count,
                          vehicle_t *vehicles, unsigned int vehicle_count,
                          ttime_t *ttimes, unsigned int ttime_count,
                          unsigned int iteration,
                          int check,
                          vehicle_path_t **vehicle_paths, int *vehicle_path_count,
                          char **err_msg ) {

    try {
        // register the signal handler
        REG_SIGINT

	#ifdef DOVRPLOG
        if ( not google::IsGoogleLoggingInitialized() ) {
            FLAGS_log_dir = "/tmp/";
            google::InitGoogleLogging( "vrp_trash_collection" );
            FLAGS_logtostderr = 0;
            FLAGS_stderrthreshold = google::FATAL;
            FLAGS_minloglevel = google::INFO;
        }
	#endif


        TrashProb prob(containers,container_count,otherlocs,otherloc_count,ttimes,ttime_count,vehicles,vehicle_count) ;

        if ( check ) {
            std::string err = prob.whatIsWrong();
            if ( err == "" )
                *err_msg = strdup("OK");
            else
                *err_msg = strdup( err.c_str() );
            return 0;
        }

        if ( not prob.isValid() ) {
            std::string err = prob.whatIsWrong();
            *err_msg = strdup( err.c_str() );
            return -1;
        }

        THROW_ON_SIGINT

        FeasableSol tp( prob );
        tp.computeCosts();

        THROW_ON_SIGINT

        TabuOpt ts( tp , iteration);

        Solution best = ts.getBestSolution();
        best.computeCosts();

        int count = 0;
        *vehicle_paths = ts.getSolutionForPg( count );
        *vehicle_path_count = count;

        if ( count == -1 ) {
            *err_msg = strdup( "Failed to allocate memory for results!" );
            return -1;
        }

        twc->cleanUp();
    }
    catch ( std::exception &e ) {
        *err_msg = strdup( e.what() );
        return -1;
    }
    catch ( ... ) {
        *err_msg = strdup( "Caught unknown expection!" );
        return -1;
    }

    *err_msg = (char *)0;

    return EXIT_SUCCESS;
}