#pragma once

#define START_FLIGHTPLAN (10.0)    // start flightplan thread after x seconds
#define GATE_THRESHOLD (0.2)	  // this is how far waypont planner can switch to next wp... Was -0.5, should compensate for delay due to forward speed
#define FINISH_THRESHOLD (-1.0)
#define SKIP_GATE_THRESHOLD 7.0
#define WAYPOINT_THRESHOLD 0.0
#define CLOSE_TO_GATE_THRESHOLD 0.2 // this is related to yaw command and snakegate.. mostly yaw towards next waypoint? TODO: improve this comment

// min 1050 max 1950
#define THRUST_RCMIN 1000
#define THRUST_RCMAX 2000
