#include "irobotNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

/// Program States
typedef enum{
	INITIAL = 0,						///< Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			///< Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			///< Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		///< Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								///< Drive straight
	TURN_RIGHT,						    ///< Turn right
    TURN_LEFT,							///< Turn left
	TURN_RIGHT_REPAIR,                  ///< Turn right until you get back to initial degree (0)
    TURN_LEFT_REPAIR,                   ///< Turn left until you get back to initial degree (0)
	SAMPLING							///< Used for sampling while stationary, in order to get better accel readings
} robotState_t;

void irobotNavigationStatechart(
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const irobotSensorGroup6_t 	sensors,
	const accelerometer_t 		accel,
	const bool					isSimulator,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed
){
	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	static uint8_t i=0; //just to help onRamp variable - needs consistent readings
	static double onRamp=0; //to see if iRobot is on ramp, 1 for true, 0 for false
	static uint8_t flagPreviousObstacle=0; //is 1 if previously encountered a wall or bumped into something
	//I am using it in order to not try to get back on track (get away from bumpy area and not worry by initial direction) immediately after "accident"
	static uint8_t previousState=DRIVE;

	static uint8_t braveDistanceAtManeuverStart=0; //just in case it gets stuck in walls again 

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************


	//*****************************************************
	// state transition - pause region (highest priority) *
	//*****************************************************
	if(   state == INITIAL
	   || state == PAUSE_WAIT_BUTTON_RELEASE
	   || state == UNPAUSE_WAIT_BUTTON_PRESS
	   || state == UNPAUSE_WAIT_BUTTON_RELEASE
	   || sensors.buttons.play				// pause button
	){
		switch(state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if(isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if(!sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if(!sensors.buttons.play){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if(sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************

	else if(state == DRIVE){
		
		if(sensors.wall){
		
			if(onRamp){ 
		       
			   flagPreviousObstacle=1;
			   distanceAtManeuverStart = netDistance;
			   angleAtManeuverStart = netAngle;

			   if(accel.y<=0)                    { state=TURN_RIGHT; } 
			   else				                 { state=TURN_LEFT;  } 
			
			}
			else { 
				  
				  if(netDistance==distanceAtManeuverStart){ //I am smashing into the wall in order to see if it's a ramp or not :) if not, turn l/r
					 if(netAngle>=0)      { state=TURN_RIGHT; }
					 else				  { state=TURN_LEFT;  }
				  }
				   
				  flagPreviousObstacle=1; 

				  distanceAtManeuverStart = netDistance;
				  angleAtManeuverStart = netAngle;
			    }
        

	    }

		else{ 

			  //left side of bot touched something or it sensed a cliff on the left hand side
			  if(     sensors.bumps_wheelDrops.bumpLeft 
				  || (sensors.cliffLeft || sensors.cliffFrontLeft)
				  || sensors.bumps_wheelDrops.wheeldropLeft ) { 
               
			   flagPreviousObstacle=1;

			   distanceAtManeuverStart = netDistance;
     		   angleAtManeuverStart = netAngle;

			   state = TURN_RIGHT; 
			   }
 			   else{ //right side of bot touched something or it sensed a cliff on the right hand side
				 if(    sensors.bumps_wheelDrops.bumpRight 
					 || (sensors.cliffFrontRight || sensors.cliffRight) 					 
					 || sensors.bumps_wheelDrops.wheeldropRight ) { 
					
					flagPreviousObstacle=1;

					distanceAtManeuverStart = netDistance;
	 	            angleAtManeuverStart = netAngle;
				    state = TURN_LEFT; 
			       }
				 else{ //here, no wall in sight and nothing is touching the robot and if it travelled 15 cm, it needs to get back on track

					 if(abs(netDistance - distanceAtManeuverStart) >= 150) { 
		
						if(previousState!=SAMPLING && !onRamp){  
						//after the 15 cm and if I'm not coming straight from SAMPLING I will go into sampling state (0 speed)
		                   state=SAMPLING;
		                   previousState=DRIVE; 
		                  
						 }
		                 else{ //previousState is SAMPLING
                             previousState=DRIVE;
						 
					   if(!flagPreviousObstacle){	
							if( (!onRamp && netAngle>=7) || (onRamp && (accel.x<0 || accel.y<-0.06)) ) { 
							
								distanceAtManeuverStart = netDistance;
								angleAtManeuverStart = netAngle;
							    state = TURN_RIGHT_REPAIR; 
						      } 
						          
							else { 
								   if( (!onRamp && netAngle<=-7) || (onRamp && (accel.x<0 || accel.y>0.06)) ) { 
								   
								   distanceAtManeuverStart = netDistance;
        	 					   angleAtManeuverStart = netAngle;
								   state = TURN_LEFT_REPAIR; 
								   }
							    }
					 }
					  else {
						  flagPreviousObstacle=0;
						  distanceAtManeuverStart = netDistance;
		                  angleAtManeuverStart = netAngle;
					  }
					}
				 }
		       }
		      }
		  }
		}


	else if(state == TURN_RIGHT) { 
		
		if(!sensors.wall && abs(netAngle - angleAtManeuverStart) >= 10){ 

		distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle;
		state = DRIVE;
	  }
		
	}

	else if(state == TURN_LEFT){ 
		
		if(!sensors.wall && abs(netAngle - angleAtManeuverStart) >= 10){

	    distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle;
		state = DRIVE;
	  
		}
	}

	else if(state == TURN_RIGHT_REPAIR){ 
		if( !onRamp && abs(netAngle)<7 ){ 

		distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle;
		state = DRIVE;
		}
		else if( onRamp && accel.x>0 && fabs(accel.y)<=0.06 ) { 
		//used this blindly, according to if it worked or not, tried the trig formulas, to no avail, though
		distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle; 
		state = DRIVE;
		}
	}

	else if(state == TURN_LEFT_REPAIR){ 
		if(!onRamp && abs(netAngle) <7){ 
		distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle;
		state = DRIVE;
		}
		else if( onRamp && accel.x>0 && fabs(accel.y)<=0.06 ) {
		
		distanceAtManeuverStart = netDistance;
		angleAtManeuverStart = netAngle; 
		state = DRIVE;
		}
	}

	else if(state == SAMPLING){ 
		if(previousState==DRIVE){ 
		
	    for(i=0;i<5;i++){ 
		   
		     if(fabs(acos(accel.z/sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z)))>=0.18){  onRamp++;  }
		}
		  if(onRamp>1)   { onRamp=1; } 
		  else           { onRamp=0; }
		
         state = DRIVE;
		}
		previousState=SAMPLING;
    	
	}
	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch(state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 175; 
		break;

	case TURN_RIGHT:
		leftWheelSpeed = 75; 
		rightWheelSpeed = -leftWheelSpeed;
		break;

    case TURN_LEFT:
		leftWheelSpeed = -75; //
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN_RIGHT_REPAIR:
		leftWheelSpeed = 75;
		rightWheelSpeed = -leftWheelSpeed;
		break;

    case TURN_LEFT_REPAIR:
		leftWheelSpeed = -75;
		rightWheelSpeed = -leftWheelSpeed;
		break;

    case SAMPLING:
		// stop and stare
		leftWheelSpeed = rightWheelSpeed = 0; 
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	// write outputs
	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
