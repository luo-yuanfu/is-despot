#ifndef COLLISION_AVOIDANCE_STATE_H
#define COLLISION_AVOIDANCE_STATE_H

#include <despot/interface/pomdp.h>
#include <stdlib.h>
#include <time.h>


#define MAP_WIDTH 30
#define MAP_HEIGHT 20
#define ERROR_RANGE (2*MAP_HEIGHT-1)

using namespace std;
using namespace despot;

class WorldState : public State{
public:
	WorldState();
	~WorldState();

	int aircraft_height; //the height of the aircraft

	int obstacle_height; //the height of the obstacle
	
	int distance; // the distance between the aircraft and the obstacle

	void GenRandomState();
};


#endif
