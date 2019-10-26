#ifndef DEMINING_STATE_H
#define DEMINING_STATE_H

#include <despot/interface/pomdp.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

#include <stdlib.h>
#include <time.h>

#define	ACTION_NORTH 0    // move forward
#define ACTION_SOUTH 1   // move backward
#define ACTION_WEST 2        // move left
#define ACTION_EAST 3       // move right
#define ACTION_REPORT_WEST 4  // report that there is a mine on the left of the robot
#define ACTION_REPORT_EAST 5 // report that there is a mine on the right of the robot
#define ACTION_REPORT_NORTH 6  // report that there is a mine in front of the robot
#define ACTION_REPORT_SOUTH 7
#define ACTION_NOT_MOVE 8

#define	OBSER_NIL 0      	  	// no mines in any surrounding cells
#define OBSER_WEST 1     		  // one mine in the west cell, no mines in other cells
#define OBSER_NORTH 2        	// one mine in the north cell, no mines in other cells
#define OBSER_EAST 3        	// ...
#define OBSER_WEST_EAST 4  		// one mine in west cell, one mine in east cell, no mine in other cells
#define OBSER_WEST_NORTH 5 		// ...
#define OBSER_NORTH_EAST 6  	// ...
#define	OBSER_WEST_NORTH_EAST 7 	// ...
#define	OBSER_SOUTH 8      	  	// ...
#define OBSER_WEST_SOUTH 9     		  // ...
#define OBSER_NORTH_SOUTH 10        	// ...
#define OBSER_EAST_SOUTH 11        	// ...
#define OBSER_WEST_EAST_SOUTH 12  		// ...
#define OBSER_WEST_NORTH_SOUTH 13 		// ...
#define OBSER_NORTH_EAST_SOUTH 14  	// ...
#define	OBSER_WEST_NORTH_EAST_SOUTH 15 	// four mines in the west, north, east and south cells respectively

//observation probability
#define PROB_NIL_CELL_OBS_MINE 0.1//probability of observing a mine in a NIL cell
#define PROB_MINE_CELL_OBS_MINE 0.9 //probability of observing a mine in a MINE cell

//direction(yaw) of robot
#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

//map size
#define MAP_WIDTH 10
#define MAP_HEIGHT 10

#define MAX_MINE_NUM (MAP_WIDTH * MAP_HEIGHT)
#define MAX_DIST_TO_NEAREST_MINE (MAP_WIDTH+MAP_HEIGHT-2)
#define MAX_DIST_ARRAY_SIZE (MAX_DIST_TO_NEAREST_MINE+2) //MAX_DIST_ARRAY_SIZE=MAX_DIST_TO_NEAREST_MINE+1. 
//Adding 1 is to consider the case that there is no mine, and the case that the distance=0

#define EXPECTED_TOTAL_MINE_NUM 5



using namespace despot;
using namespace std;

struct RobotState
{
	int x;
	int y;
//	int yaw;

};

struct MineState
{
	int x;
	int y;
	bool reported;//whether the mine in (x,y) has been reported
};

//the joint state of RobotState and MineState
class DeminingState : public State
{
public:
	RobotState robot_state_;
	std::vector <MineState> mine_states_;
	
public:
	DeminingState();
	DeminingState(RobotState robot_state, std::vector<MineState> mine_states);


	//calculate the probability of observing mines in the west, north , east and south cell respectively
	void ProbOfObsvingMine(double &prob_west_cell, double &prob_north_cell, double &prob_east_cell, double &prob_south_cell) const;
	
	void GetSurroundingPos(int &x, int &y, int direction) const;

	bool IsAllMinesAround(int x, int y);
	
	//return true if the cell has a mine, false otherwise
	bool IsMineCell(int x, int y) const;
	
	//return true if the cell has a mine, false otherwise. 
	//in the mean time, assign the mine index in mine_states_ vector if there is a mine, assign -1 otherwise
	bool IsMineCell(int x, int y, int& index) const;

	//generate random state
	void GenRandomState();

	//generate DeminingState with random MineStates but deterministic RobotState
	void GenStateWithRandomMine(RobotState robot_state);

	bool AllMinesClear();

	//get the Manhattan distance to the nearest mine
	int GetDistToNearestMine() const;
	int GetDistToNearestReportedMine() const;
	int GetDistToNearestUnreportedMine() const;

	void GetDistToNearestMine(int &x, int &y) const;
	void GetDistToNearestReportedMine(int &x, int &y) const;
	void GetDistToNearestUnreportedMine(int &x, int &y) const;

	//set mines position
	void SetMines(std::vector <MineState> mine_states);

	int NumOfUnreportedMines() const;

};


#endif