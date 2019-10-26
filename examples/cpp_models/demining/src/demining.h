#ifndef DEMINING_H
#define DEMINING_H

#include "closed_form_belief.h"

//rewards and penalty
#define PENALTY_STEP_OVER_MINE -1000
#define PENALTY_WRONG_REPORT -10
#define REWARD_CORRECT_REPORT 10
#define PENALTY_COLLIDE_WITH_WALL -1
#define COST_OBSERVE 0
#define PENALTY_REPORT_REPEATED_MINE -1


/* ==============================================================================
 * Demining class
 * ==============================================================================*/

class Demining : public DSPOMDP {
 public:
  enum {                // action list
    A_NORTH = 0,      // move forward
    A_SOUTH = 1,     // move backward
    A_WEST = 2,         // move left
    A_EAST = 3,        // move right
    A_REPORT_WEST = 4,  // report that there is a mine on the left of the robot
    A_REPORT_EAST = 5, // report that there is a mine on the right of the robot
    A_REPORT_NORTH = 6,  // report that there is a mine in front of the robot
	  A_REPORT_SOUTH = 7,
	  A_NOT_MOVE = 8
  };
  
  enum {                 	// observation list
    O_NIL = 0,      	  	// no mines in any surrounding cells
    O_WEST = 1,     		  // one mine in the west cell, no mines in other cells
    O_NORTH = 2,        	// one mine in the north cell, no mines in other cells
    O_EAST = 3,        	// ...
    O_WEST_EAST = 4,  		// one mine in west cell, one mine in east cell, no mine in other cells
    O_WEST_NORTH = 5, 		// ...
    O_NORTH_EAST = 6,  	// ...
	  O_WEST_NORTH_EAST = 7, 	// ...
	  O_SOUTH = 8,      	  	// ...
    O_WEST_SOUTH = 9,     		  // ...
    O_NORTH_SOUTH = 10,        	// ...
    O_EAST_SOUTH = 11,        	// ...
    O_WEST_EAST_SOUTH = 12,  		// ...
    O_WEST_NORTH_SOUTH = 13, 		// ...
    O_NORTH_EAST_SOUTH = 14,  	// ...
	  O_WEST_NORTH_EAST_SOUTH = 15 	// four mines in the west, north, east and south cells respectively
  };

 private:
  int num_particles; //number of particles that used to represent (initial) belief

 protected:
  mutable MemoryPool<DeminingState> memory_pool_;
  mutable DeminingState* start_state_;

 public:
 	mutable double belief_table[MAP_HEIGHT][MAP_WIDTH];
  double sampling_weight_[MAX_MINE_NUM][MAX_DIST_ARRAY_SIZE];

 public:
  Demining();

  ~Demining();

  /* ========================================================================
   * Deterministic simulative model and related functions
   * ========================================================================*/
  /**
   * Determistic simulative model for POMDP.
   */
  bool Step(State &state, double random_num, int action, double &reward,
            OBS_TYPE &obs) const;

  /* ========================================================================
   * Action
   * ========================================================================*/
  /**
   * Returns number of actions.
   */
  inline int NumActions() const {return 9;}

  /* ========================================================================
   * Functions related to beliefs and starting states.
   * ========================================================================*/
  /**
   * Returns the observation probability.
   */
  double ObsProb(OBS_TYPE obs, const State &state, int action) const;

  /**
   * Returns a starting state.
   */
  State *CreateStartState(string type = "DEFAULT")const;

  /**
   * Returns the initial belief.
   */
  Belief *InitialBelief(const State *start, string type = "DEFAULT") const;

  /* ========================================================================
   * Bound-related functions.
   * ========================================================================*/
  /**
   * Returns the maximum reward.
   */
  inline double GetMaxReward() const {return REWARD_CORRECT_REPORT;}

  /**
   * Returns (a, v), where a is an action with largest minimum reward when it is
   * executed, and v is its minimum reward, that is, a = \max_{a'} \min_{s}
   * R(a', s), and v = \min_{s} R(a, s).
   */
  inline ValuedAction GetBestAction() const 
  {
    return ValuedAction(A_NOT_MOVE, COST_OBSERVE); 
  }

  /* ========================================================================
   * Display
   * ========================================================================*/
  /**
   * Prints a state.
   */
  void PrintState(const State &state, ostream &out = cout) const;

  /**
   * Prints an observation.
   */
  void PrintObs(const State &state, OBS_TYPE obs, ostream &out = cout) const;

  /**
   * Prints an action.
   */
  void PrintAction(int action, ostream &out = cout) const;

  /**
   * Prints a belief.
   */
  void PrintBelief(const Belief &belief, ostream &out = cout) const;

  void PrintParticles(const vector<State*> particles, ostream& out) const;

  /* ========================================================================
   * Memory management.
   * ========================================================================*/
  /**
   * Allocate a state.
   */
  State *Allocate(int state_id = -1, double weight = 0) const;

  /**
   * Returns a copy of the state.
   */
  State *Copy(const State *state) const;

  /**
   * Returns a copy of the particle.
   */
  void Free(State *state) const;

  /**
   * Returns number of allocated particles.
   */
  int NumActiveParticles() const;

  OBS_TYPE GetObs(DeminingState demining_state, Random &random) const;

  vector<double> ImportanceWeight(vector<State*> particles) const;

  vector<double> Feature(const State& state) const ;

};

#endif
