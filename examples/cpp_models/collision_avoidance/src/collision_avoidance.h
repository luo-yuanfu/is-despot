#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "collision_avoidance_state.h"
#include <math.h>
#include <random>
#include <iostream>

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

#define PENALTY_COLLISION -1000
#define PENALTY_OUT_OF_BOUNDARY -1
#define PENALTY_MOVE_UP -1
#define PENALTY_MOVE_DOWN -1

#define ERROR_MEAN 0.0 //the mean of the observation error
#define ERROR_STDDEV 1.0 //the standard deviation of the observation error
#define ROUND(X) (int)(floor(X+0.5))

#define PROB_UP 0.25 //the probability that the obstacle moves up
#define PROB_DOWN 0.25 //the probability that the obstacle moves down
#define PROB_NOT_MOVE 0.5 //the probability that the obstacle remains where it is

#define PROB_MOVE_SUCCESS 1.0 //the probability that the aircraft move successfully to the place it intends to go


/* ==============================================================================
 * CollisionAvoidance class
 * ==============================================================================*/
using namespace std;

namespace despot {

class CollisionAvoidance : public DSPOMDP {
 public:
  enum {                // action list
    A_FORWARD = 0, //fly forward
    A_UPWARD = 1, //fly up
    A_DOWNWARD = 2 //fly down
  };

  OBS_TYPE observation[MAP_HEIGHT];
  

 private:
  int num_particles; //number of particles that used to represent (initial) belief
  
  double  error_prob[ERROR_RANGE];
  mutable std::default_random_engine generator_;
  mutable std::normal_distribution<double> distribution_;

  double sampling_weight_[MAP_WIDTH][MAP_HEIGHT];

 protected:
	mutable MemoryPool<WorldState> memory_pool_;
  mutable WorldState* start_state_;

 public:
  CollisionAvoidance();

  ~CollisionAvoidance();

  /* ========================================================================
   * Deterministic simulative model and related functions
   * ========================================================================*/
  /**
   * Determistic simulative model for POMDP.
   */
  bool Step(State &state, double random_num, ACT_TYPE action, double &reward,
            OBS_TYPE &obs) const;

  bool ImportanceSamplingStep(State& s, double random_num, ACT_TYPE action, double& reward,
    OBS_TYPE& obs) const;

  vector<double> ImportanceWeight(vector<State*> particles) const;

  /* ========================================================================
   * Action
   * ========================================================================*/
  /**
   * Returns number of actions.
   */
  inline int NumActions() const {return 3;}

  /* ========================================================================
   * Functions related to beliefs and starting states.
   * ========================================================================*/
  /**
   * Returns the observation probability.
   */
  double ObsProb(OBS_TYPE obs, const State &state, ACT_TYPE action) const;

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
  inline double GetMaxReward() const {return 0;}

  /**
   * Returns (a, v), where a is an action with largest minimum reward when it is
   * executed, and v is its minimum reward, that is, a = \max_{a'} \min_{s}
   * R(a', s), and v = \min_{s} R(a, s).
   */
  inline ValuedAction GetBestAction() const 
  {
    return ValuedAction(A_FORWARD, -1000); 
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
  void PrintAction(ACT_TYPE action, ostream &out = cout) const;

  /**
   * Prints a belief.
   */
  void PrintBelief(const Belief &belief, ostream &out = cout) const;

  void PrintParticles(const vector<State*> particles, ostream& out = cout) const;

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

  OBS_TYPE GetObs(WorldState world_state, Random &random) const;

  vector<double> Feature(const State& state) const;
};
}

#endif
