#include "collision_avoidance.h"
#include <iomanip>

namespace despot {

CollisionAvoidance::CollisionAvoidance()
{
	srand(time(NULL));
	num_particles=10000;
	start_state_=NULL;

	std::normal_distribution<double> temp_distr(ERROR_MEAN,ERROR_STDDEV);
	distribution_ = temp_distr;

	//following code uses a table error_prob to approximate the probability
	for(int i=0; i<ERROR_RANGE; i++){
		error_prob[i]=1; //this is to avoid the situation that some observation's probability = 0
	}

	int total_count=ERROR_RANGE;
	for(int i=0; i<2000; i++){
		double number = distribution_(generator_);
    	if ( ROUND(number) >= -(MAP_HEIGHT-1) && ROUND(number) <= MAP_HEIGHT-1 ) {
    		error_prob[ROUND(number)+MAP_HEIGHT-1]=error_prob[ROUND(number)+MAP_HEIGHT-1]+1;
    		total_count ++;
    	}
	}

	//normalize the probability
	for(int i=0; i<ERROR_RANGE; i++)
		error_prob[i] /= total_count;
	

	for(int i=0; i<MAP_HEIGHT; i++)
		observation[i]=i;

	if(Globals::config.use_is_despot == false){
		cout<<"use_is_despot is set to false; will not use importance sampling"<<endl;
		for(int i=0; i<MAP_WIDTH; i++)
			for(int j=0; j<MAP_HEIGHT; j++)
				sampling_weight_[i][j]=1;
		return;
	}

	for(int i=0; i<MAP_WIDTH; i++)
		for(int j=0; j<MAP_HEIGHT; j++)
			sampling_weight_[i][j]=0;

	ifstream file;
	file.open("sampling_weight.dat", std::ifstream::in);
	if(file.fail()){
		cout<<"fail to open sampling_weight.dat file"<<endl;
		cout<<"will not use importance sampling"<<endl;
		for(int i=0; i<MAP_WIDTH; i++)
			for(int j=0; j<MAP_HEIGHT; j++)
				sampling_weight_[i][j]=1;
		return;
	}

	int horizontal_dist, vertical_dist;
	double mean, std, ste, importance, global_importance;
	while(file>>horizontal_dist>>vertical_dist>>mean>>std>>ste>>importance>>global_importance){
		sampling_weight_[horizontal_dist][vertical_dist]=importance;
		//sampling_weight_[horizontal_dist][vertical_dist]=fabs(mean);
	}

	for(int i=0; i<MAP_WIDTH; i++)
		for(int j=0; j<MAP_HEIGHT; j++)	{
					if(sampling_weight_[i][j]==0) sampling_weight_[i][j]=0.005;
			assert(sampling_weight_[i][j]!=0);
			
		}

	file.close();

	/*file.open("sampling_weight-is.dat", std::ifstream::in);
        if(file.fail()){
                cout<<"fail to open sampling_weight-is.dat file"<<endl;
                cout<<"don't use mixture importance sampling"<<endl;
        }

        while(file>>horizontal_dist>>vertical_dist>>mean>>std>>ste>>importance>>global_importance){
                sampling_weight_[horizontal_dist][vertical_dist]=0.5*sampling_weight_[horizontal_dist][vertical_dist]+0.5*importance;
                //sampling_weight_[horizontal_dist][vertical_dist]=fabs(mean);
        }
	file.close();*/
}

CollisionAvoidance::~CollisionAvoidance()
{
	if(start_state_!=NULL)
	{
		delete start_state_;
		start_state_=NULL;
	}
	
} 

/* ========================================================================
 * Deterministic simulative model and related functions
 * ========================================================================*/
/**
 * Determistic simulative model for POMDP.
 */
 bool CollisionAvoidance::Step(State &state, double random_num, ACT_TYPE action, double &reward,
                  OBS_TYPE &obs) const
 {
 	Random random(random_num);
 	WorldState& world_state = static_cast<WorldState&>(state);

 	reward=0;

 	switch(action)
 	{
	 	case A_FORWARD:
	 	{
	 		if(random.NextDouble()<=PROB_MOVE_SUCCESS) ;
	 		else{
	 			if(random.NextInt(2)) {
	 				if(world_state.aircraft_height+1<MAP_HEIGHT) world_state.aircraft_height++;
	 			}
	 			else {
	 				if(world_state.aircraft_height-1>=0) world_state.aircraft_height--;
	 			}
	 		}
	 		world_state.distance--; 			
			break;
	 	}
	    case A_UPWARD:
	    {
	    	if(random.NextDouble()<=PROB_MOVE_SUCCESS){
	    		if(world_state.aircraft_height+1<MAP_HEIGHT) world_state.aircraft_height++;
	    		else {
	    			reward+=PENALTY_OUT_OF_BOUNDARY;
	    		}
	    	}
	 		else{
	 			if(random.NextInt(2)==0) {
	 				if(world_state.aircraft_height+2<MAP_HEIGHT) world_state.aircraft_height += 2;
	 			}
	 			else {
	 				;
	 			}
	 		}

	 		reward += PENALTY_MOVE_UP;
	 		world_state.distance--; 			
			break;

	 	}
	    case A_DOWNWARD:
	    {
	 		if(random.NextDouble()<=PROB_MOVE_SUCCESS){
	    		if(world_state.aircraft_height-1>=0) world_state.aircraft_height--;
	    		else {
	    			reward+=PENALTY_OUT_OF_BOUNDARY;
	    		}
	    	}
	 		else{
	 			if(random.NextInt(2)==0) {
	 				if(world_state.aircraft_height-2>=0) world_state.aircraft_height -= 2;
	 			}
	 			else {
	 				;
	 			}
	 		}

	 		reward += PENALTY_MOVE_DOWN;
	 		world_state.distance--; 			
			break;
	 	}
	    
		default:
		{
			break;
		}
	}

	if( random.NextDouble() < PROB_UP){
		if(world_state.obstacle_height+1<MAP_HEIGHT) world_state.obstacle_height++;
	}
	else if(random.NextDouble() < PROB_UP + PROB_DOWN) {
		if(world_state.obstacle_height-1>=0) world_state.obstacle_height--;
	}
	else {
		;
	}

	if(world_state.distance==0)
	{
		if(world_state.aircraft_height == world_state.obstacle_height)
			reward += PENALTY_COLLISION;
		else reward += 0;
		obs = GetObs(world_state, random);
		return true;
	}
	
 	obs = GetObs(world_state, random);
 	return false;
 }


 bool CollisionAvoidance::ImportanceSamplingStep(State& s, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
 	Random random(random_num);
 	WorldState& world_state = static_cast<WorldState&>(s);

 	reward=0;
 	double prob_up, prob_down, prob_not_move;

 	switch(action)
 	{
	 	case A_FORWARD:
	 	{
	 		if(random.NextDouble()<=PROB_MOVE_SUCCESS) ;
	 		else{
	 			if(random.NextInt(2)==0) {
	 				if(world_state.aircraft_height+1<MAP_HEIGHT) world_state.aircraft_height++;
	 			}
	 			else {
	 				if(world_state.aircraft_height-1>=0) world_state.aircraft_height--;
	 			}
	 		}
	 		world_state.distance--; 			
			break;
	 	}
	    case A_UPWARD:
	    {
	    	if(random.NextDouble()<=PROB_MOVE_SUCCESS){
	    		if(world_state.aircraft_height+1<MAP_HEIGHT) world_state.aircraft_height++;
	    		else {
	    			reward+=PENALTY_OUT_OF_BOUNDARY;
	    		}
	    	}
	 		else{
	 			if(random.NextInt(2)==0) {
	 				if(world_state.aircraft_height+2<MAP_HEIGHT) world_state.aircraft_height += 2;
	 			}
	 			else {
	 				;
	 			}
	 		}

	 		reward += PENALTY_MOVE_UP;
	 		world_state.distance--; 			
			break;

	 	}
	    case A_DOWNWARD:
	    {
	 		if(random.NextDouble()<=PROB_MOVE_SUCCESS){
	    		if(world_state.aircraft_height-1>=0) world_state.aircraft_height--;
	    		else {
	    			reward+=PENALTY_OUT_OF_BOUNDARY;
	    		}
	    	}
	 		else{
	 			if(random.NextInt(2)==0) {
	 				if(world_state.aircraft_height-2>=0) world_state.aircraft_height -= 2;
	 			}
	 			else {
	 				;
	 			}
	 		}

	 		reward += PENALTY_MOVE_DOWN;
	 		world_state.distance--; 			
			break;
	 	}
	    
		default:
		{
			break;
		}
	}

	int obst_height;
	if(world_state.obstacle_height+1<MAP_HEIGHT) obst_height =  world_state.obstacle_height+1;
	else obst_height = world_state.obstacle_height;
	prob_up=sampling_weight_[world_state.distance][int(abs(obst_height  - world_state.aircraft_height ))]*PROB_UP;

	if(world_state.obstacle_height-1>=0) obst_height = world_state.obstacle_height-1;
	else obst_height = world_state.obstacle_height;
	prob_down=sampling_weight_[world_state.distance][int(abs(obst_height  - world_state.aircraft_height ))]*PROB_DOWN;

	obst_height = world_state.obstacle_height;
	prob_not_move=sampling_weight_[world_state.distance][int(abs(obst_height  - world_state.aircraft_height ))]*PROB_NOT_MOVE;

	double total_prob;
	total_prob=prob_up+prob_down+prob_not_move;
	prob_up /= total_prob;
	prob_down /= total_prob;
	prob_not_move /= total_prob;

//hand-crafted importance distribution
/*	if(world_state.obstacle_height < world_state.aircraft_height){
 		prob_up=0.762;
 		prob_down=0.048;
 		prob_not_move=0.190;
  	}
  	else if(world_state.obstacle_height == world_state.aircraft_height){
 		prob_up=0.1665;
 		prob_down=0.1665;
 		prob_not_move=0.667;
  	}
  	else{
 		prob_up=0.048;
 		prob_down=0.762;
 		prob_not_move=0.190;
  	}*/

	if( random.NextDouble() < prob_up){
		if(world_state.obstacle_height+1<MAP_HEIGHT) world_state.obstacle_height++;
		world_state.weight *= (PROB_UP/prob_up);
		if(world_state.distance==0)
		{
			if(world_state.aircraft_height == world_state.obstacle_height)
				reward += PENALTY_COLLISION * (PROB_UP/prob_up);
			else reward += 0;
			obs = GetObs(world_state, random);
			return true;
		}
	}
	else if(random.NextDouble() < prob_up + prob_down) {
		if(world_state.obstacle_height-1>=0) world_state.obstacle_height--;
		world_state.weight *= (PROB_DOWN/prob_down);
		if(world_state.distance==0)
		{
			if(world_state.aircraft_height == world_state.obstacle_height)
				reward += PENALTY_COLLISION * (PROB_DOWN/prob_down);
			else reward += 0;
			obs = GetObs(world_state, random);
			return true;
		}
	}
	else {
		world_state.weight *= (PROB_NOT_MOVE/prob_not_move);
		if(world_state.distance==0)
		{
			if(world_state.aircraft_height == world_state.obstacle_height)
				reward += PENALTY_COLLISION * (PROB_NOT_MOVE/prob_not_move);
			else reward += 0;
			obs = GetObs(world_state, random);
			return true;
		}
	}

	
	
 	obs = GetObs(world_state, random);
 	return false;
}

 OBS_TYPE CollisionAvoidance::GetObs(WorldState world_state, Random &random) const
 {
 	double number;
 	while(true) {
 		//number = distribution_(generator_);
 		number = random.NextGaussian() * ERROR_STDDEV + ERROR_MEAN;
    	if ( ROUND(number) + world_state.obstacle_height >= 0 && ROUND(number) + world_state.obstacle_height < MAP_HEIGHT )
    		return observation[ROUND(number) + world_state.obstacle_height];
    }
 } 


/* ========================================================================
 * Functions related to beliefs and starting states.
 * ========================================================================*/
/**
 * Returns the observation probability: the probability of observing 'obs' after executing 'action' and reaching 'state'
 */
 double CollisionAvoidance::ObsProb(OBS_TYPE obs, const State &state, ACT_TYPE action)  const
 {
	const WorldState& world_state = static_cast<const WorldState&>(state);
	double prob[MAP_HEIGHT];
	double total_weight=0;

	for(int i=0; i<MAP_HEIGHT; i++) {
		prob[i]=error_prob[i+MAP_HEIGHT-1-world_state.obstacle_height];
		total_weight+=prob[i];
	}

	if(total_weight==0) return 0;
	
	//normalize the prob
	for(int i=0; i<MAP_HEIGHT; i++) {
		prob[i] /= total_weight;
	}

	return prob[obs];
 }

/**
 * Returns a starting state.
 */
 State * CollisionAvoidance::CreateStartState(string type)  const
 {
 	if(start_state_!=NULL)
 	{
 		delete start_state_;
 //		start_state_=NULL;
 	}
 	start_state_=new WorldState();
 	start_state_->GenRandomState();
 	return start_state_;
 }

/**
 * Returns the initial belief.
 */
 Belief * CollisionAvoidance::InitialBelief(const State *start, string type)  const
 {
 	vector<State*> particles;
 	double weight=1.0/num_particles;

 	for(int n=0; n<num_particles; n++)
 	{
 		WorldState* world_state = static_cast<WorldState*>(Allocate(n, weight));
 		world_state->GenRandomState();
 		world_state->aircraft_height=start_state_->aircraft_height;
 		particles.push_back(world_state);
 	}

 	return new ParticleBelief(particles, this);
 }



/* ========================================================================
 * Display
 * ========================================================================*/
/**
 * Prints a state.
 */
 void CollisionAvoidance::PrintState(const State &state, ostream &out)  const
 {
 /*	const WorldState& world_state = static_cast<const WorldState &>(state);

 	char map[MAP_HEIGHT][MAP_WIDTH];
 	for(int i=0; i<MAP_HEIGHT; i++)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			map[i][j]='.';
 	}

 	map[world_state.aircraft_height][world_state.distance]='A';

 	map[world_state.obstacle_height][0]='O';

 	for(int i=MAP_HEIGHT-1; i>=0; i--)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			out<<map[i][j];
 		out<<endl;
 	}
*/
 }

/**
 * Prints an observation.
 */
 void CollisionAvoidance::PrintObs(const State &state, OBS_TYPE obs, ostream &out)  const
 {
/* 	out<<endl;
 	const WorldState& world_state = static_cast<const WorldState &>(state);

 	char map[MAP_HEIGHT][MAP_WIDTH];
 	for(int i=0; i<MAP_HEIGHT; i++)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			map[i][j]='.';
 	}

 	map[world_state.aircraft_height][world_state.distance]='A';

 	map[obs][0]='O';

 	for(int i=MAP_HEIGHT-1; i>=0; i--)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			out<<map[i][j];
 		out<<endl;
 	}*/
 }

/**
 * Prints an action.
 */
 void CollisionAvoidance::PrintAction(ACT_TYPE action, ostream &out)  const
 {
 	switch(action)
 	{
	 	case A_FORWARD:      // move forward
	 	{
	 		out<<"FORWARD"<<endl;
			break;
	 	}
	    case A_UPWARD:     // move backward
	    {
	 		out<<"UP"<<endl;
			break;
	 	}
	    case A_DOWNWARD:         // move left
	    {
	 		out<<"DOWN"<<endl;
			break;
	 	}
 	}
 }

/**
 * Prints a belief.
 */
 void CollisionAvoidance::PrintBelief(const Belief &belief, ostream &out) const
 {
/* 	const std::vector<State*>& particles 
 	 	= (static_cast<const ParticleBelief&>(belief).particles());

 	int particle_num = particles.size();
 	out<<particle_num<<endl;
 	double obst_belief[MAP_HEIGHT];
 	double air_belief[MAP_HEIGHT];
 	double total_weight=0;

 	for(int i=0;i<MAP_HEIGHT;i++){
 		obst_belief[i]=0;
 		air_belief[i]=0;
 	}

 	for(int i=0; i<particle_num; i++)
 	{
 		const WorldState* world_state=static_cast<const WorldState*>(particles.at(i));
 		
 		obst_belief[world_state->obstacle_height] += world_state->weight;
 		air_belief[world_state->aircraft_height] += world_state->weight;
 		total_weight += world_state->weight;
 	}

 	for(int i=MAP_HEIGHT-1;i>=0; i--)
 	{

 		//if(joint_belief[i][j]!=0) 
 		out<<setiosflags(ios::fixed);
 		//	else out<<setiosflags(ios::right);
 		
 
 		if(obst_belief[i]>=0.000000999999) 
 			out<<setw(10)<<obst_belief[i]/total_weight;
 		else out<<"         "<<setw(1)<<0;

 		out<<"........";

 		if(air_belief[i]>=0.000000999999) 
 			out<<setw(10)<<air_belief[i]/total_weight;
 		else out<<"         "<<setw(1)<<0;
 		
 		out<<endl;
 	}*/
 } 


 void CollisionAvoidance::PrintParticles(const vector<State*> particles, ostream& out) const
 {
/*
 	int particle_num = particles.size();
 	out<<particle_num<<endl;
 	double obst_belief[MAP_HEIGHT];
 	double air_belief[MAP_HEIGHT];
 	double total_weight=0;

 	for(int i=0;i<MAP_HEIGHT;i++){
 		obst_belief[i]=0;
 		air_belief[i]=0;
 	}

 	for(int i=0; i<particle_num; i++)
 	{
 		const WorldState* world_state=static_cast<const WorldState*>(particles.at(i));
 		
 		obst_belief[world_state->obstacle_height] += world_state->weight;
 		air_belief[world_state->aircraft_height] += world_state->weight;
 		total_weight += world_state->weight;
 	}

 	for(int i=MAP_HEIGHT-1;i>=0; i--)
 	{

 		//if(joint_belief[i][j]!=0) 
 		out<<setiosflags(ios::fixed);
 		//	else out<<setiosflags(ios::right);
 		
 
 		if(obst_belief[i]>=0.000000999999) 
 			out<<setw(10)<<obst_belief[i]/total_weight;
 		else out<<"         "<<setw(1)<<0;

 		out<<"........";

 		if(air_belief[i]>=0.000000999999) 
 			out<<setw(10)<<air_belief[i]/total_weight;
 		else out<<"         "<<setw(1)<<0;
 		
 		out<<endl;
 	}*/
 } 
/* ========================================================================
 * Memory management.
 * ========================================================================*/
/**
 * Allocate a state.
 */
 State * CollisionAvoidance::Allocate(int state_id, double weight)  const
 {
 	WorldState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
 }

/**
 * Returns a copy of the state.
 */
 State * CollisionAvoidance::Copy(const State *state)  const
 {
 	WorldState* copy_state=memory_pool_.Allocate();
 	*copy_state = * static_cast<const WorldState*>(state);
 	copy_state->SetAllocated();
 	return copy_state;
 }

/**
 * Returns a copy of the particle.
 */
 void CollisionAvoidance::Free(State *state)  const
 {
 	memory_pool_.Free(static_cast<WorldState*>(state));
 }

/**
 * Returns number of allocated particles.
 */
 int CollisionAvoidance::NumActiveParticles()  const
 {
 	return memory_pool_.num_allocated();
 }


 vector<double> CollisionAvoidance::ImportanceWeight(vector<State*> particles) const
 {
 	double total_weight=State::Weight(particles);
 	double new_total_weight=0;
	int particles_num=particles.size();

	vector<WorldState*> collision_avoidance_particles;

	vector <double> importance_weight;

	for(int i=0; i<particles_num;i++){
		collision_avoidance_particles.push_back(static_cast<WorldState*>(particles[i]));
	}

	for(int i=0; i<particles_num;i++){
		importance_weight.push_back(collision_avoidance_particles[i]->weight * sampling_weight_[collision_avoidance_particles[i]->distance][int(abs( collision_avoidance_particles[i]->aircraft_height - collision_avoidance_particles[i]->obstacle_height))]);
		new_total_weight += importance_weight[i];
	}

	//normalize to total_weight
	for(int i=0; i<particles_num;i++){
		importance_weight[i]=importance_weight[i]*total_weight/new_total_weight;
		assert(importance_weight[i]>0);
	}

	return importance_weight;

//hand-crafted importance distribution for initial belief
/*
 	double total_weight=State::Weight(particles);
 	double new_total_weight=0;
	int particles_num=particles.size();

	vector<WorldState*> collision_avoidance_particles;

	vector <double> importance_weight;
	vector <double> self_importance_weight;

	for(int i=0; i<particles_num;i++){
		collision_avoidance_particles.push_back(static_cast<WorldState*>(particles[i]));
	}

	double power[MAP_HEIGHT];
	for (int i=0; i<MAP_HEIGHT; i++){
		power[i]=pow(PROB_UP, i);
	}

	double weight_to_push;
	for(int i=0; i<particles_num;i++){
		//weight_to_push=power[int(abs(collision_avoidance_particles[i]->obstacle_height - collision_avoidance_particles[i]->aircraft_height))]*collision_avoidance_particles[i]->weight;
		weight_to_push=power[int(abs(collision_avoidance_particles[i]->obstacle_height - collision_avoidance_particles[i]->aircraft_height))];

		importance_weight.push_back(weight_to_push);
		new_total_weight += weight_to_push;
	}

	//normalize to 1
	for(int i=0; i<particles_num;i++){
		importance_weight[i]=importance_weight[i]/new_total_weight;
	}

	//normalize to 1
	for(int i=0; i<particles_num;i++){
		self_importance_weight.push_back(collision_avoidance_particles[i]->weight/total_weight);
	}



	//normalize to total_weight
	for(int i=0; i<particles_num;i++){
		importance_weight[i]=(0.6*importance_weight[i]+0.4*self_importance_weight[i])*total_weight;
	}

	return importance_weight;
*/

 }


 vector<double> CollisionAvoidance::Feature(const State& state) const {
	vector<double> feature;

	if(Globals::config.collect_data == false) return feature;
	
	const WorldState& world_state = static_cast<const WorldState&>(state);
	feature.push_back(world_state.distance);
	feature.push_back(abs(world_state.aircraft_height - world_state.obstacle_height));

	return feature;
}

}
