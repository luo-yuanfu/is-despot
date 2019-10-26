#include "demining.h"
#include <iomanip>

Demining::Demining()
{
	srand(time(NULL));
	start_state_=NULL;

	if(Globals::config.use_is_despot == false){
		cout<<"no-importance-sampling is set; don't use importance sampling"<<endl;
		for(int i=0; i<MAX_MINE_NUM; i++)
			for(int j=0; j<MAX_DIST_ARRAY_SIZE; j++)
				sampling_weight_[i][j]=1;
		return;
	}

	ifstream file;
	file.open("sampling_weight.dat", std::ifstream::in);
	if(file.fail()){
		cout<<"fail to open sampling_weight.dat file"<<endl;
		cout<<"don't use importance sampling"<<endl;
		for(int i=0; i<MAX_MINE_NUM; i++)
			for(int j=0; j<MAX_DIST_ARRAY_SIZE; j++)
				sampling_weight_[i][j]=1;
		return;
	}

	for(int i=0; i<MAX_MINE_NUM; i++)
		for(int j=0; j<MAX_DIST_ARRAY_SIZE; j++)
			sampling_weight_[i][j]=0;

	int num_unreported_mine, dist_nearest_mine;
	double mean, std, ste, importance, global_importance;
	while(file>>num_unreported_mine>>dist_nearest_mine>>mean>>std>>ste>>importance>>global_importance){
		if(dist_nearest_mine > MAX_DIST_TO_NEAREST_MINE) dist_nearest_mine=MAX_DIST_ARRAY_SIZE-1; //sometimes, there is no mine. then the distance to the nearest unreported mine is > MAX_DIST_TO_NEAREST_MINE
		sampling_weight_[num_unreported_mine][dist_nearest_mine]=importance;
		//sampling_weight_[num_unreported_mine][dist_nearest_mine]=fabs(mean);
	}

	for(int i=0; i<MAX_MINE_NUM; i++)
		for(int j=0; j<MAX_DIST_ARRAY_SIZE; j++)
			if(j==0) sampling_weight_[i][j]=0; //never sample the case the robot is on the mine cell
			else if(sampling_weight_[i][j]==0){
				sampling_weight_[i][j] = 0.005; //in case some extreme situation happens
			}
	file.close();

	return;
}

Demining::~Demining()
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
 bool Demining::Step(State &state, double random_num, int action, double &reward,
                  OBS_TYPE &obs) const
 {
 	Random random(random_num);
 	DeminingState& demining_state = static_cast<DeminingState&>(state);

 	reward=0;

 	switch(action)
 	{
	 	case A_NORTH:      // move up
	 	{
	 		
	 		if(demining_state.robot_state_.y+1<MAP_HEIGHT)
	 				demining_state.robot_state_.y=demining_state.robot_state_.y+1;
	 		else reward += PENALTY_COLLIDE_WITH_WALL;
			
			reward += COST_OBSERVE;
	 			
			break;
	 	}
	    case A_SOUTH:     // move down
	    {
	 		if(demining_state.robot_state_.y-1>=0)
	 				demining_state.robot_state_.y=demining_state.robot_state_.y-1;
	 		else reward += PENALTY_COLLIDE_WITH_WALL;
			
			reward += COST_OBSERVE;
	 			
			break;
	 	}
	    case A_WEST:         // move left
	    {
	 		if(demining_state.robot_state_.x-1>=0)
	 				demining_state.robot_state_.x=demining_state.robot_state_.x-1;
	 		else reward += PENALTY_COLLIDE_WITH_WALL;
			
			reward += COST_OBSERVE;
	 			
			break;
	 	}
	    case A_EAST:        // move right
	    {
			if(demining_state.robot_state_.x+1<MAP_WIDTH)
	 				demining_state.robot_state_.x=demining_state.robot_state_.x+1;
	 		else reward += PENALTY_COLLIDE_WITH_WALL;
			
			reward += COST_OBSERVE;
	 			
			break;
	 	}
		case A_REPORT_NORTH:
		case A_REPORT_SOUTH:
		case A_REPORT_WEST: 
		case A_REPORT_EAST:
	    {
			int x, y, index;

			switch(action)
			{
				case A_REPORT_WEST:
				{
					demining_state.GetSurroundingPos(x,y,WEST);
					break;
				}
				case A_REPORT_EAST:
				{
					demining_state.GetSurroundingPos(x,y,EAST);
					break;
				}
				case A_REPORT_NORTH:
				{
					demining_state.GetSurroundingPos(x,y,NORTH);
					break;
				}
				case A_REPORT_SOUTH:
				{
					demining_state.GetSurroundingPos(x,y,SOUTH);
					break;
				}
			}
			
			//assign the report position to x and y based on action; assign -1 and -1 if the position is out of boundary
			//demining_state.GetReportPos(x,y,action);
			
			if(demining_state.IsMineCell(x,y,index)==true)
			{
				if(demining_state.mine_states_.at(index).reported==true)
					reward+=PENALTY_REPORT_REPEATED_MINE;
				else {	
					reward+=REWARD_CORRECT_REPORT;
					demining_state.mine_states_.at(index).reported=true;
				}
			}
			else reward+=PENALTY_WRONG_REPORT;
	 		break;
	 	}
		case A_NOT_MOVE: 
		{
			reward += COST_OBSERVE; break;
		}
		default:
		{
			break;
		}
	}

	if(demining_state.IsMineCell(demining_state.robot_state_.x, demining_state.robot_state_.y))
	{
		reward += PENALTY_STEP_OVER_MINE;
		obs = GetObs(demining_state,random);
		return true;
	}
	

 	obs = GetObs(demining_state,random);

 	if(demining_state.AllMinesClear()) return true;

 	return false;
 }

 OBS_TYPE Demining::GetObs(DeminingState demining_state, Random &random) const
 {
 	//west, east and north locations of the robot
	int west_x, west_y, east_x, east_y, north_x, north_y, south_x, south_y;

	bool west_is_mine, east_is_mine, north_is_mine, south_is_mine;

	demining_state.GetSurroundingPos(west_x, west_y, WEST);
	demining_state.GetSurroundingPos(east_x, east_y, EAST);
	demining_state.GetSurroundingPos(north_x, north_y, NORTH);
	demining_state.GetSurroundingPos(south_x, south_y, SOUTH);

	//generate observation for west cell
	if(west_x==-1) west_is_mine=false;
	else if(demining_state.IsMineCell(west_x, west_y)==true)
	{
		if( random.NextDouble() < PROB_MINE_CELL_OBS_MINE ) west_is_mine=true;
		else west_is_mine=false;
	}
	else
	{
		if( random.NextDouble() < PROB_NIL_CELL_OBS_MINE ) west_is_mine=true;
		else west_is_mine=false;
	}
	
	//generate observation for east cell
	if(east_x==-1) east_is_mine=false;
	else if(demining_state.IsMineCell(east_x, east_y)==true)
	{
		if( random.NextDouble() < PROB_MINE_CELL_OBS_MINE ) east_is_mine=true;
		else east_is_mine=false;
	}
	else
	{
		if( random.NextDouble() < PROB_NIL_CELL_OBS_MINE ) east_is_mine=true;
		else east_is_mine=false;
	}
	
	//generate observation for north cell
	if(north_x==-1) north_is_mine=false;
	else if(demining_state.IsMineCell(north_x, north_y)==true)
	{
		if( random.NextDouble() < PROB_MINE_CELL_OBS_MINE ) north_is_mine=true;
		else north_is_mine=false;
	}
	else
	{
		if( random.NextDouble() < PROB_NIL_CELL_OBS_MINE ) north_is_mine=true;
		else north_is_mine=false;
	}
	
	//generate observation for north cell
	if(south_x==-1) south_is_mine=false;
	else if(demining_state.IsMineCell(south_x, south_y)==true)
	{
		if( random.NextDouble() < PROB_MINE_CELL_OBS_MINE ) south_is_mine=true;
		else south_is_mine=false;
	}
	else
	{
		if( random.NextDouble() < PROB_NIL_CELL_OBS_MINE ) south_is_mine=true;
		else south_is_mine=false;
	}
	
	//return joint observation
	if(west_is_mine)
	{
		if(north_is_mine)
		{
			if(east_is_mine)
			{
				if(south_is_mine)
					return O_WEST_NORTH_EAST_SOUTH;
				else return O_WEST_NORTH_EAST;
			}
			else 
			{
				if(south_is_mine)
					return O_WEST_NORTH_SOUTH;
				else return O_WEST_NORTH;
			}
		}
		else
		{
			if(east_is_mine)
			{
				if(south_is_mine)
					return O_WEST_EAST_SOUTH;
				else return O_WEST_EAST;
			}
			else 
			{
				if(south_is_mine)
					return O_WEST_SOUTH;
				else return O_WEST;
			}
		}			
	}
	else
	{
		if(north_is_mine)
		{
			if(east_is_mine)
			{
				if(south_is_mine)
					return O_NORTH_EAST_SOUTH;
				else return O_NORTH_EAST;
			}
			else 
			{
				if(south_is_mine)
					return O_NORTH_SOUTH;
				else return O_NORTH;
			}
		}
		else
		{
			if(east_is_mine)
			{
				if(south_is_mine)
					return O_EAST_SOUTH;
				else return O_EAST;
			}
			else 
			{
				if(south_is_mine)
					return O_SOUTH;
				else return O_NIL;
			}
		}			
	}

 } 


/* ========================================================================
 * Functions related to beliefs and starting states.
 * ========================================================================*/
/**
 * Returns the observation probability: the probability of observing 'obs' after executing 'action' and reaching 'state'
 */
 double Demining::ObsProb(OBS_TYPE obs, const State &state, int action)  const
 {
	const DeminingState& demining_state = static_cast<const DeminingState&>(state);
	
	double prob_west;  //the probability of observing a mine in the west cell
	double prob_north; //the probability of observing a mine in the north cell
	double prob_east; //the probability of observing a mine in the east cell
	double prob_south; //the probability of observing a mine in the south cell
	
	//calculate the probability of observing mines in the west, north and east cell respectively
	demining_state.ProbOfObsvingMine(prob_west, prob_north, prob_east, prob_south);
	
 	switch(obs)
	{
		case O_NIL: 	// no mines in any surrounding cells
		{
			return (1-prob_west)*(1-prob_north)*(1-prob_east)*(1-prob_south);
		}
		case O_WEST: 	// one mine in the west cell, no mines in other cells
		{
			return prob_west*(1-prob_north)*(1-prob_east)*(1-prob_south);
		}
		case O_NORTH: 	// one mine in the north cell, no mines in other cells
		{
			return (1-prob_west)*prob_north*(1-prob_east)*(1-prob_south);
		}
		case O_EAST: 	// ...
		{
			return (1-prob_west)*(1-prob_north)*prob_east*(1-prob_south);
		}
		case O_WEST_EAST: // one mine in west cell, one mine in east cell, no mine in other cell
		{
			return prob_west*(1-prob_north)*prob_east*(1-prob_south);
		}
		case O_WEST_NORTH:	// ...
		{
			return prob_west*prob_north*(1-prob_east)*(1-prob_south);
		}
		case O_NORTH_EAST:	// ...
		{
			return (1-prob_west)*prob_north*prob_east*(1-prob_south);
		}
		case O_WEST_NORTH_EAST:
		{
			return prob_west*prob_north*prob_east*(1-prob_south);
		}
		
		case O_SOUTH: 	// no mines in the west, north and east cells
		{
			return (1-prob_west)*(1-prob_north)*(1-prob_east)*prob_south;
		}
		case O_WEST_SOUTH: 	// one mine in the west and south cells, no mines in other cells
		{
			return prob_west*(1-prob_north)*(1-prob_east)*prob_south;
		}
		case O_NORTH_SOUTH: 	
		{
			return (1-prob_west)*prob_north*(1-prob_east)*prob_south;
		}
		case O_EAST_SOUTH: 	// ...
		{
			return (1-prob_west)*(1-prob_north)*prob_east*prob_south;
		}
		case O_WEST_EAST_SOUTH: // one mine in west cell, one mine in east cell, one mine in the south cell, but no mine in north cell
		{
			return prob_west*(1-prob_north)*prob_east*prob_south;
		}
		case O_WEST_NORTH_SOUTH:	// ...
		{
			return prob_west*prob_north*(1-prob_east)*prob_south;
		}
		case O_NORTH_EAST_SOUTH:	// ...
		{
			return (1-prob_west)*prob_north*prob_east*prob_south;
		}
		case O_WEST_NORTH_EAST_SOUTH:
		{
			return prob_west*prob_north*prob_east*prob_south;
		}
		default: return 0; //observation is not defined
	}

 }

/**
 * Returns a starting state.
 */
 State * Demining::CreateStartState(string type)  const
 {
 	if(start_state_!=NULL)
 	{
 		delete start_state_;
 //		start_state_=NULL;
 	}
 	start_state_=new DeminingState();
 	start_state_->GenRandomState();
 	return start_state_;
 }

/**
 * Returns the initial belief.
 */
 Belief * Demining::InitialBelief(const State *start, string type)  const
 {
 	const DeminingState * start_state=static_cast<const DeminingState *>(start);
 	double initial_belief=1.0/(MAP_HEIGHT*MAP_WIDTH - 1);

 	for(int i=0; i<MAP_HEIGHT; i++){
		for(int j=0; j<MAP_WIDTH; j++){
			belief_table[i][j]=initial_belief;
		}
	}

	belief_table[start_state->robot_state_.y][start_state->robot_state_.x]=0;

	return new DeminingClosedFormBelief(this, belief_table, start_state->robot_state_.x, start_state->robot_state_.y);

 }



/* ========================================================================
 * Display
 * ========================================================================*/
/**
 * Prints a state.
 */
 void Demining::PrintState(const State &state, ostream &out)  const
 {
	return;
 	const DeminingState& demining_state = static_cast<const DeminingState &>(state);

 	char map[MAP_HEIGHT][MAP_WIDTH];
 	for(int i=0; i<MAP_HEIGHT; i++)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			map[i][j]='.';
 	}

 	map[demining_state.robot_state_.y][demining_state.robot_state_.x]='@';

 	for(int i=0; i<demining_state.mine_states_.size();i++)
 	{
 		if(demining_state.mine_states_.at(i).reported==false)
 			map[demining_state.mine_states_.at(i).y][demining_state.mine_states_.at(i).x]='M';
 		else map[demining_state.mine_states_.at(i).y][demining_state.mine_states_.at(i).x]='S';
 	}

 	for(int i=MAP_HEIGHT-1; i>=0; i--)
 	{
 		for(int j=0;j<MAP_WIDTH; j++)
 			out<<map[i][j];
 		out<<endl;
 	}

 }

/**
 * Prints an observation.
 */
 void Demining::PrintObs(const State &state, OBS_TYPE obs, ostream &out)  const
 {
	return;
 	const DeminingState& demining_state = static_cast<const DeminingState &>(state);
 	char neighbor_cell[3][3];

 	for(int i=0; i<3; i++)
 		for(int j=0; j<3; j++)
 			neighbor_cell[i][j]='.';

 	neighbor_cell[1][1]='@'; //robot position

 	switch(obs)
	{
		case O_NIL: 	// no mines in WEST_NORTH_EAST_SOUTH cells
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_WEST: 	// one mine in the left cell, no mines in other cells
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_NORTH: 	// one mine in the front cell, no mines in other cells
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_EAST: 	// ...
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_WEST_EAST: // one mine in left cell, one mine in right cell, no mine in front cell
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_WEST_NORTH:	// ...
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_NORTH_EAST:	// ...
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='*';
			break;
		}
		case O_WEST_NORTH_EAST:
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='*';
			break;
		}


		case O_SOUTH: 	// no mines in WEST_NORTH_EAST_SOUTH cells
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_WEST_SOUTH: 	// one mine in the left cell, no mines in other cells
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_NORTH_SOUTH: 	// one mine in the front cell, no mines in other cells
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_EAST_SOUTH: 	// ...
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_WEST_EAST_SOUTH: // one mine in left cell, one mine in right cell, no mine in front cell
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='*';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_WEST_NORTH_SOUTH:	// ...
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='*';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_NORTH_EAST_SOUTH:	// ...
		{
			neighbor_cell[1][0]='*';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='M';
			break;
		}
		case O_WEST_NORTH_EAST_SOUTH:
		{
			neighbor_cell[1][0]='M';
			neighbor_cell[2][1]='M';
			neighbor_cell[1][2]='M';
			neighbor_cell[0][1]='M';
			break;
		}
	}

	cout<<endl;
	for(int i=2; i>=0; i--)
	{
		for(int j=0; j<3; j++)
			out<<neighbor_cell[i][j];
		out<<endl;
	}
 }

/**
 * Prints an action.
 */
 void Demining::PrintAction(int action, ostream &out)  const
 {
 	switch(action)
 	{
	 	case A_NORTH:      // move forward
	 	{
	 		out<<"NORTH"<<endl;
			break;
	 	}
	    case A_SOUTH:     // move backward
	    {
	 		out<<"SOUTH"<<endl;
			break;
	 	}
	    case A_WEST:         // move left
	    {
	 		out<<"WEST"<<endl;
			break;
	 	}
	    case A_EAST:        // move right
	    {
			out<<"EAST"<<endl;
			break;
	 	}
	    case A_REPORT_WEST:  // report that there is a mine on the left of the robot
	    {
			out<<"REPORT_WEST"<<endl;
			break;
	 	}
	    case A_REPORT_EAST: // report that there is a mine on the right of the robot
	    {
			out<<"REPORT_EAST"<<endl;
			break;
	 	}
	    case A_REPORT_NORTH:  // report that there is a mine in front of the robot
	    {
			out<<"REPORT_NORTH"<<endl;
			break;
	 	}
	 	case A_REPORT_SOUTH:  // report that there is a mine in front of the robot
	    {
			out<<"REPORT_SOUTH"<<endl;
			break;
	 	}
	 	case A_NOT_MOVE:  // report that there is a mine in front of the robot
	    {
			out<<"NOT_MOVE"<<endl;
			break;
	 	}
 	}
 }

/**
 * Prints a belief.
 */
 void Demining::PrintBelief(const Belief &belief, ostream &out) const
 {
	return;
 	for(int i=MAP_HEIGHT-1;i>=0; i--){
		for(int j=0; j<MAP_WIDTH; j++){
			if(belief_table[i][j]>=0.0000099999999) 
 				out<<setw(12)<<belief_table[i][j];
 			else out<<"           "<<setw(1)<<0;
		}
		cout<<endl;
	}
 } 


 void Demining::PrintParticles(const vector<State*> particles, ostream& out) const
 {
 	return;

 	int particle_num = particles.size();
 	out<<particle_num<<endl;

 	double total_weight=0;

 	double particle_belief[MAP_HEIGHT][MAP_WIDTH];


 	for(int i=0;i<MAP_HEIGHT;i++){
 		for(int j=0; j<MAP_WIDTH;j++){
 			particle_belief[i][j]=0;
 		}
 	}

 	for(int i=0; i<particle_num; i++)
 	{
 		const DeminingState* demining_state=static_cast<const DeminingState*>(particles.at(i));
 		
 		for(int j=0; j<demining_state->mine_states_.size(); j++){
 			particle_belief[demining_state->mine_states_.at(j).y][demining_state->mine_states_.at(j).x] += demining_state->weight;
 		}
 		total_weight += demining_state->weight;
 	}

 	for(int i=MAP_HEIGHT-1;i>=0; i--){
		for(int j=0; j<MAP_WIDTH; j++){
			if(particle_belief[i][j]>=0.0000099999999) 
 				out<<setw(12)<<particle_belief[i][j];
 			else out<<"           "<<setw(1)<<0;
		}
		cout<<endl;
	}
 }

/* ========================================================================
 * Memory management.
 * ========================================================================*/
/**
 * Allocate a state.
 */
 State * Demining::Allocate(int state_id, double weight)  const
 {
 	DeminingState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
 }

/**
 * Returns a copy of the state.
 */
 State * Demining::Copy(const State *state)  const
 {
 	DeminingState* copy_state=memory_pool_.Allocate();
 	*copy_state = * static_cast<const DeminingState*>(state);
 	copy_state->SetAllocated();
 	return copy_state;
 }

/**
 * Returns a copy of the particle.
 */
 void Demining::Free(State *state)  const
 {
 	memory_pool_.Free(static_cast<DeminingState*>(state));
 }

/**
 * Returns number of allocated particles.
 */
 int Demining::NumActiveParticles()  const
 {
 	return memory_pool_.num_allocated();
 }


 vector<double> Demining::ImportanceWeight(vector<State*> particles) const
 {
 	double total_weight=State::Weight(particles);
 	double new_total_weight = 0;
 	
	int particles_num=particles.size();

	vector<DeminingState*> demining_particles;

	vector <double> importance_weight;

	for(int i=0; i<particles_num;i++){
		demining_particles.push_back(static_cast<DeminingState*>(particles[i]));
	}

	for(int i=0; i<particles_num;i++){
		int dist = demining_particles[i]->GetDistToNearestMine();
		int num_mines = demining_particles[i]->mine_states_.size();
		importance_weight.push_back(demining_particles[i]->weight*sampling_weight_[num_mines][dist]);
		new_total_weight += importance_weight[i];
	}
	
	//normalize to total_weight
	for(int i=0; i<particles_num;i++){
		importance_weight[i]=importance_weight[i]*total_weight/new_total_weight;
	}

	return importance_weight;

 }


 vector<double> Demining::Feature(const State& state) const {
	vector<double> feature;
	
	if(Globals::config.collect_data == false) return feature;
	
	const DeminingState& demining_state = static_cast<const DeminingState&>(state);
	feature.push_back(demining_state.mine_states_.size());	
	feature.push_back(demining_state.NumOfUnreportedMines());
	feature.push_back(demining_state.GetDistToNearestMine());
	feature.push_back(demining_state.GetDistToNearestUnreportedMine());


	return feature;
}
