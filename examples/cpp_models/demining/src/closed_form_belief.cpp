#include "closed_form_belief.h"
#include <iomanip>

DeminingClosedFormBelief::DeminingClosedFormBelief(const DSPOMDP* model, double (* belief_table)[MAP_WIDTH], 
	int robot_pos_x, int robot_pos_y):
	Belief(model),
	belief_table_(belief_table),
	robot_pos_x_(robot_pos_x),
	robot_pos_y_(robot_pos_y)
{

}


DeminingClosedFormBelief::~DeminingClosedFormBelief()
{
}

vector<State*> DeminingClosedFormBelief::Sample(int num) const
{

	std::vector<State*> scenarios;

	double weight=1.0/15000;

	for(int ii=0; ii<15000; ii++){
		vector <MineState> mine_states;
		for(int i=0; i<MAP_HEIGHT; i++){
			for(int j=0; j<MAP_WIDTH; j++){
				double rand_num = (rand()%100000)/100000.0;
				if(rand_num<=belief_table_[i][j]){
					MineState mine_state;
					mine_state.x=j;
					mine_state.y=i;
					if(IsReported(mine_state.x, mine_state.y)==true) mine_state.reported=true;
					else mine_state.reported=false;
					mine_states.push_back(mine_state);
				}
			}
		}
		DeminingState *scenario = new DeminingState;
		scenario->SetMines(mine_states);
		scenario->robot_state_.x=robot_pos_x_;
		scenario->robot_state_.y=robot_pos_y_;
		scenario->weight = weight;

		State* one_scenario=model_->Copy(scenario);
		scenarios.push_back(one_scenario);

		delete scenario;
	}

	vector<State*> scenarios_to_return = Sample(num, scenarios, model_);
	for(int i=0; i<scenarios.size(); i++)
	{
		model_->Free(scenarios.at(i));
	}
	
	return scenarios_to_return;
}


bool DeminingClosedFormBelief::IsReported(int x, int y) const
{
	for(int i=0; i<reported_mines_.size(); i++){
		if(x==reported_mines_.at(i).x && y==reported_mines_.at(i).y)
			return true;
	}
	return false;
}

void DeminingClosedFormBelief::Update(int action, OBS_TYPE obs)
{
	//update the belief on the robot position -- it is deterministic
	switch(action)
 	{
	 	case ACTION_NORTH:      // move up
	 	{
	 		if(robot_pos_y_+1<MAP_HEIGHT)
	 				robot_pos_y_++;	 			
			break;
	 	}
	    case ACTION_SOUTH:     // move down
	    {
	 		if(robot_pos_y_-1>=0)
	 				robot_pos_y_--;	 			
			break;
	 	}
	    case ACTION_WEST:         // move left
	    {
	 		if(robot_pos_x_-1>=0)
	 				robot_pos_x_--;	 			
			break;
	 	}
	    case ACTION_EAST:        // move right
	    {
			if(robot_pos_x_+1<MAP_WIDTH)
	 				robot_pos_x_++;	 			
			break;
	 	}
	 	case ACTION_REPORT_NORTH:      // move up
	 	{
	 		if(robot_pos_y_+1<MAP_HEIGHT){
	 			MineState reported_mine;
	 			reported_mine.x=robot_pos_x_;
	 			reported_mine.y=robot_pos_y_+1;
	 			reported_mines_.push_back(reported_mine);	
	 		}
	 				 			
			break;
	 	}
	    case ACTION_REPORT_SOUTH:     // move down
	    {
	 		if(robot_pos_y_-1>=0){
	 			MineState reported_mine;
	 			reported_mine.x=robot_pos_x_;
	 			reported_mine.y=robot_pos_y_-1;
	 			reported_mines_.push_back(reported_mine);	
	 		}	 			
			break;
	 	}
	    case ACTION_REPORT_WEST:         // move left
	    {
	 		if(robot_pos_x_-1>=0){
	 			MineState reported_mine;
	 			reported_mine.x=robot_pos_x_-1;
	 			reported_mine.y=robot_pos_y_;
	 			reported_mines_.push_back(reported_mine);	
	 		} 			
			break;
	 	}
	    case ACTION_REPORT_EAST:        // move right
	    {
			if(robot_pos_x_+1<MAP_WIDTH){
	 			MineState reported_mine;
	 			reported_mine.x=robot_pos_x_+1;
	 			reported_mine.y=robot_pos_y_;
	 			reported_mines_.push_back(reported_mine);	
	 		}		
			break;
	 	}
	}

	bool pos_valid[4];//whether the position is valid
	if(robot_pos_y_+1 < MAP_HEIGHT) pos_valid[NORTH]=true;
	else pos_valid[NORTH]=false;

	if(robot_pos_y_-1 >= 0) pos_valid[SOUTH]=true;
	else pos_valid[SOUTH]=false;

	if(robot_pos_x_+1 < MAP_WIDTH) pos_valid[EAST]=true;
	else pos_valid[EAST]=false;

	if(robot_pos_x_-1 >= 0) pos_valid[WEST]=true;
	else pos_valid[WEST]=false;

	/*********
	assume each cell is independent, then based on Bayes rule, we have:

	if we observe mine, then update prob based on:
	p(is mine | obs mine) = p(obs mine | is mine) * p(is mine) / p(obs mine)
	p(is nil | obs mine) = p(obs mine | is nil) * p(is nil) / p(obs mine)
	and then we can normalize

	if we observe nil, then update prob based on:
	p(is mine | obs nil) = p(obs nil | is mine) * p(is mine) / p(obs nil)
	p(is nil | obs nil) = p(obs nil | is nil) * p(is nil) / p(obs nil)
	and then we can normalize
	*********/

	switch(obs)
	{
		case OBSER_NIL:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_WEST:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_NORTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_EAST:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_WEST_EAST:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		//////
		case OBSER_WEST_NORTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_NORTH_EAST:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_WEST_NORTH_EAST:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			break;
		}
		case OBSER_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		case OBSER_WEST_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		case OBSER_NORTH_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		///from here
		case OBSER_EAST_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		case OBSER_WEST_EAST_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		case OBSER_WEST_NORTH_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}

		//////(())
		case OBSER_NORTH_EAST_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_nil = (1-PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_nil = (1-PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_nil + prob_is_nil_obs_nil <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_nil / (prob_is_mine_obs_nil + prob_is_nil_obs_nil);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
		case OBSER_WEST_NORTH_EAST_SOUTH:
		{
			if(pos_valid[WEST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_-1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_-1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_-1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[NORTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_+1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_+1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_+1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[EAST]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_][robot_pos_x_+1];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_][robot_pos_x_+1]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_][robot_pos_x_+1] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			if(pos_valid[SOUTH]==true)
			{
				double prob_is_mine_obs_mine = (PROB_MINE_CELL_OBS_MINE) * belief_table_[robot_pos_y_-1][robot_pos_x_];
				double prob_is_nil_obs_mine = (PROB_NIL_CELL_OBS_MINE) * (1-belief_table_[robot_pos_y_-1][robot_pos_x_]);
				if(prob_is_mine_obs_mine + prob_is_nil_obs_mine <=0) {cout<<"error: prob<=0"<<endl;return;}
				//normalize
				belief_table_[robot_pos_y_-1][robot_pos_x_] = prob_is_mine_obs_mine / (prob_is_mine_obs_mine + prob_is_nil_obs_mine);
			}
			break;
		}
	}

}

vector<State*> DeminingClosedFormBelief::Sample(int num, vector<State*> particles,
	const DSPOMDP* model) const {
	
	if(Globals::config.use_is_despot) { //use importance sampling for belief
		double unit = 1.0 / num;
		double mass = Random::RANDOM.NextDouble(0, unit);
		int pos = 0;

		vector <double> importance_weight = model->ImportanceWeight(particles);

		double cur = importance_weight[0];

		double total_weight=0;

		vector<State*> sample;
		for (int i = 0; i < num; i++) {
			while (mass > cur) {
				pos++;
				if (pos == particles.size())
					pos = 0;

				cur += importance_weight[pos];
			}

			assert(importance_weight[pos]!=0);
			mass += unit;

			State* particle = model->Copy(particles[pos]);
			particle->weight = unit*(particles[pos]->weight/importance_weight[pos]);
			total_weight+=particle->weight;
			sample.push_back(particle);
		}

		for (int i=0; i<num; i++)
			sample[i]->weight /= total_weight;

		random_shuffle(sample.begin(), sample.end());

		logd << "[Belief::Sample] Sampled " << sample.size() << " particles"
			<< endl;
		for (int i = 0; i < sample.size(); i++) {
			logv << " " << i << " = " << *sample[i] << endl;
		}

		return sample;
	}
	else{ // don't use importance sampling for initial belief
		double unit = 1.0 / num;
		double mass = Random::RANDOM.NextDouble(0, unit);
		int pos = 0;
		double cur = particles[0]->weight;

		vector<State*> sample;
		for (int i = 0; i < num; i++) {
			while (mass > cur) {
				pos++;
				if (pos == particles.size())
					pos = 0;

				cur += particles[pos]->weight;
			}

			mass += unit;

			State* particle = model->Copy(particles[pos]);
			particle->weight = unit;
			sample.push_back(particle);
		}

		random_shuffle(sample.begin(), sample.end());

		logd << "[Belief::Sample] Sampled " << sample.size() << " particles"
			<< endl;
		for (int i = 0; i < sample.size(); i++) {
			logv << " " << i << " = " << *sample[i] << endl;
		}

		return sample;
	}
}


Belief* DeminingClosedFormBelief::MakeCopy() const 
{
	//return new DeminingClosedFormBelief(model_, belief_table);
	return NULL;
}
