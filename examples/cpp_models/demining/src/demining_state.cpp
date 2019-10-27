#include "demining_state.h"

DeminingState::DeminingState()
{

}

DeminingState::DeminingState(RobotState robot_state, std::vector<MineState> mine_states)
{
	robot_state_=robot_state;
	mine_states_=mine_states;
}

//check whether the all the cells around the robot are mine cells or out of boundary
bool DeminingState::IsAllMinesAround(int x, int y)
{
	//bool all_mines_around=true;
	//let robot position be (x,y) for temporary use
	robot_state_.x=x;
	robot_state_.y=y;

	int surround_pos_x, surround_pos_y;

	GetSurroundingPos(surround_pos_x, surround_pos_y, WEST);
	if(surround_pos_x==-1 || IsMineCell(surround_pos_x, surround_pos_y)) ;//need further checking
	else return false;

	GetSurroundingPos(surround_pos_x, surround_pos_y, EAST);
	if(surround_pos_x==-1 || IsMineCell(surround_pos_x, surround_pos_y)) ;//need further checking
	else return false;

	GetSurroundingPos(surround_pos_x, surround_pos_y, NORTH);
	if(surround_pos_x==-1 || IsMineCell(surround_pos_x, surround_pos_y)) ;//need further checking
	else return false;

	GetSurroundingPos(surround_pos_x, surround_pos_y, SOUTH);
	if(surround_pos_x==-1 || IsMineCell(surround_pos_x, surround_pos_y)) return true;
	else return false;

}

//generate random state
void DeminingState::GenRandomState()
{
	int x, y;

	mine_states_.clear();

 	double initial_belief=double(EXPECTED_TOTAL_MINE_NUM)/(MAP_HEIGHT*MAP_WIDTH); 

 	for(int i=0; i<MAP_HEIGHT; i++){
		for(int j=0; j<MAP_WIDTH; j++){
			//belief_table[i][j]=initial_belief;
			if((rand()%10000)/10000.0 < initial_belief) {
				MineState mine;
				mine.x=j;
				mine.y=i;
				mine.reported=false;

				mine_states_.push_back(mine);
			}
		}
	}

	//generate random robot position
	x=rand()%MAP_WIDTH;
	y=rand()%MAP_HEIGHT;

	//check collision
	while(IsMineCell(x,y) || IsAllMinesAround(x,y))
	{
		x=rand()%MAP_WIDTH;
		y=rand()%MAP_HEIGHT;
	}

	robot_state_.x=x;
	robot_state_.y=y;
}

//generate DeminingState with random MineStates but deterministic RobotState
void DeminingState::GenStateWithRandomMine(RobotState robot_state)
{
}

//calculate the probability of observing mines in the west, north, east and south cell respectively
void DeminingState::ProbOfObsvingMine(double &prob_west_cell, double &prob_north_cell, double &prob_east_cell, double &prob_south_cell) const
{
	int x, y;
	
	GetSurroundingPos(x,y, WEST);
	if(x==-1) prob_west_cell=0;
	else if(IsMineCell(x, y) == true) prob_west_cell = PROB_MINE_CELL_OBS_MINE;
	else prob_west_cell = PROB_NIL_CELL_OBS_MINE;
	
	GetSurroundingPos(x,y, NORTH);
	if(x==-1) prob_north_cell=0;
	else if(IsMineCell(x, y) == true) prob_north_cell = PROB_MINE_CELL_OBS_MINE;
	else prob_north_cell = PROB_NIL_CELL_OBS_MINE;
	
	GetSurroundingPos(x,y, EAST);
	if(x==-1) prob_east_cell=0;
	else if(IsMineCell(x, y) == true) prob_east_cell = PROB_MINE_CELL_OBS_MINE;
	else prob_east_cell = PROB_NIL_CELL_OBS_MINE;
	
	GetSurroundingPos(x,y, SOUTH);
	if(x==-1) prob_south_cell=0;
	else if(IsMineCell(x, y) == true) prob_south_cell = PROB_MINE_CELL_OBS_MINE;
	else prob_south_cell = PROB_NIL_CELL_OBS_MINE;
	
}

void DeminingState::GetSurroundingPos(int &x, int &y, int direction) const
{
	switch(direction)
	{
		case WEST:
		{
			x=robot_state_.x-1;
			y=robot_state_.y;
			break;
		}
		case EAST:
		{
			x=robot_state_.x+1;
			y=robot_state_.y;
			break;
		}
		case NORTH:
		{
			x=robot_state_.x;
			y=robot_state_.y+1;
			break;
		}
		case SOUTH:
		{
			x=robot_state_.x;
			y=robot_state_.y-1;
			break;
		}
		default:
		{
			x=-1;
			y=-1;
			break;
		}
	}
	
	if(x>MAP_WIDTH-1 || x<0
		|| y>MAP_HEIGHT-1 || y<0)
	{
		x=-1;
		y=-1;
	}
}


bool DeminingState::IsMineCell(int x, int y) const
{
	for(int i=0; i<mine_states_.size();i++)
		if(x==mine_states_.at(i).x && y==mine_states_.at(i).y)
			return true;
			
	return false;
}

bool DeminingState::IsMineCell(int x, int y, int& index) const
{
	for(int i=0; i<mine_states_.size();i++)
		if(x==mine_states_.at(i).x && y==mine_states_.at(i).y)
		{
			index=i;
			return true;
		}
	
	index=-1;	
	return false;
}

bool DeminingState::AllMinesClear()
{
	for(int i=0; i<mine_states_.size();i++)
		if(mine_states_[i].reported==false) return false;
	
	return true;
}

int DeminingState::GetDistToNearestMine() const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
		{
			x_dis = abs(mine_states_.at(i).x - robot_state_.x);
			y_dis = abs(mine_states_.at(i).y - robot_state_.y);
		}
	}

	return x_dis + y_dis;
}


void DeminingState::GetDistToNearestMine(int &x, int &y) const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
		{
			x_dis = abs(mine_states_.at(i).x - robot_state_.x);
			y_dis = abs(mine_states_.at(i).y - robot_state_.y);
		}
	}

	x=x_dis;
	y=y_dis;
}


int DeminingState::GetDistToNearestReportedMine() const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).reported == true))
		{
			if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
			{
				x_dis = abs(mine_states_.at(i).x - robot_state_.x);
				y_dis = abs(mine_states_.at(i).y - robot_state_.y);
			}
		}
		
	}

	return x_dis + y_dis;
}


void DeminingState::GetDistToNearestReportedMine(int &x, int &y) const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).reported == true))
		{
			if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
			{
				x_dis = abs(mine_states_.at(i).x - robot_state_.x);
				y_dis = abs(mine_states_.at(i).y - robot_state_.y);
			}
		}
		
	}

	x=x_dis;
	y=y_dis;
}


int DeminingState::GetDistToNearestUnreportedMine() const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).reported == false))
		{
			if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
			{
				x_dis = abs(mine_states_.at(i).x - robot_state_.x);
				y_dis = abs(mine_states_.at(i).y - robot_state_.y);
			}
		}
		
	}

	return x_dis + y_dis;
}

void DeminingState::GetDistToNearestUnreportedMine(int &x, int &y) const
{
	int x_dis=1000, y_dis=1000;

	for(int i=0; i<mine_states_.size();i++)
	{
		if(abs(mine_states_.at(i).reported == false))
		{
			if(abs(mine_states_.at(i).x - robot_state_.x) + abs(mine_states_.at(i).y - robot_state_.y)
			< x_dis + y_dis)
			{
				x_dis = abs(mine_states_.at(i).x - robot_state_.x);
				y_dis = abs(mine_states_.at(i).y - robot_state_.y);
			}
		}
		
	}

	x=x_dis;
	y=y_dis;
}

void DeminingState::SetMines(std::vector <MineState> mine_states)
{
	for(int i=0; i<mine_states.size();i++)
	{
		mine_states_.push_back(mine_states.at(i));
	}
	
}

int DeminingState::NumOfUnreportedMines() const
{
	int num_unreported_mines=0;
	for(int i=0; i<mine_states_.size();i++){
		if(mine_states_.at(i).reported == false) num_unreported_mines++;
	} 
	return num_unreported_mines;
}