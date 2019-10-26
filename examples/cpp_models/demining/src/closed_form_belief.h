#ifndef CLOSED_FORM_BELIEF
#define CLOSED_FORM_BELIEF


#include "demining_state.h"


//class DeminingState;

class DeminingClosedFormBelief: public Belief{
public:
	double (* belief_table_)[MAP_WIDTH];
	int robot_pos_x_;
	int robot_pos_y_;
	std::vector <MineState> reported_mines_;
public:
	DeminingClosedFormBelief(const DSPOMDP* model, double (* belief_table)[MAP_WIDTH], int robot_pos_x, int robot_pos_y);
	virtual ~DeminingClosedFormBelief();

	virtual vector<State*> Sample(int num) const;
	virtual vector<State*> Sample(int num, vector<State*> particles, const DSPOMDP* model) const;
	virtual void Update(int action, OBS_TYPE obs);
	virtual Belief* MakeCopy() const;

	bool IsReported(int x, int y) const;

};

#endif
