#include "rock_sample.h"

using namespace std;

namespace despot {

/* =============================================================================
 * RockSample class
 * =============================================================================*/

RockSample::RockSample(string map) :
	BaseRockSample(map) {
	half_efficiency_distance_ = 20;
}

RockSample::RockSample(int size, int rocks) :
	BaseRockSample(size, rocks) {
	half_efficiency_distance_ = 20;

	sampling_weight_.resize(rocks+1);
	for(int i=0; i<rocks+1; i++){
		sampling_weight_[i].resize(size*2);
		for(int j=0; j<size*2; j++)
			sampling_weight_[i][j].resize(size*2);
	}

	if(Globals::config.use_is_despot == false){
		cout<<"use_is_despot is set to false; will not use importance sampling"<<endl;
		for(int i=0; i<rocks+1; i++)
			for(int j=0; j<size*2; j++)
				for(int k=0; k<size*2; k++)
					sampling_weight_[i][j][k]=1;
		return;
	}

	ifstream file;
	if(size==7 && rocks==8){
		file.open("sampling_weight7x8.dat", std::ifstream::in);
	}
	else if(size==11 && rocks==11){
		file.open("sampling_weight11x11.dat", std::ifstream::in);
	}
	else if(size==15 && rocks==15){
		file.open("sampling_weight15x15.dat", std::ifstream::in);
	}
	else{
		cout<<"don't use importance sampling"<<endl;
		for(int i=0; i<rocks+1; i++)
			for(int j=0; j<size*2; j++)
				for(int k=0; k<size*2; k++)
					sampling_weight_[i][j][k]=1;
		return;
	}

	if(file.fail()) {
		cout<<"fail to open sampling_weight file"<<endl;
		cout<<"will not use importance sampling"<<endl;
		for(int i=0; i<rocks+1; i++)
			for(int j=0; j<size*2; j++)
				for(int k=0; k<size*2; k++)
					sampling_weight_[i][j][k]=1;
		return;
	}
	
	for(int i=0; i<rocks+1; i++)
		for(int j=0; j<size*2; j++)
			for(int k=0; k<size*2; k++)
				sampling_weight_[i][j][k]=0;

	int num_good_rock,min_dist_good,min_dist_bad; //power step, manhattan distance
	double mean, std, ste, importance, global_importance;
	while(file>>num_good_rock>>min_dist_good>>min_dist_bad>>mean>>std>>ste>>importance>>global_importance) {
		if(min_dist_good == 100 && min_dist_bad == 100) sampling_weight_[num_good_rock][size*2-1][size*2-1]=importance;
		else if(min_dist_good == 100) sampling_weight_[num_good_rock][size*2-1][min_dist_bad]=importance;
		else if(min_dist_bad == 100) sampling_weight_[num_good_rock][min_dist_good][size*2-1]=importance; 
		else sampling_weight_[num_good_rock][min_dist_good][min_dist_bad]=importance;
	}
	
	for(int i=0; i<rocks+1; i++)
		for(int j=0; j<size*2; j++)	
			for (int k=0; k<size*2; k++){
				if(sampling_weight_[i][j][k]==0) {
					if(j>0) sampling_weight_[i][j][k]=sampling_weight_[i][j-1][k];
					else {
							int next_j=j;
							while(next_j!=size*2 && sampling_weight_[i][next_j][k]==0){
								next_j++;
							if(next_j!=size*2) sampling_weight_[i][j][k]=sampling_weight_[i][next_j][k];
						}
					}
				}
				if(sampling_weight_[i][j][k]==0) {
					if(k>0) sampling_weight_[i][j][k]=sampling_weight_[i][j][k-1];
					else {
							int next_k=k;
							while(next_k!=size*2 && sampling_weight_[i][j][next_k]==0){
								next_k++;
							if(next_k!=size*2) sampling_weight_[i][j][k]=sampling_weight_[i][j][next_k];
						}
					}
				}

				if(sampling_weight_[i][j][k]==0) {
					if(i>0) sampling_weight_[i][j][k]=sampling_weight_[i-1][j][k];
					else {
							int next_i=i;
							while(next_i!=rocks+1 && sampling_weight_[next_i][j][k]==0){
								next_i++;
							if(next_i!=rocks+1) sampling_weight_[i][j][k]=sampling_weight_[next_i][j][k];
						}
					}
				}

				assert(sampling_weight_[i][j][k]!=0);
			
			}

	file.close();

}

bool RockSample::Step(State& state, double rand_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	RockSampleState& rockstate = static_cast<RockSampleState&>(state);
	reward = 0;
	obs = E_NONE;

	if (action < E_SAMPLE) { // Move
		switch (action) {
		case Compass::EAST:
			if (GetX(&rockstate) + 1 < size_) {
				IncX(&rockstate);
				break;
			} else {
				reward = +10;
				return true;
			}

		case Compass::NORTH:
			if (GetY(&rockstate) + 1 < size_)
				IncY(&rockstate);
			else
				reward = -100;
			break;

		case Compass::SOUTH:
			if (GetY(&rockstate) - 1 >= 0)
				DecY(&rockstate);
			else
				reward = -100;
			break;

		case Compass::WEST:
			if (GetX(&rockstate) - 1 >= 0)
				DecX(&rockstate);
			else
				reward = -100;
			break;
		}
	}

	if (action == E_SAMPLE) { // Sample
		int rock = grid_(GetRobPosIndex(&rockstate));
		if (rock >= 0) {
			if (GetRock(&rockstate, rock))
				reward = +10;
			else
				reward = -10;
			SampleRock(&rockstate, rock);
		} else {
			reward = -100;
		}
	}

	if (action > E_SAMPLE) { // Sense
		int rock = action - E_SAMPLE - 1;
		assert(rock < num_rocks_);
		obs = GetObservation(rand_num, rockstate, rock);
	}

	// assert(reward != -100);
	return false;
}

int RockSample::NumActions() const {
	return num_rocks_ + 5;
}

double RockSample::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
	if (action <= E_SAMPLE)
		return obs == E_NONE;

	if (obs != E_GOOD && obs != E_BAD)
		return 0;

	const RockSampleState& rockstate =
		static_cast<const RockSampleState&>(state);

	int rock = action - E_SAMPLE - 1;
	double distance = Coord::EuclideanDistance(GetRobPos(&rockstate),
		rock_pos_[rock]);
	double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
		* 0.5;

	return
		((GetRock(&rockstate, rock) & 1) == obs) ? efficiency : (1 - efficiency);
}

void RockSample::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	switch (observation) {
	case E_NONE:
		out << "None" << endl;
		break;
	case E_GOOD:
		out << "Good" << endl;
		break;
	case E_BAD:
		out << "Bad" << endl;
		break;
	}
}

vector<double> RockSample::Feature(const State& state) const {
	vector<double> feature;

	if(Globals::config.collect_data == false) return feature;
	const RockSampleState& rockstate = static_cast<const RockSampleState&>(state);
	
	int min_good_rock_dist=100;
	int min_bad_rock_dist=100;
	int num_good_rock=0;
	for(int rock=0; rock<num_rocks_; rock++){
		int dist=Coord::ManhattanDistance(GetRobPos(&rockstate),rock_pos_[rock]);
		int status = GetRock(&state, rock); //GOOD if status==1, BAD if status==0	
		if(status){
			if(dist<min_good_rock_dist) min_good_rock_dist=dist;
			num_good_rock++;
		}
		else{
			if(dist<min_bad_rock_dist) min_bad_rock_dist=dist;
		}
	}
	feature.push_back(num_good_rock);
	feature.push_back(min_good_rock_dist);
	feature.push_back(min_bad_rock_dist);

	return feature;
}

vector<double> RockSample::ImportanceWeight(vector<State*> particles) const
{
	double total_weight=State::Weight(particles);
	double sum_of_weight=0;
	
	int particles_num=particles.size();
	vector<RockSampleState*> rocksample_particles;
	vector <double> importance_weight;
	for(int i=0; i<particles_num;i++){
		rocksample_particles.push_back(static_cast<RockSampleState*>(particles[i]));
	}

	for(int i=0; i<particles_num;i++){
		int min_good_rock_dist=size_*2-1;
		int min_bad_rock_dist=size_*2-1;
		int num_good_rock=0;
		for(int rock=0; rock<num_rocks_; rock++){
			int dist=Coord::ManhattanDistance(GetRobPos(rocksample_particles[i]),rock_pos_[rock]);
			int status = GetRock(particles[i], rock); //GOOD if status==1, BAD if status==0	
			if(status){
				if(dist<min_good_rock_dist) min_good_rock_dist=dist;
				num_good_rock++;
			}
			else{
				if(dist<min_bad_rock_dist) min_bad_rock_dist=dist;
			}
		}

		importance_weight.push_back(rocksample_particles[i]->weight * sampling_weight_[num_good_rock][min_good_rock_dist][min_bad_rock_dist]);
		sum_of_weight += importance_weight[i];
	}

	//normalize
	for(int i=0; i<particles_num;i++){
		importance_weight[i] = importance_weight[i]*total_weight/sum_of_weight;
	}

	return importance_weight;
}


} // namespace despot
