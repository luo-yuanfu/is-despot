#include "tiger.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

const ACT_TYPE Tiger::LEFT = 0;
const ACT_TYPE Tiger::RIGHT = 1;
const ACT_TYPE Tiger::LISTEN = 2;
const double Tiger::NOISE = 0.15;

/* =============================================================================
 * TigerState class
 * =============================================================================*/

TigerState::TigerState() :
	tiger_position(0) {
}

TigerState::TigerState(int position) :
	tiger_position(position) {
}

TigerState::TigerState(double rand_num){
	if(rand_num<0.99) tiger_position = 0;
	else tiger_position = 1;

}


string TigerState::text() const {
	return tiger_position == Tiger::LEFT ? "LEFT" : "RIGHT";
}

/* =============================================================================
 * OptimalTigerPolicy class
 * =============================================================================*/

class OptimalTigerPolicy: public DefaultPolicy {
public:
	OptimalTigerPolicy(const DSPOMDP* model,
		ParticleLowerBound* bound) :
		DefaultPolicy(model, bound) {
	}

	// NOTE: optimal for noise = 0.15
	ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {
		/*
		 if (history.Size() == 0 || history.LastAction() != LISTEN) {
		 actions->push_back(LISTEN);
		 return actions;
		 }

		 actions->push_back(history.LastObservation());
		 */

		int count_diff = 0;
		for (int i = history.Size() - 1;
			i >= 0 && history.Action(i) == Tiger::LISTEN; i--)
			count_diff += history.Observation(i) == Tiger::LEFT ? 1 : -1;

		if (count_diff >= 2)
			return Tiger::RIGHT;
		else if (count_diff <= -2)
			return Tiger::LEFT;
		else
			return Tiger::LISTEN;
	}
};

/* =============================================================================
 * Tiger class
 * =============================================================================*/

Tiger::Tiger() {
	if(Globals::config.use_is_despot == false){
		cout<<"use_is_despot is set to false; will not use importance sampling"<<endl;
		sampling_weight_[0]=1;
		sampling_weight_[1]=1;
		return;
	}

	ifstream file;
	file.open("sampling_weight.dat", std::ifstream::in);
	if(file.fail()){
		cout<<"fail to open sampling_weight.dat file"<<endl;
		cout<<"will not use importance sampling"<<endl;
		sampling_weight_[0]=1;
		sampling_weight_[1]=1;
		return;
	}

	int feature;
	double mean, std, ste, importance, global_importance;
	while(file>>feature>>mean>>std>>ste>>importance>>global_importance){
		sampling_weight_[feature]=importance;
	}
	file.close();

	return;

}

bool Tiger::Step(State& s, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	TigerState& state = static_cast<TigerState&>(s);

	if (action == LEFT || action == RIGHT) {
		if(state.tiger_position == LEFT){
			if(action == LEFT)
				reward = -100;
			else reward = 10;
		}
		else{
			if(action == RIGHT)
				reward = -10000;
			else reward = 10;
		}
		obs = 2; // can use arbitary observation
		return true;
	} else {
		reward = -1;
		if (random_num <= 1 - NOISE)
			obs = state.tiger_position;
		else
			obs = (LEFT + RIGHT - state.tiger_position);
	}
	return false;

}

int Tiger::NumStates() const {
	return 2;
}

int Tiger::NumActions() const {
	return 3;
}

double Tiger::ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const {
	const TigerState& state = static_cast<const TigerState&>(s);

	if (a != LISTEN)
		return obs == 2;

	return state.tiger_position == obs ? (1 - NOISE) : NOISE;
}

State* Tiger::CreateStartState(string type) const {
	return new TigerState(Random::RANDOM.NextDouble());
}

Belief* Tiger::InitialBelief(const State* start, string type) const {
	vector<State*> particles;
	TigerState* left = static_cast<TigerState*>(Allocate(-1, 0.99));
	left->tiger_position = LEFT;
	particles.push_back(left);
	TigerState* right = static_cast<TigerState*>(Allocate(-1, 0.01));
	right->tiger_position = RIGHT;
	particles.push_back(right);
	return new ParticleBelief(particles, this);
}

ScenarioLowerBound* Tiger::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	ScenarioLowerBound* bound = NULL;
	if (name == "TRIVIAL" || name == "DEFAULT") {
		bound = new TrivialParticleLowerBound(this);
	} else if (name == "RANDOM") {
		bound = new RandomPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "LEFT") {
		bound = new BlindPolicy(this, LEFT,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "RIGHT") {
		bound = new BlindPolicy(this, RIGHT,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "LISTEN") {
		bound = new BlindPolicy(this, LISTEN,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "OPTIMAL") {
		bound = new OptimalTigerPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported scenario lower bound: " << name << endl;
		exit(1);
	}
	return bound;
}

void Tiger::PrintState(const State& state, ostream& out) const {
	const TigerState& tigerstate = static_cast<const TigerState&>(state);
	out << tigerstate.text() << endl;
}

void Tiger::PrintBelief(const Belief& belief, ostream& out) const {
}

void Tiger::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
	out << (obs == LEFT ? "LEFT" : "RIGHT") << endl;
}

void Tiger::PrintAction(ACT_TYPE action, ostream& out) const {
	if (action == LEFT) {
		out << "Open left" << endl;
	} else if (action == RIGHT) {
		out << "Open right" << endl;
	} else {
		out << "Listen" << endl;
	}
}

State* Tiger::Allocate(int state_id, double weight) const {
	TigerState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State* Tiger::Copy(const State* particle) const {
	TigerState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const TigerState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void Tiger::Free(State* particle) const {
	memory_pool_.Free(static_cast<TigerState*>(particle));
}

int Tiger::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

vector<double> Tiger::ImportanceWeight(vector<State*> particles) const {
 	double total_weight=State::Weight(particles);
	int particles_num=particles.size();

	vector<TigerState*> tiger_particles;

	vector <double> importance_weight;

	for(int i=0; i<particles_num;i++){
		tiger_particles.push_back(static_cast<TigerState*>(particles[i]));
	}

	double new_total_weight=0;
	for(int i=0; i<particles_num;i++){
		importance_weight.push_back(tiger_particles[i]->weight * sampling_weight_[tiger_particles[i]->tiger_position]);
		new_total_weight += importance_weight[i];
	}

	//normalize to total_weight
	for(int i=0; i<particles_num;i++){
		importance_weight[i]=importance_weight[i]*total_weight/new_total_weight;
	}

	return importance_weight;
 }

 vector<double> Tiger::Feature(const State& state) const {
	vector<double> feature;

	if(Globals::config.collect_data == false) return feature;
	const TigerState& tiger_state = static_cast<const TigerState&>(state);
	feature.push_back(tiger_state.tiger_position);	

	return feature;
}


} // namespace despot
