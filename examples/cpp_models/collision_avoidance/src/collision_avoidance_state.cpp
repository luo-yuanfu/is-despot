#include "collision_avoidance_state.h"

WorldState::WorldState() {
}

WorldState::~WorldState() {
}

void WorldState::GenRandomState() {
	aircraft_height = rand()%MAP_HEIGHT;
	obstacle_height = rand()%MAP_HEIGHT;
	distance = MAP_WIDTH-1;
}