#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan_yyz(int start,int start_direct, int end);
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    int* priority_order;
    int* agent_path_index;

    void map_index_to_vec_index(int map_h, int map_w, int &vec_index);

    void vec_index_to_map_index(int &map_h, int &map_w, int vec_index);
    int RHCR_w; //The time interval that we want to resolve collisions
    int RHCR_h; //planning interval
    bool decide_when_to_plan(int current_timestep, int RGCR_h);
    list<pair<int,int>>* agents_path;
    list<pair<int,int>>* safe_intervals;
    std::vector<int> map;
    std::vector<int> index;
};
