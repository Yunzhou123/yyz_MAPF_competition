#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

struct SIPPNode {
    int arrive_time;
    int arrive_dir;
    pair<int,int> safe_interval;
    int location;
    int f,g,h;
    SIPPNode* parent;
    SIPPNode(int _arrive_time,pair<int,int> _safe_interval,int _f, int _g, int _h, SIPPNode* _parent,int _location,int _arrive_dir):
            arrive_time(_arrive_time),safe_interval(_safe_interval),f(_f),g(_g),h(_h),parent(_parent),location(_location),arrive_dir(_arrive_dir) {} ;
};
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
    std::vector<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    // End kit dummy implementation

    int* priority_order;
    int* agent_path_index;
    int RHCR_w; //The time interval that we want to resolve collisions
    int RHCR_h; //planning interval
    vector<pair<int,int>>* agents_path;
    vector<pair<int,int>>* safe_intervals; // safe intervals for all vertices
    vector<pair<int,int>>* last_move_pos;
    std::vector<int> map;
    std::vector<int> index;

    void map_index_to_vec_index(int map_h, int map_w, int &vec_index);
    void vec_index_to_map_index(int &map_h, int &map_w, int vec_index);
    
    bool decide_when_to_plan(int current_timestep, int RGCR_h);
    vector<pair<int,int>> single_agent_plan_SIPP(int start, int start_direct, int end,vector<pair<int,int>>* safe_interval);
    pair<int,int> compute_current_interval(vector<pair<int,int>> current_safe_intervals,int current_time);
    vector<SIPPNode> SIPP_get_neighbor(SIPPNode* sipp_node,vector<pair<int,int>>* last_move_pos,vector<pair<int,int>>* safe_intervals,int end);
    void SIPP_update_safe_intervals(const vector<pair<int, int>>* agent_planned_path, const int current_time_step, const int& rhcr_w);
};

