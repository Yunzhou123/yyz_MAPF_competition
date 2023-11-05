#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
struct node_interval{
    bool is_safe; // True means safe intervals,False means non-safe intervals
    int start_timestep;
    int end_timestep;
    int id; //Only for non-safe intervals
    int from_where; //Only for non-safe intervals
    node_interval(bool _is_safe,int _start_timestep,int _end_timestep,int _id, int _from_where):
            is_safe(_is_safe),start_timestep(_start_timestep),end_timestep(_end_timestep),id(_id),from_where(_from_where) {};
};
struct SIPPNode {
    int arrive_time;
    int arrive_dir;
    node_interval safe_interval;
    int location;
    int f,g,h;
    int parent;
    int current_interval_next_pos;
    int next_agent_id;
    SIPPNode(int _arrive_time,node_interval _safe_interval,int _f, int _g, int _h, int _parent,int _location,int _arrive_dir,int _current_interval_next_pos,int _agent_id):
            arrive_time(_arrive_time),safe_interval(_safe_interval),f(_f),g(_g),h(_h),parent(_parent),location(_location),arrive_dir(_arrive_dir),current_interval_next_pos(_current_interval_next_pos),next_agent_id(_agent_id) {} ;
};
struct SIPP_cmp
{
    bool operator()(SIPPNode a, SIPPNode b)
    {
        if(a.f == b.f) return a.g <= b.g;
        else return a.f > b.f;
    }
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

    int* agents_index;
    int* priority_order;
    int* agent_path_index;
    int RHCR_w; //The time interval that we want to resolve collisions
    int RHCR_h; //planning interval
    bool replan_flag; //One flag that indicates replanning when there are new tasks
    bool plan_first_time_flag;

    vector<int> map;
    vector<int> index;
    //vector<pair<int,int>>* occupy_id;  // use to record after the i-th safe interval, which agent comes. Start with -1 if the safe interval starts from 0. End with -1
    
    vector<pair<int,int>>* agents_path;
    //vector<pair<int,int>>* safe_intervals;
    vector<node_interval>* all_interval_nodes;
    //vector<pair<int,int>>* last_move_pos;

    void map_index_to_vec_index(int map_h, int map_w, int &vec_index);
    void vec_index_to_map_index(int &map_h, int &map_w, int vec_index);
    
    bool decide_when_to_plan(int current_timestep, int RGCR_h);
    vector<pair<int,int>> single_agent_plan_SIPP(int start, int start_direct, int end, vector<pair<int,int>>* safe_interval, bool* find_flag, int agent_id);
    node_interval compute_current_interval(vector<node_interval> current_safe_intervals,int current_time,int* rtn_index);
    vector<pair<int,int>> single_agent_plan_SIPP_with_constraints(int start, int start_direct, int end, vector<node_interval>* all_interval_nodes, bool* find_flag, int agent_id,std::vector<std::vector<std::vector<int>>> constraints,vector<int>* related_agents);

    void insert_safe_intervals(int location, int time,int last_pos,int agent_id);
    void SIPP_update_safe_intervals(vector<pair<int, int>> agent_planned_path, int agent_id);
    vector<int>*  generate_random_constraint();
};

