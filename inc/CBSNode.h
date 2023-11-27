#pragma once
#include "common.h"

struct CBS_node{
    int conflict_num;
    int solution_cost;
    vector<conflict> conflicts;
    vector<pair<int,int>>* replan_paths;
    vector<int>* time_list;

    vector<vector<vector<int>>>* constraints;
    vector<vector<vector<int>>>* stay_constraints;
    int agent_num;
    CBS_node (int _conflict_num,int _solution_cost,vector<conflict> _conflicts,vector<pair<int,int>>* _replan_paths,std::vector<std::vector<std::vector<int>>>* _constraints,vector<int>* _time_list,int _agent_num,std::vector<std::vector<std::vector<int>>>* _stay_constraints){
        conflict_num=_conflict_num;
        solution_cost=_solution_cost;
        conflicts=_conflicts;
        agent_num=_agent_num;
        replan_paths=new vector<pair<int,int>>[agent_num];
        time_list=new vector<int>[agent_num];
        constraints=new vector<vector<vector<int>>>[agent_num];
        stay_constraints=new vector<vector<vector<int>>>[agent_num];
        for (int i=0;i<agent_num;i++){
            replan_paths[i]=_replan_paths[i];
            constraints[i]=  _constraints[i];
            stay_constraints[i]=_stay_constraints[i];
            time_list[i]=_time_list[i];
        }
    }
};

struct CBS_cmp
{
    bool operator()(CBS_node a, CBS_node b)
    {
        if(a.solution_cost == b.solution_cost) return a.conflict_num <= b.conflict_num;
        else return a.solution_cost > b.solution_cost;
    }
};

// struct ConstraintsHasher // Hash a CT node by constraints on one agent
// {
//     int a{};
//     const CBS_node* n{};
//     ConstraintsHasher(int a, CBS_node* n) : a(a), n(n) {};

//     struct EqNode
//     {
//         bool operator()(const ConstraintsHasher& n1, const ConstraintsHasher& n2) const
//         {
//             if (n1.a != n2.a) return false;
//         }
            

//     }
// };