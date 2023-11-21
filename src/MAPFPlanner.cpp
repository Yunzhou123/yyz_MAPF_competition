#include <MAPFPlanner.h>
#include <random>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
using namespace std::chrono;
int a=1+1;
/*
    Assume every agents' path start from the same time t
    the length of path may various
    pair<position, direction>
    direction => 0: Forward
                 1: Backward
                 2: upward
                 3: downward
    map size: env->rows * env->cols
*/

vector<struct conflict> MAPFPlanner::detect_conflict(vector<int>agent_id, vector<pair<int, int>>* agents_path,int start_time){
    vector<struct conflict> v;
    for(int i = 1; i < agent_id.size(); i++){
        for(int j = 0; j < agents_path[i].size(); j++){

            int loc = agents_path[i][j].first;
            int dir = agents_path[i][j].second;
            for(int k = 0; k < i; k++){
                int prev_path_len = agents_path[k].size();
                if(j < prev_path_len and j<RHCR_w-1){

                    int prev_loc = agents_path[k][j].first;
                    int prev_dir = agents_path[k][j].second;
                    if (agents_path[i].size()!=0){
                    }
                    if(prev_loc == loc){    // node conflict
                        int agent1_constraint_type=0;
                        int agent2_constraint_type=0;

                        int dest = agents_path[i][j-1].first;
                        int prev_dest = agents_path[k][j-1].first;
                        if (dest==loc){
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"stay agent: "<<agent_id[i]<<endl;
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile <<"stay agent: "<<agent_id[i]<<endl;
                                // Close the file
                                MyFile.close();
                            }
                            agent1_constraint_type=1;
                        }
                        if (prev_dest==prev_loc){
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"stay agent: "<<agent_id[k]<<endl;
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile <<"stay agent: "<<agent_id[k]<<endl;
                                // Close the file
                                MyFile.close();
                            }
                            agent2_constraint_type=1;
                        }

                        struct conflict node_conflict = {loc, -1, agent_id[i], agent_id[k], 0,start_time+j,agent1_constraint_type,agent2_constraint_type};
                        v.push_back(node_conflict);
                    }
                    else{
                        if((j < prev_path_len - 1) && (j < agents_path[i].size() - 1)){
                            int dest = agents_path[i][j+1].first;
                            int prev_dest = agents_path[k][j+1].first;
                            if(prev_dest == loc && prev_loc == dest){  //edge conflict
                                struct conflict edge_conflict = {loc, dest, agent_id[i], agent_id[k], 1,start_time+j+1,0,0};
                                v.push_back(edge_conflict);
                            }
                        }
                    }
                }
            }
        }
    }
    return v;
}
int compute_solution_cost(vector<pair<int,int>>* replan_paths, int agent_num){
    int cost=0;
    for (int i=0;i<agent_num;i++){
        cost=cost+replan_paths[i].size();
    }
    return cost;
}
void MAPFPlanner::CBS(vector<conflict> conflicts,vector<pair<int,int>>* replan_paths,vector<int> related_agents,vector<int>* time_list,bool* find_sol_flag,vector<pair<int,int>>* found_replan_paths,vector<int>* found_time_list,vector<int>* CBS_related_agents){
    cout<<"start to use CBS!"<<endl;
    std::vector<std::vector<std::vector<int>>> initial_constraints[related_agents.size()];
    std::vector<std::vector<std::vector<int>>> initial_stay_constraints[related_agents.size()];
    vector<int> new_CBS_related_agents;
    int select_neigh_num=10;

    *find_sol_flag= false;
    cout<<related_agents.size()<<endl;
    for (int i=0;i<related_agents.size();i++){
        std::vector<std::vector<std::vector<int>>> constraint(env->cols*env->rows, std::vector<std::vector<int>>(4));
        std::vector<std::vector<std::vector<int>>> stay_constraint(env->cols*env->rows, std::vector<std::vector<int>>(1));
        initial_constraints[i]=constraint;
        initial_stay_constraints[i]=stay_constraint;
        cout<<related_agents[i]<<endl;
        new_CBS_related_agents.push_back(related_agents[i]);
    }
    int conflict_num=conflicts.size();
    int sol_cost= compute_solution_cost(replan_paths,related_agents.size());
    CBS_node initial_node=CBS_node(conflict_num,sol_cost,conflicts,replan_paths,initial_constraints,time_list,related_agents.size(),initial_stay_constraints);

    priority_queue<CBS_node,vector<CBS_node>,CBS_cmp> open_list;
    open_list.push(initial_node);
    CBS_node curr_node = open_list.top();
    open_list.pop();
    while (curr_node.conflict_num!=0){
        cout<<curr_node.conflict_num<<endl;
        int random = rand() % curr_node.conflict_num;
        conflict select_conflict=curr_node.conflicts[random];
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (outfile.is_open()){
            outfile <<"collision location "<<select_conflict.location<<" time "<<select_conflict.conflict_time<<" "<<select_conflict.agent1<<" "<<select_conflict.agent2<<endl;
            outfile <<"current time "<<env->curr_timestep<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile <<"collision location "<<select_conflict.location<<" time "<<select_conflict.conflict_time<<" "<<select_conflict.agent1<<" "<<select_conflict.agent2<<endl;
            MyFile<<"current time "<<env->curr_timestep<<endl;
            // Close the file
            MyFile.close();
        }
        std::vector<std::vector<std::vector<int>>>* constraint1=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
        std::vector<std::vector<std::vector<int>>>* constraint2=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
        std::vector<std::vector<std::vector<int>>>* stay_constraint1=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
        std::vector<std::vector<std::vector<int>>>* stay_constraint2=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
        cout<<"phase 1"<<endl;
        generate_constraints(select_conflict,curr_node.constraints,constraint1,constraint2,related_agents,curr_node.stay_constraints,stay_constraint1,stay_constraint2);
        vector<pair<int,int>>* new_replan_path_1=new vector<pair<int,int>>[related_agents.size()];
        vector<pair<int,int>>* new_replan_path_2=new vector<pair<int,int>>[related_agents.size()];
        vector<int>* new_time_list_1=new vector<int>[related_agents.size()];
        vector<int>* new_time_list_2=new vector<int>[related_agents.size()];


        for (int i=0;i<related_agents.size();i++){
            new_replan_path_1[i]=replan_paths[i];
            new_replan_path_2[i]=replan_paths[i];
            new_time_list_1[i]=time_list[i];
            new_time_list_2[i]=time_list[i];
        }
        bool find_flag_case1= true;
        bool find_flag_case2= true;
        for (int n=0;n<related_agents.size();n++){
            vector<int> related_agents_1;
            vector<int> related_agents_2;
            bool current_find_flag1= false;
            bool current_find_flag2=false;
            new_replan_path_1[n]=single_agent_plan_SIPP_with_constraints(env->curr_states[related_agents[n]].location,
                                                                    env->curr_states[related_agents[n]].orientation,
                                                                    env->goal_locations[related_agents[n]].front().first,all_interval_nodes,&current_find_flag1, related_agents[n],constraint1[n],&related_agents_1,&new_time_list_1[n],
                                                                    false,stay_constraint1[n]);
            for (int c=0;c<related_agents_1.size();c++){
                if (new_CBS_related_agents.size()<select_neigh_num){
                    bool find_neigh_flag= false;
                    for (int r=0;r<new_CBS_related_agents.size();r++){
                        if (new_CBS_related_agents[r]==related_agents_1[c]){
                            find_neigh_flag= true;
                            break;
                        }
                    }
                    if (find_neigh_flag== false){
                        new_CBS_related_agents.push_back(related_agents_1[c]);
                    }
                }
            }
            new_replan_path_2[n]=single_agent_plan_SIPP_with_constraints(env->curr_states[related_agents[n]].location,
                                                                    env->curr_states[related_agents[n]].orientation,
                                                                    env->goal_locations[related_agents[n]].front().first,all_interval_nodes,&current_find_flag2, related_agents[n],constraint2[n],&related_agents_2,&new_time_list_2[n],
                                                                    false,stay_constraint2[n]);
            for (int c=0;c<related_agents_2.size();c++){
                if (new_CBS_related_agents.size()<select_neigh_num){
                    bool find_neigh_flag= false;
                    for (int r=0;r<new_CBS_related_agents.size();r++){
                        if (new_CBS_related_agents[r]==related_agents_2[c]){
                            find_neigh_flag= true;
                            break;
                        }
                    }
                    if (find_neigh_flag== false){
                        new_CBS_related_agents.push_back(related_agents_2[c]);
                    }
                }
            }
            for (int c=0;c<related_agents_1.size();c++){

            }
            if (current_find_flag1== false){
                find_flag_case1= false;
            }
            if (current_find_flag2== false){
                find_flag_case2= false;
            }
        }
        cout<<"phase 2"<<endl;
        if (find_flag_case1== true){
            vector<conflict> conflicts=detect_conflict(related_agents, new_replan_path_1, env->curr_timestep);
            int conflict_num=conflicts.size();
            int sol_cost= compute_solution_cost(new_replan_path_1,related_agents.size());
            CBS_node new_node=CBS_node(conflict_num,sol_cost,conflicts,new_replan_path_1,constraint1,new_time_list_1,related_agents.size(),stay_constraint1);
            open_list.push(new_node);
        }
        if (find_flag_case2== true){
            vector<conflict> conflicts=detect_conflict(related_agents,new_replan_path_2, env->curr_timestep);
            int conflict_num=conflicts.size();
            int sol_cost= compute_solution_cost(new_replan_path_2,related_agents.size());
            CBS_node new_node=CBS_node(conflict_num,sol_cost,conflicts,new_replan_path_2,constraint2,new_time_list_2,related_agents.size(),stay_constraint2);
            open_list.push(new_node);
        }
        if (open_list.size()==0){
            cout<<"not feasible!"<<endl;
            found_replan_paths=new vector<pair<int,int>>[related_agents.size()];
            found_time_list=new vector<int>[related_agents.size()];
            *find_sol_flag= false;
            for (int j=0;j<related_agents.size();j++){
                found_replan_paths[j]=curr_node.replan_paths[j];
                found_time_list[j]=curr_node.time_list[j];
            }
            break;
        }
        cout<<"phase 3"<<endl;
        curr_node = open_list.top();
        open_list.pop();
        if (curr_node.conflict_num==0){
            *find_sol_flag=true;
            cout<<"found solution!"<<endl;
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<"find valid solution!"<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile <<"find valid solution!"<<endl;
                // Close the file
                MyFile.close();
            }
            for (int j=0;j<related_agents.size();j++){
                std::ofstream outfile("log.txt",std::ofstream::app);
                if (outfile.is_open()){
                    outfile <<"agent "<<related_agents[j]<<endl;
                    outfile.close();
                }
                else{
                    std::ofstream MyFile("log.txt");
                    // Write to the file
                    MyFile <<"agent "<<related_agents[j]<<endl;
                    // Close the file
                    MyFile.close();
                }
                for (int p=0;p<curr_node.replan_paths[j].size();p++){
                    std::ofstream outfile("log.txt",std::ofstream::app);
                    if (outfile.is_open()){
                        outfile <<"replan position "<<curr_node.replan_paths[j][p].first <<" replan direction"<<curr_node.replan_paths[j][p].second<<endl;
                        outfile.close();
                    }
                    else{
                        std::ofstream MyFile("log.txt");
                        // Write to the file
                        MyFile <<"replan position "<<curr_node.replan_paths[j][p].first <<" replan direction"<<curr_node.replan_paths[j][p].second<<endl;
                        // Close the file
                        MyFile.close();
                    }
                }
                found_replan_paths[j]=curr_node.replan_paths[j];
                found_time_list[j]=curr_node.time_list[j];

            }
            break;
        }
    }
    *CBS_related_agents=new_CBS_related_agents;
}
void MAPFPlanner::generate_constraints(conflict curr_conflict,std::vector<std::vector<std::vector<int>>>* old_constraints,std::vector<std::vector<std::vector<int>>>* constaint_1,std::vector<std::vector<std::vector<int>>>* constaint_2,vector<int> related_agents,std::vector<std::vector<std::vector<int>>>* old_stay_constraints,std::vector<std::vector<std::vector<int>>>* stay_constaint_1,std::vector<std::vector<std::vector<int>>>* stay_constaint_2){
    std::vector<std::vector<std::vector<int>>>* new_constaint_1=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
    std::vector<std::vector<std::vector<int>>>* new_constaint_2=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
    std::vector<std::vector<std::vector<int>>>* new_stay_constaint_1=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
    std::vector<std::vector<std::vector<int>>>* new_stay_constaint_2=new std::vector<std::vector<std::vector<int>>>[related_agents.size()];
    int index_agent1=0;
    int index_agent2=0;
    for (int i=0;i<related_agents.size();i++){
        new_constaint_1[i]=old_constraints[i];
        new_constaint_2[i]=old_constraints[i];
        new_stay_constaint_1[i]=old_stay_constraints[i];
        new_stay_constaint_2[i]=old_stay_constraints[i];
    }
    for (int i=0;i<related_agents.size();i++){
        if (related_agents[i]==curr_conflict.agent1){
            index_agent1=i;
        }
        if (related_agents[i]==curr_conflict.agent2){
            index_agent2=i;
        }
    }
    if  (curr_conflict.type==1){
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (outfile.is_open()){
            outfile <<"index 1 "<<index_agent1<<" index 2 "<<index_agent2<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile <<"index 1 "<<index_agent1<<" index 2 "<<index_agent2<<endl;
            // Close the file
            MyFile.close();
        }
        if (curr_conflict.dest-curr_conflict.location==1){
            new_constaint_1[index_agent1][curr_conflict.location][0].push_back(curr_conflict.conflict_time-1);
            new_constaint_2[index_agent2][curr_conflict.dest][2].push_back(curr_conflict.conflict_time-1);
        }
        else if (curr_conflict.dest-curr_conflict.location==-1){
            new_constaint_1[index_agent1][curr_conflict.location][2].push_back(curr_conflict.conflict_time-1);
            new_constaint_2[index_agent2][curr_conflict.dest][0].push_back(curr_conflict.conflict_time-1);
        }
        else if (curr_conflict.dest-curr_conflict.location==env->cols){
            new_constaint_1[index_agent1][curr_conflict.location][1].push_back(curr_conflict.conflict_time-1);
            new_constaint_2[index_agent2][curr_conflict.dest][3].push_back(curr_conflict.conflict_time-1);
        }
        else if (curr_conflict.dest-curr_conflict.location==-env->cols){
            new_constaint_1[index_agent1][curr_conflict.location][3].push_back(curr_conflict.conflict_time-1);
            new_constaint_2[index_agent2][curr_conflict.dest][1].push_back(curr_conflict.conflict_time-1);

        }
    }
    else {
        if (curr_conflict.agent1_constraint_type==1){
            new_stay_constaint_1[index_agent1][curr_conflict.location][0].push_back(curr_conflict.conflict_time);
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<"some robot just rotate!"<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile <<"some robot just rotate!"<<endl;
                // Close the file
                MyFile.close();
            }
        }
        if (curr_conflict.agent2_constraint_type==1){
            new_stay_constaint_2[index_agent2][curr_conflict.location][0].push_back(curr_conflict.conflict_time);
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<"some robot just rotate!"<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile <<"some robot just rotate!"<<endl;
                // Close the file
                MyFile.close();
            }
        }
        int candidates[4] = { curr_conflict.location - 1,curr_conflict.location - env->cols, curr_conflict.location + 1, curr_conflict.location + env->cols};
        for (int i=0;i<4;i++){
            if (candidates[i]>=0 and candidates[i]<env->cols*env->cols){
                if (curr_conflict.agent1_constraint_type==0){
                    new_constaint_1[index_agent1][candidates[i]][i].push_back(curr_conflict.conflict_time-1);
                }
                if (curr_conflict.agent2_constraint_type==0){
                    new_constaint_2[index_agent2][candidates[i]][i].push_back(curr_conflict.conflict_time-1);
                }


            }
        }
    }
    for (int i=0;i<related_agents.size();i++){
        constaint_1[i]=new_constaint_1[i];
        constaint_2[i]=new_constaint_2[i];
        stay_constaint_1[i]=new_stay_constaint_1[i];
        stay_constaint_2[i]=new_stay_constaint_2[i];
    }
}

/**
 * @brief helper function
 * @param array The array to sort
 * @return The indices of the sorted array in ascending order
*/
template<typename T> std::vector<int> argsort(const std::vector<T>& array)
{
    const int array_len(array.size());
    std::vector<int> array_index(array_len, 0);
    for (int i = 0; i < array_len; ++i)
        array_index[i] = i;

    std::sort(array_index.begin(), array_index.end(),
              [&array](int pos1, int pos2) {return (array[pos1] < array[pos2]); });

    return array_index;
}

/**
 * @brief extract the corresponding safe interval of the current time from the safe interval list
 * @param current_safe_intervals The safe intervals of the current vertex
 * @param current_time The current time
 * @param rtn_index
 * @return The corresponding safe interval
*/
node_interval MAPFPlanner::compute_current_interval(vector<node_interval> current_safe_intervals, int current_time, int* rtn_index){

    int intervals_num=current_safe_intervals.size();
    int low_index=0;
    int high_index=intervals_num-1;
    int check_index=int((low_index+high_index)/2);
    bool flag= false;
    node_interval* rtn_interval;


    /* 
        locate the current_time for the specifc interval in current_sata_intervals using binary search
        The interval should be treated as [start_timestamp, end_timestamp] or [start_timestamp, end_timestamp)?
    
    */
    while (!flag){
        if ((current_time<=current_safe_intervals[check_index].end_timestep) and (current_time>=current_safe_intervals[check_index].start_timestep)) {
            rtn_interval=&current_safe_intervals[check_index];
            flag= true;
        } else {
            if (high_index == low_index+1) {

                if (current_time > current_safe_intervals[check_index].end_timestep) {
                    check_index=high_index;
                    low_index=high_index;
                } else {
                    check_index = low_index;
                    high_index = low_index;
                }

            }
            else {

                if (current_time > current_safe_intervals[check_index].end_timestep) {
                    low_index = check_index;
                } else{
                    high_index = check_index;
                }

                check_index = int((low_index+high_index)/2);
            }
        }
    }
    *rtn_index = check_index;
    return *rtn_interval;
}

/// initialize the planner
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    map=env->map;
    //Initialize a priority order
    int new_order[env->num_of_agents];
    int new_agent_path_index[env->num_of_agents];
    for (int i = 0; i < env->num_of_agents; ++i) {
        new_order[i] = i;
        new_agent_path_index[i] = 0;
    }
    std::shuffle(&new_order[0],&new_order[env->num_of_agents], std::mt19937(std::random_device()()));

    priority_order = new_order;
    agent_path_index = new_agent_path_index;
    RHCR_w = 18;
    RHCR_h = 15;
    replan_flag = false;
    plan_first_time_flag= true;
    agents_path = new vector<pair<int,int>>[env->num_of_agents];
    agents_time_list=new vector<int>[env->num_of_agents];
    agents_index = new int [env->num_of_agents];
    agent_prev_pos=new int[env->num_of_agents];
    //this->occupy_id = new vector<pair<int,int>>[env->rows*env->cols];

    //safe_intervals = new vector<pair<int,int>>[env->rows*env->cols];
    //last_move_pos = new vector<pair<int,int>>[env->rows*env->cols];
    all_interval_nodes=new vector<node_interval>[env->rows*env->cols];

    for (int i = 0; i < env->rows*env->cols; ++i) {
        pair<int, int> initial_interval;
        pair<int, int> initial_lastmove;
        initial_interval.first = 0;
        initial_interval.second = 10000;
        initial_lastmove.first = -1;
        initial_lastmove.second = -1;
        node_interval initial_node=node_interval(true,0,10000,-1,-1);
        all_interval_nodes[i].push_back(initial_node);

        //safe_intervals[i].push_back(initial_interval);
        //last_move_pos[i].push_back(initial_lastmove);
        //this->occupy_id[i].push_back(make_pair(-1,-1));
    }
    
    cout << env->map.size() << endl;
    //int map_height=env->rows;
    //int map_width=env->cols;
    //int map_arr[map_height][map_width]; //0 means traversable, 1 means not traversable
    //int vectorIndex = 0;
    //for (int i = 0; i < map_height; ++i) {
        //for (int j = 0; j < map_width; ++j) {
            //map_arr[i][j] = env->map[vectorIndex++];
        //}
    //}
    cout << "planner initialize done" << endl;

}

/// convert 2D index to 1D index
void MAPFPlanner::map_index_to_vec_index(int map_h, int map_w,int& vec_index)
{
    vec_index = map_h*env->cols+map_w;
}

/// convert 1D index to 2D index
void MAPFPlanner::vec_index_to_map_index(int& map_h, int& map_w,int vec_index)
{
    map_h = int(vec_index/env->cols); // will this always round down? 
    map_w = vec_index-map_h*env->cols;
}

/// RHCR Algorithm
bool MAPFPlanner::decide_when_to_plan(int current_timestep,int RHCR_h){
    bool flag = false;
    float residue = current_timestep-RHCR_h*int(current_timestep/RHCR_h);

    if (residue == 0) {
        flag = true;
    }

    return flag;
}

void MAPFPlanner::insert_safe_intervals(int location, int time){

    vector<node_interval> current_node_intervals=all_interval_nodes[location];
    int intervals_num = current_node_intervals.size();
    int low_index = 0;
    int high_index = intervals_num-1;
    int check_index = int((low_index+high_index)/2);
    int index_num = -1;
    bool flag = false;
    while (flag == false){
        if ((time >= current_node_intervals[check_index].start_timestep) and (time <= current_node_intervals[check_index].end_timestep)) {
            index_num = check_index;
            flag= true;
            continue;
        }
        else{
            if (high_index == low_index+1){
                if (time > current_node_intervals[check_index].end_timestep) {
                    check_index = high_index;
                    low_index = high_index;
                }
                else{
                    check_index = low_index;
                    high_index = low_index;
                }
            }
            else {
                if (time > current_node_intervals[check_index].end_timestep) {
                    low_index = check_index;
                }
                else {
                    high_index=check_index;
                }
                check_index = int((low_index+high_index)/2);
            }
        }
    }
    if (current_node_intervals[index_num].is_safe== true){ //The interval is a safe interval, no actions are needed to be performed
        int delete_count=0;
        for (int k=index_num+1;k<current_node_intervals.size();k++){
            if (current_node_intervals[k].is_safe== false){
                break;
            }
            else{
                current_node_intervals[index_num].end_timestep=current_node_intervals[k].end_timestep;
                delete_count=delete_count+1;
            }
        }
        for (int k=0;k<delete_count;k++){
            current_node_intervals.erase(current_node_intervals.begin()+index_num+1);
        }
        delete_count=0;
        for (int k=0;k<index_num;k++){
            check_index=index_num-1-k;
            if (current_node_intervals[check_index].is_safe== false){
                break;
            }
            else{
                current_node_intervals[index_num].start_timestep=current_node_intervals[k].start_timestep;
                delete_count=delete_count+1;
            }
        }
        for (int k=0;k<delete_count;k++){
            current_node_intervals.erase(current_node_intervals.begin()+index_num-delete_count);
        }
    }
    else if (time==current_node_intervals[index_num].end_timestep){
        if (current_node_intervals[index_num+1].is_safe== true) {
            
            // what if the current_node_intervals[index_num].start_timestep == current_node_intervals[index_num].end_timestep
            current_node_intervals[index_num].end_timestep=current_node_intervals[index_num].end_timestep-1;

            current_node_intervals[index_num+1].start_timestep=current_node_intervals[index_num+1].start_timestep-1;
            if (current_node_intervals[index_num].start_timestep>current_node_intervals[index_num].end_timestep){
                if (index_num>=1){
                    if (current_node_intervals[index_num-1].is_safe== true){
                        current_node_intervals[index_num-1].end_timestep=current_node_intervals[index_num+1].end_timestep;
                        current_node_intervals.erase(current_node_intervals.begin()+index_num);
                    }
                }
                current_node_intervals.erase(current_node_intervals.begin()+index_num);
            }
        }
        else {
            // same problem as above
            current_node_intervals[index_num].end_timestep=current_node_intervals[index_num].end_timestep-1;
            node_interval new_node=node_interval(true,time,time,-1,-1);
            current_node_intervals.insert(current_node_intervals.begin()+index_num+1,new_node);
            if (current_node_intervals[index_num].start_timestep>current_node_intervals[index_num].end_timestep){
                current_node_intervals.erase(current_node_intervals.begin()+index_num);
                if (index_num>=1){
                    if (current_node_intervals[index_num-1].is_safe== true){
                        current_node_intervals[index_num-1].end_timestep=current_node_intervals[index_num-1].end_timestep+1;
                        current_node_intervals.erase(current_node_intervals.begin()+index_num);
                    }
                }
            }
        }

    }
    else if (time==current_node_intervals[index_num].start_timestep){
        if (index_num==0){
            current_node_intervals[index_num].start_timestep=current_node_intervals[index_num].start_timestep+1;
            node_interval new_node=node_interval(true,time,time,-1,-1);
            current_node_intervals.insert(current_node_intervals.begin()+index_num,new_node);
        }
        else{
            if (current_node_intervals[index_num-1].is_safe== true){
                current_node_intervals[index_num-1].end_timestep=current_node_intervals[index_num-1].end_timestep+1;
                current_node_intervals[index_num].start_timestep=current_node_intervals[index_num].start_timestep+1;
                if (current_node_intervals[index_num].start_timestep>current_node_intervals[index_num].end_timestep){
                    current_node_intervals.erase(current_node_intervals.begin()+index_num);
                }
            }
            else{
                current_node_intervals[index_num].start_timestep=current_node_intervals[index_num].start_timestep+1;
                node_interval new_node=node_interval(true,time,time,-1,-1);
                current_node_intervals.insert(current_node_intervals.begin()+index_num,new_node);
            }
        }
    }
    else { //Seems impossible to happen
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (outfile.is_open()){
            outfile <<"something wrong happens!"<<endl;
            outfile <<location<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile <<"something wrong happens!"<<endl;
            MyFile<<location<<endl;
            // Close the file
            MyFile.close();
        }
        cout<<"something wrong happens!"<<endl;
        int id_copy=current_node_intervals[index_num].id;
        int from_where_copy=current_node_intervals[index_num].from_where;
        int end_time_copy=current_node_intervals[index_num].end_timestep;
        current_node_intervals[index_num].end_timestep=time-1;
        /*
            should also update from_where in the current_node_intervals[index_num]
        */
        node_interval insert_node_1=node_interval(true,time,time,-1,-1);
        current_node_intervals.insert(current_node_intervals.begin()+index_num+1,insert_node_1);
        node_interval insert_node_2=node_interval(false,time+1,end_time_copy,id_copy,from_where_copy);
        current_node_intervals.insert(current_node_intervals.begin()+index_num+2,insert_node_2);
    }
    all_interval_nodes[location]=current_node_intervals;
}
/// plan and refine the paths, and generate actions for each agent
void MAPFPlanner::plan(int time_limit, vector<Action> & actions)
{

    auto program_start_time = high_resolution_clock::now();
    int current_time = env->curr_timestep;
    bool replan_all_flag = decide_when_to_plan(current_time,RHCR_h);
    if (replan_all_flag){
        cout<<"Time to replan all agents ! "<<endl;
    }
    else{
        cout<<"Not time to replan all agets ! "<<endl;
    }
    actions = vector<Action>(env->curr_states.size(), Action::W);
    cout<<"replan flag "<<replan_flag<<endl;
    std::vector<std::vector<std::vector<int>>> constraints(env->cols*env->rows, std::vector<std::vector<int>>(4));
    std::vector<std::vector<std::vector<int>>> stay_constraints(env->cols*env->rows, std::vector<std::vector<int>>(1));

    //generate a random constraint, just for testing
    int constraint_num=0;
    for (int i=0;i<constraint_num;i++) {
        int map_size=env->cols*env->rows;
        int direction_num=4;
        int time_limit=500;
        int sample_location=rand()%map_size;
        int sample_direction=rand()%direction_num;
        int sample_time=rand() %time_limit;
        constraints[sample_location][sample_direction].push_back(sample_time);
    }
    if (env->curr_timestep>2000) {
        // leave empty for testing
    } else if (replan_all_flag==true) {
        /*
            should delete the old allocated space (otherwise will leave the job for the garbage collector)
        */
        delete []all_interval_nodes;
        int already_planned_agents[env->num_of_agents];
        for (int i=0;i<env->num_of_agents;i++){
            already_planned_agents[i]=0;
        }

        all_interval_nodes=new vector<node_interval>[env->rows*env->cols];
        //safe_intervals = new vector<pair<int,int>>[env->rows*env->cols];
        //last_move_pos = new vector<pair<int,int>>[env->rows*env->cols];
        //delete []occupy_id;
        //occupy_id = new vector<pair<int,int>>[env->rows*env->cols];

        for (int i = 0; i < env->rows*env->cols; ++i) {
            // initialize safe_intervals
            node_interval initial_node=node_interval(true,0,10000,-1,-1);
            all_interval_nodes[i].push_back(initial_node);
            //safe_intervals[i].push_back(make_pair(1, 10000));
            // initialize last_move_pos
            //last_move_pos[i].push_back(make_pair(-1, -1));
            //this->occupy_id[i].push_back(make_pair(-1,-1)); // initial: occupy_id[i] = <-1, -1>
        }

        vector<int> vec_data;

        for (int i = 0; i < env->num_of_agents; ++i) {
            vec_data.push_back(priority_order[i]);
            index = argsort(vec_data);    //why don't we sort the vec_data outside the for loop?
        }

        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent = index[i];
            if (already_planned_agents[current_agent]==1){
                continue;
            }
            int current_position = env->curr_states[current_agent].location;
            int current_map_h = -1;
            int current_map_w = -1;
            cout << "current agent: " << current_agent << endl;
            cout << "Begin the planning of an agent" << endl;

            vec_index_to_map_index(current_map_h, current_map_w, current_position);
            vector<pair<int, int>> path;
            agent_path_index[current_agent] = agent_path_index[current_agent] + 1;
            if (plan_first_time_flag== true){
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep);
            }
            else{
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep);
            }

            if (env->goal_locations[current_agent].empty()) {
                agents_path[current_agent].push_back(
                        {env->curr_states[current_agent].location, env->curr_states[current_agent].orientation});
                agents_time_list[current_agent].push_back(env->curr_timestep+i);
            } else {
                cout << "Begin to use SIPP" << endl;
                bool find_flag = false;
                vector<int> related_agents;
                vector <int> curent_time_list;
                path = single_agent_plan_SIPP_with_constraints(env->curr_states[current_agent].location,
                                              env->curr_states[current_agent].orientation,
                                              env->goal_locations[current_agent].front().first, all_interval_nodes,
                                              &find_flag, current_agent,constraints,&related_agents,&curent_time_list,true,stay_constraints);

                cout<<"Find flag is "<<find_flag<<endl;
                std::ofstream outfile("log.txt",std::ofstream::app);
                if (outfile.is_open()){
                    outfile <<"findall flag "<<find_flag<<endl;
                    outfile.close();
                }
                else{
                    std::ofstream MyFile("log.txt");
                    // Write to the file
                    MyFile <<"findall flag "<<find_flag<<endl;
                    // Close the file
                    MyFile.close();
                }
                if (find_flag== true) {
                    agents_time_list[current_agent]=curent_time_list;
                    agents_index[current_agent] = 1;
                    agents_path[current_agent] = path;
                    cout << "begin to generate path!" << endl;
                    cout<<current_agent<<endl;
                    cout << "path length: " << agents_path[current_agent].size() << endl;
                    already_planned_agents[current_agent]=1;
                    if (!find_flag) {
                        actions[current_agent] = Action::W;
                        continue;
                    } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {
                        actions[current_agent] = Action::FW; //forward action
                        if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                            replan_flag = true;
                            cout << "This robot will reach its goal at next time!" << endl;
                            agents_index[current_agent] = 0;
                        }
                    } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                        int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;
                        if (incr == 1 || incr == -3) {
                            actions[current_agent] = Action::CR; //C--counter clockwise rotate
                        } else if (incr == -1 || incr == 3) {
                            actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                        }
                        else if (incr==0){
                            actions[current_agent] = Action::W;
                        }
                    }
                    else{
                        actions[current_agent] = Action::W;
                    }
                }
                else {
                    bool CBS_success_flag= false;
                    bool CBS_terminate_flag= false;
                    int CBS_replan_count=0;
                    related_agents.insert(related_agents.begin(),current_agent);
                    while (CBS_terminate_flag== false){
                        insert_safe_intervals_from_path(related_agents,agents_path,agents_time_list);
                        //related_agents.push_back(current_agent);
                        vector<pair<int,int>>* replan_paths;
                        replan_paths=new vector<pair<int,int>>[related_agents.size()];
                        vector<int> *replan_time_lists;
                        replan_time_lists=new vector<int>[related_agents.size()];
                        int solution_cost=0;
                        bool replan_find_flag= true;
                        cout<<"debug"<<endl;
                        cout<<related_agents.size()<<endl;
                        cout<<replan_paths->size()<<endl;

                        for (int n=0;n<related_agents.size();n++){
                            vector<int> new_related_agents;
                            vector <int> curent_time_list;
                            bool current_find_flag= false;
                            //if (already_planned_agents[related_agents[n]]==1){
                              //  insert_safe_intervals_from_one_path(related_agents[n],agents_path[related_agents[n]],agents_time_list[related_agents[n]]);
                            //}
                            replan_paths[n]=single_agent_plan_SIPP_with_constraints(env->curr_states[related_agents[n]].location,
                                                                                    env->curr_states[related_agents[n]].orientation,
                                                                                    env->goal_locations[related_agents[n]].front().first,all_interval_nodes,&current_find_flag, related_agents[n],constraints,&new_related_agents,&curent_time_list,
                                                                                    false,stay_constraints);


                            if (current_find_flag== false){
                                replan_find_flag=false;
                            }
                            replan_time_lists[n]=curent_time_list;
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"agent "<<related_agents[n]<<endl;
                                outfile <<env->curr_timestep<<endl;
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile <<"agent "<<related_agents[n]<<endl;
                                MyFile<<env->curr_timestep<<endl;
                                // Close the file
                                MyFile.close();
                            }
                            for (int z=0;z<curent_time_list.size();z++){
                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<replan_paths[n][z].first<<" ," <<replan_paths[n][z].second<<endl;
                                    outfile <<curent_time_list[z]<<endl;
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    MyFile <<replan_paths[n][z].first<<" ," <<replan_paths[n][z].second<<endl;
                                    MyFile <<curent_time_list[z]<<endl;
                                    // Close the file
                                    MyFile.close();
                                }
                            }
                            solution_cost=solution_cost+replan_paths[n].size();
                        }
                        vector<conflict> conflicts=detect_conflict(related_agents, replan_paths, env->curr_timestep);



                        cout<<"The number of conflicts is "<<conflicts.size()<<endl;
                        std::ofstream outfile("log.txt",std::ofstream::app);
                        if (outfile.is_open()){
                            outfile <<"The number of conflicts is "<<conflicts.size()<<endl;
                            outfile.close();
                        }
                        else {
                            std::ofstream MyFile("log.txt");
                            // Write to the file
                            MyFile << "The number of conflicts is " << conflicts.size() << endl;
                            // Close the file
                            MyFile.close();
                        }
                        if  (conflicts.size()==0 and replan_find_flag== true){
                            agents_index[current_agent] = 1;
                            find_flag=true;
                            CBS_success_flag= true;
                            CBS_terminate_flag= true;
                            for (int n=0;n<related_agents.size();n++){
                                agents_path[related_agents[n]] = replan_paths[n];
                                SIPP_update_safe_intervals(replan_paths[n],related_agents[n]);
                                agents_time_list[related_agents[n]]=replan_time_lists[n];
                            }
                        }
                        else{
                            bool find_sol_flag=false;
                            vector<pair<int,int>> found_replan_paths[related_agents.size()];
                            vector<int> found_replan_time_lists[related_agents.size()];
                            vector<int> CBS_related_agents;
                            CBS(conflicts,replan_paths,related_agents,agents_time_list,&find_sol_flag,found_replan_paths,found_replan_time_lists,&CBS_related_agents);
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"The neighborhood agents: "<<endl;
                                for (int z=0;z<CBS_related_agents.size();z++){
                                    outfile <<CBS_related_agents[z]<<endl;
                                }
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile<<"The neighborhood agents: "<<endl;
                                for (int z=0;z<CBS_related_agents.size();z++){
                                    MyFile <<CBS_related_agents[z]<<endl;
                                }
                                // Close the file
                                MyFile.close();
                            }
                            if (find_sol_flag== true){
                                CBS_terminate_flag= true;
                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<"update path! "<<endl;
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    MyFile<<"update path!"<<endl;
                                    // Close the file
                                    MyFile.close();
                                }
                                find_flag=true;
                                CBS_success_flag= true;
                                for (int n=0;n<related_agents.size();n++){
                                    already_planned_agents[related_agents[n]]=1;
                                    agents_index[related_agents[n]] = 1;
                                    agents_path[related_agents[n]] = found_replan_paths[n];
                                    SIPP_update_safe_intervals(found_replan_paths[n],related_agents[n]);
                                    agents_time_list[related_agents[n]]=found_replan_time_lists[n];
                                }
                            }
                            else{
                                related_agents=CBS_related_agents;
                                //insert_safe_intervals_from_path(related_agents,agents_path,agents_time_list);

                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<"cannot find path!"<<endl;
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    outfile <<"cannot find path!"<<endl;
                                    // Close the file
                                    MyFile.close();
                                }

                                agents_index[current_agent] = 0;
                            }
                            if (CBS_replan_count>3){
                                CBS_terminate_flag= true;
                            }
                        }
                        }
                    if (CBS_success_flag== false){
                        cout << "begin to generate path!" << endl;
                        cout<<current_agent<<endl;
                        cout << "path length: " << agents_path[current_agent].size() << endl;
                        if (!find_flag) {
                            actions[current_agent] = Action::W;
                            continue;
                        } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {
                            actions[current_agent] = Action::FW; //forward action
                            if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                                replan_flag = true;
                                cout << "This robot will reach its goal at next time!" << endl;
                                agents_index[current_agent] = 0;
                            }
                        } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                            int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;
                            if (incr == 1 || incr == -3) {
                                actions[current_agent] = Action::CR; //C--counter clockwise rotate
                            } else if (incr == -1 || incr == 3) {
                                actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                            }
                            else if (incr==0){
                                actions[current_agent] = Action::W;
                            }
                        }
                        else{
                            actions[current_agent] = Action::W;
                        }
                    }
                    else{
                        for (int r=0;r<related_agents.size();r++){
                            cout << "begin to generate path!" << endl;
                            current_agent=related_agents[r];
                            cout << "path length: " << agents_path[current_agent].size() << endl;
                            if (!find_flag) {
                                actions[current_agent] = Action::W;
                                continue;
                            } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {
                                actions[current_agent] = Action::FW; //forward action
                                if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                                    replan_flag = true;
                                    cout << "This robot will reach its goal at next time!" << endl;
                                    agents_index[current_agent] = 0;
                                }
                            } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                                int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;
                                if (incr == 1 || incr == -3) {
                                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                                } else if (incr == -1 || incr == 3) {
                                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                                }
                                else if (incr==0){
                                    actions[current_agent] = Action::W;
                                }
                            }
                            else{
                                actions[current_agent] = Action::W;
                            }
                            string action="";
                            if (actions[current_agent]==Action::W){
                                action="wait";
                            }
                            else if (actions[current_agent]==Action::CCR){
                                action="CCR";
                            }
                            else if (actions[current_agent]==Action::CR){
                                action="CR";
                            }
                            else if (actions[current_agent]==Action::FW){
                                action="Forward";
                            }
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"update path of agent "<<related_agents[r]<<endl;
                                outfile<<agents_path[current_agent][0].first<<" "<<agents_path[current_agent][1].first<<endl;
                                outfile<<action<<endl;
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile<<"update path of agent "<<related_agents[r]<<endl;
                                MyFile<<agents_path[current_agent][0].first<<" "<<agents_path[current_agent][1].first<<endl;
                                MyFile<<action<<endl;
                                // Close the file
                                MyFile.close();
                            }
                        }
                    }

                }


            }
        }
        if (plan_first_time_flag== true){
            plan_first_time_flag= false;
        }

    }
    else if (replan_flag) {
        int already_planned_agents[env->num_of_agents];
        for (int i=0;i<env->num_of_agents;i++){
            already_planned_agents[i]=0;
        }
        replan_flag = false;

        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent = index[i];
            int current_timestep = agents_index[current_agent];

            cout<<"current agent: "<<current_agent<<endl;
            cout<<"step: "<<current_timestep<<endl;
            cout<<"position on the planned path: "<<agents_path[current_agent][current_timestep].first<<endl;
            cout<<"actual position on the map: "<<env->curr_states[current_agent].location<<endl;
            cout<<"direction on the planned path: "<<agents_path[current_agent][current_timestep].second<<endl;
            cout<<"actual direction on the map: "<<env->curr_states[current_agent].orientation<<endl;
            if (current_timestep>=agents_path[current_agent].size()-1 or current_timestep==0){
                actions[current_agent] = Action::W;
                continue;
            }
            if (agents_path[current_agent][current_timestep].first != agents_path[current_agent][current_timestep+1].first) {
                actions[current_agent] = Action::FW; //forward action
                if (agents_path[current_agent][current_timestep+1].first == env->goal_locations[current_agent].front().first) {
                    replan_flag = true;
                    cout<<"This robot will reach its goal at next time!"<<endl;
                    agents_index[current_agent] = -1;
                    continue;
                }
            } else if (agents_path[current_agent][current_timestep+1].second != agents_path[current_agent][current_timestep].second) {
                int incr = agents_path[current_agent][current_timestep+1].second - agents_path[current_agent][current_timestep].second;
                if (incr == 1 || incr == -3) {
                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                } else if (incr == -1 || incr == 3) {
                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
                else if (incr==0){
                    actions[current_agent]=Action::W;
                }
            }
            agents_index[current_agent] = agents_index[current_agent]+1;
        }
        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent=index[i];
            if (already_planned_agents[current_agent]==1){
                continue;
            }
            if (agents_index[current_agent]==0) {
                cout<<"insert intervals"<<endl;
                for (int p=0;p<all_interval_nodes[env->curr_states[current_agent].location].size();p++){
                    cout<<all_interval_nodes[env->curr_states[current_agent].location][p].start_timestep<<","<<all_interval_nodes[env->curr_states[current_agent].location][p].end_timestep<<endl;

                }
                for (int p=0;p<all_interval_nodes[env->curr_states[current_agent].location].size();p++){
                    cout<<all_interval_nodes[env->curr_states[current_agent].location][p].from_where<<endl;

                }
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep);
                cout<<"replanning!"<<endl;

                int current_position=env->curr_states[current_agent].location;
                int goal_location=env->goal_locations[current_agent].front().first;
                int current_map_h=-1;
                int current_map_w=-1;
                cout<<current_agent<<endl;
                cout<<"Begin the replanning of an agent"<<endl;
                //cout<<env->curr_states[current_agent].location<<endl;
                //cout<<env->curr_states[current_agent].orientation<<endl;
                vec_index_to_map_index(current_map_h,current_map_w,current_position);
                //cout<<env->curr_states[current_agent]<<endl;
                vector<pair<int,int>> path;
                agent_path_index[current_agent]=agent_path_index[current_agent]+1;
                if (env->goal_locations[current_agent].empty())
                {
                    //path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
                    agents_path[current_agent].push_back({env->curr_states[current_agent].location, env->curr_states[current_agent].orientation});
                }
                else
                {

                    cout<<"The agent number "<<current_agent<<endl;
                    //cout<<"Begin to use A star"<<endl;
                    //path = single_agent_plan(env->curr_states[current_agent].location,
                    //env->curr_states[current_agent].orientation,
                    //env->goal_locations[current_agent].front().first);
                    cout<<"Begin to use SIPP"<<endl;
                    bool find_flag = false;
                    vector<int> related_agents;
                    vector <int> curent_time_list;
                    path = single_agent_plan_SIPP_with_constraints(env->curr_states[current_agent].location,
                                                env->curr_states[current_agent].orientation,
                                                env->goal_locations[current_agent].front().first,all_interval_nodes,&find_flag, current_agent,constraints,&related_agents,&curent_time_list,
                                                                   true,stay_constraints);
                    cout<<"Find flag is "<<find_flag<<endl;
                    std::ofstream outfile("log.txt",std::ofstream::app);
                    if (outfile.is_open()){
                        outfile <<"Replanfind flag "<<find_flag<<endl;
                        outfile.close();
                    }
                    else{
                        std::ofstream MyFile("log.txt");
                        // Write to the file
                        MyFile <<"Replanfind flag "<<find_flag<<endl;
                        // Close the file
                        MyFile.close();
                    }
                    if (find_flag==true) {
                        already_planned_agents[current_agent]=1;
                        agents_path[current_agent] = path;
                        agents_time_list[current_agent]=curent_time_list;
                        agents_index[current_agent] = 1;
                        cout<<"begin to generate path!"<<endl;
                        cout<<agents_path[current_agent].size()<<endl;
                        if (!find_flag) {
                            actions[current_agent] = Action::W;
                            continue;
                        } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {

                            actions[current_agent] = Action::FW; //forward action

                            if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                                replan_flag = true;
                                cout<<"This robot will reach its goal at next time!"<<endl;
                                agents_index[current_agent] = 0;
                            }

                        } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                            int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;

                            if (incr == 1 || incr == -3) {

                                actions[current_agent] = Action::CR; //C--counter clockwise rotate

                            } else if (incr == -1 || incr == 3) {

                                actions[current_agent] = Action::CCR; //CCR--clockwise rotate

                            }
                            else if (incr==0){
                                actions[current_agent] = Action::W;
                            }
                        }
                        else{
                            actions[current_agent] = Action::W;
                        }
                    } else {
                        bool CBS_replan_flag= false;
                        related_agents.insert(related_agents.begin(),current_agent);
                        bool CBS_terminate_flag= false;
                        int CBS_replan_count=0;
                        while (CBS_terminate_flag== false){
                            insert_safe_intervals_from_path(related_agents,agents_path,agents_time_list);
                            vector<pair<int,int>>* replan_paths;
                            replan_paths=new vector<pair<int,int>>[related_agents.size()];
                            vector<int> *replan_time_lists;
                            replan_time_lists=new vector<int>[related_agents.size()];
                            int solution_cost=0;
                            bool replan_find_flag= true;
                            cout<<"debug"<<endl;
                            cout<<related_agents.size()<<endl;
                            for (int n=0;n<related_agents.size();n++){
                                vector<int> new_related_agents;
                                vector <int> curent_time_list;
                                bool current_find_flag= false;
                                //if (already_planned_agents[related_agents[n]]==1){
                                  //  insert_safe_intervals_from_one_path(related_agents[n],agents_path[related_agents[n]],agents_time_list[related_agents[n]]);
                                //}
                                replan_paths[n]=single_agent_plan_SIPP_with_constraints(env->curr_states[related_agents[n]].location,
                                                                                        env->curr_states[related_agents[n]].orientation,
                                                                                        env->goal_locations[related_agents[n]].front().first,all_interval_nodes,&current_find_flag, related_agents[n],constraints,&new_related_agents,&curent_time_list,
                                                                                        false,stay_constraints);
                                if (current_find_flag== false){
                                    replan_find_flag=false;
                                }
                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<"agent "<<related_agents[n]<<endl;
                                    outfile <<env->curr_timestep<<endl;
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    MyFile <<"agent "<<related_agents[n]<<endl;
                                    MyFile<< env->curr_timestep<<endl;
                                    // Close the file
                                    MyFile.close();
                                }
                                for (int z=0;z<curent_time_list.size();z++){
                                    std::ofstream outfile("log.txt",std::ofstream::app);
                                    if (outfile.is_open()){
                                        outfile <<replan_paths[n][z].first<<" ," <<replan_paths[n][z].second<<endl;
                                        outfile <<curent_time_list[z]<<endl;
                                        outfile.close();
                                    }
                                    else{
                                        std::ofstream MyFile("log.txt");
                                        // Write to the file
                                        MyFile <<replan_paths[n][z].first<<" ," <<replan_paths[n][z].second<<endl;
                                        MyFile<<curent_time_list[z]<<endl;
                                        // Close the file
                                        MyFile.close();
                                    }
                                }
                                replan_time_lists[n]=curent_time_list;
                                solution_cost=solution_cost+replan_paths[n].size();
                            }
                            vector<conflict> conflicts=detect_conflict(related_agents, replan_paths, env->curr_timestep);

                            cout<<"The number of conflicts is "<<conflicts.size()<<endl;
                            std::ofstream outfile("log.txt",std::ofstream::app);
                            if (outfile.is_open()){
                                outfile <<"The number of conflicts is "<<conflicts.size()<<endl;
                                outfile.close();
                            }
                            else{
                                std::ofstream MyFile("log.txt");
                                // Write to the file
                                MyFile<<"The number of conflicts is "<<conflicts.size()<<endl;
                                // Close the file
                                MyFile.close();
                            }
                            if  (conflicts.size()==0 and replan_find_flag== true){
                                CBS_replan_flag=true;
                                CBS_terminate_flag=true;
                                agents_index[current_agent] = 1;
                                find_flag=true;
                                for (int n=0;n<related_agents.size();n++){
                                    agents_index[related_agents[n]]=1;
                                    agents_path[related_agents[n]] = replan_paths[n];
                                    agents_time_list[related_agents[n]]=replan_time_lists[n];
                                    //std::ofstream outfile("log.txt",std::ofstream::app);
                                    //if (outfile.is_open()){
                                      //  outfile <<"before update"<<endl;
                                    //}
                                    //else{
                                    //    std::ofstream MyFile("log.txt");
                                        // Write to the file
                                    //    MyFile<<"before update"<<endl;
                                        // Close the file
                                    //    MyFile.close();
                                    //}
                                    //log_safe_intervals(replan_paths[n],related_agents[n]);
                                    SIPP_update_safe_intervals(replan_paths[n],related_agents[n]);
                                   // if (outfile.is_open()){
                                     //   outfile <<"after update"<<endl;
                                      //  outfile.close();
                                    //}
                                    //else{
                                     //   std::ofstream MyFile("log.txt");
                                        // Write to the file
                                      //  MyFile<<"after update!"<<endl;
                                        // Close the file
                                      //  MyFile.close();
                                    //}
                                    //log_safe_intervals(replan_paths[n],related_agents[n]);
                                }
                            }
                            else{
                                bool find_sol_flag=false;
                                vector<pair<int,int>> found_replan_paths[related_agents.size()];
                                vector<int> found_replan_time_lists[related_agents.size()];
                                vector<int> CBS_related_agents;
                                CBS(conflicts,replan_paths,related_agents,agents_time_list,&find_sol_flag,found_replan_paths,found_replan_time_lists,&CBS_related_agents);
                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<"The neighborhood agents: "<<endl;
                                    for (int z=0;z<CBS_related_agents.size();z++){
                                        outfile <<CBS_related_agents[z]<<endl;
                                    }
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    MyFile<<"The neighborhood agents: "<<endl;
                                    for (int z=0;z<CBS_related_agents.size();z++){
                                        MyFile <<CBS_related_agents[z]<<endl;
                                    }
                                    // Close the file
                                    MyFile.close();
                                }
                                if (find_sol_flag== true){
                                    std::ofstream outfile("log.txt",std::ofstream::app);
                                    if (outfile.is_open()){
                                        outfile <<"update path! "<<endl;
                                        outfile.close();
                                    }
                                    else{
                                        std::ofstream MyFile("log.txt");
                                        // Write to the file
                                        MyFile<<"update path!"<<endl;
                                        // Close the file
                                        MyFile.close();
                                    }
                                    CBS_terminate_flag=true;
                                    find_flag=true;
                                    CBS_replan_flag= true;
                                    for (int n=0;n<related_agents.size();n++){
                                        agents_index[related_agents[n]] = 1;
                                        agents_path[related_agents[n]] = found_replan_paths[n];
                                        SIPP_update_safe_intervals(found_replan_paths[n],related_agents[n]);
                                        agents_time_list[related_agents[n]]=found_replan_time_lists[n];
                                    }
                                }
                                else{
                                    related_agents=CBS_related_agents;
                                    std::ofstream outfile("log.txt",std::ofstream::app);
                                    if (outfile.is_open()){
                                        outfile <<"cannot find path!"<<endl;
                                        outfile.close();
                                    }
                                    else{
                                        std::ofstream MyFile("log.txt");
                                        // Write to the file
                                        outfile <<"cannot find path!"<<endl;
                                        // Close the file
                                        MyFile.close();
                                    }
                                    agents_index[current_agent] = 0;
                                }
                            }
                            CBS_replan_count=CBS_replan_count+1;
                            if (CBS_replan_count>3){
                                CBS_terminate_flag= true;
                            }
                        }
                        if (CBS_replan_flag== false){
                            cout<<"begin to generate path!"<<endl;
                            cout<<agents_path[current_agent].size()<<endl;
                            if (!find_flag) {
                                actions[current_agent] = Action::W;
                                continue;
                            } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {

                                actions[current_agent] = Action::FW; //forward action

                                if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                                    replan_flag = true;
                                    cout<<"This robot will reach its goal at next time!"<<endl;
                                    agents_index[current_agent] = 0;
                                }

                            } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                                int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;

                                if (incr == 1 || incr == -3) {

                                    actions[current_agent] = Action::CR; //C--counter clockwise rotate

                                } else if (incr == -1 || incr == 3) {

                                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate

                                }
                                else if (incr==0){
                                    actions[current_agent] = Action::W;
                                }
                            }
                            else{
                                actions[current_agent] = Action::W;
                            }
                        }
                        else{
                            for (int r=0;r<related_agents.size();r++){

                                cout<<"begin to generate path!"<<endl;
                                cout<<r<<endl;
                                current_agent=related_agents[r];
                                already_planned_agents[current_agent]=1;
                                cout<<agents_path[current_agent].size()<<endl;
                                if (!find_flag) {
                                    actions[current_agent] = Action::W;
                                    continue;
                                } else if (agents_path[current_agent][0].first != agents_path[current_agent][1].first) {

                                    actions[current_agent] = Action::FW; //forward action

                                    if (agents_path[current_agent][1].first == env->goal_locations[current_agent].front().first) {
                                        replan_flag = true;
                                        cout<<"This robot will reach its goal at next time!"<<endl;
                                        agents_index[current_agent] = 0;
                                    }

                                } else if (agents_path[current_agent][1].second != agents_path[current_agent][0].second) {

                                    int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;

                                    if (incr == 1 || incr == -3) {

                                        actions[current_agent] = Action::CR; //C--counter clockwise rotate

                                    } else if (incr == -1 || incr == 3) {

                                        actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                                    }
                                }
                                else {
                                    actions[current_agent] = Action::W;
                                }
                                string action="";
                                if (actions[current_agent]==Action::W){
                                    action="wait";
                                }
                                else if (actions[current_agent]==Action::CCR){
                                    action="CCR";
                                }
                                else if (actions[current_agent]==Action::CR){
                                    action="CR";
                                }
                                else if (actions[current_agent]==Action::FW){
                                    action="Forward";
                                }
                                std::ofstream outfile("log.txt",std::ofstream::app);
                                if (outfile.is_open()){
                                    outfile <<"update path of agent "<<related_agents[r]<<endl;
                                    outfile<<agents_path[current_agent][0].first<<" "<<agents_path[current_agent][1].first<<endl;
                                    outfile<<action<<endl;
                                    outfile.close();
                                }
                                else{
                                    std::ofstream MyFile("log.txt");
                                    // Write to the file
                                    MyFile<<"update path of agent "<<related_agents[r]<<endl;
                                    MyFile<<agents_path[current_agent][0].first<<" "<<agents_path[current_agent][1].first<<endl;
                                    MyFile<<action<<endl;
                                    // Close the file
                                    MyFile.close();
                                }
                            }
                        }
                    }

                }
            }
        }

        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent=index[i];
            if (agents_index[current_agent]==-1) {
                agents_index[current_agent]=0;
            }
        }

    } else {
        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent=index[i];
            int current_timestep = agents_index[current_agent];

            cout<<"current agent: "<<current_agent<<endl;
            cout<<"step: "<<current_timestep<<endl;
            cout<<"position on the planned path: "<<agents_path[current_agent][current_timestep].first<<endl;
            cout<<"actual position on the map: "<<env->curr_states[current_agent].location<<endl;
            cout<<"direction on the planned path: "<<agents_path[current_agent][current_timestep].second<<endl;
            cout<<"actual direction on the map: "<<env->curr_states[current_agent].orientation<<endl;
            if (current_timestep>=agents_path[current_agent].size()-1 or current_timestep==0){
                actions[current_agent]=Action::W;
                continue;
            }
            if (agents_path[current_agent][current_timestep].first != agents_path[current_agent][current_timestep+1].first) {
                actions[current_agent] = Action::FW; //forward action
                if (agents_path[current_agent][current_timestep+1].first==env->goal_locations[current_agent].front().first){
                    replan_flag= true;
                    cout<<"This robot will reach its goal at next time!"<<endl;
                    agents_index[current_agent]=0;
                    continue;
                }
            } else if (agents_path[current_agent][current_timestep+1].second != agents_path[current_agent][current_timestep].second) {
                int incr = agents_path[current_agent][current_timestep+1].second - agents_path[current_agent][current_timestep].second;
                if (incr == 1 || incr == -3) {
                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                } else if (incr == -1 || incr == 3) {
                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
                else if (incr==0){
                    actions[current_agent] = Action::W;
                }
            }
            else{
                actions[current_agent] = Action::W;
            }
            agents_index[current_agent]=agents_index[current_agent]+1;
        }
    }
    for (int i=0;i<env->num_of_agents;i++){
        agent_prev_pos[i]=env->curr_states[i].location;
    }

  return;
}

void MAPFPlanner::removeDuplicates(std::vector<int>& nums) {
    unordered_set<int> numSet;
    vector<int> result;

    for (int num : nums) {
        if (numSet.find(num) == numSet.end()) {
            numSet.insert(num);
            result.push_back(num);
        }
    }

    nums = result;
}
vector<pair<int,int>> MAPFPlanner::single_agent_plan_SIPP_with_constraints(int start, int start_direct, int end,vector<node_interval>* all_interval_nodes, bool* find_flag,int agent_id, std::vector<std::vector<std::vector<int>>> constraints,vector<int>* related_agents,vector<int>* current_time_list,bool update_flag,std::vector<std::vector<std::vector<int>>> stay_constraints) {

    int start_time=env->curr_timestep;
    vector<int> current_related_agents=*related_agents;
    vector<pair<int,int>> path;
    vector<node_interval> current_interval=all_interval_nodes[start];
    int rtn_index;
    *find_flag=false;
    cout<<"planning agent "<<agent_id<<endl;
    //for (int r = 0; r<current_interval.size(); r++) {
        //cout<<"(non)safe interval at the start location: "<<current_interval[r].start_timestep<<","<<current_interval[r].end_timestep<<endl;
        //cout<<"starting time: "<< start_time<<endl;
    //}
    node_interval current_safe_interval = compute_current_interval(current_interval,start_time, &rtn_index);
    if (current_safe_interval.is_safe== false){
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (outfile.is_open()){
            outfile <<"something wrong"<<endl;
            outfile <<current_safe_interval.start_timestep<<","<<current_safe_interval.end_timestep<<endl;
            outfile<<current_safe_interval.id<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile <<"something wrong"<<endl;
            MyFile<<current_safe_interval.start_timestep<<","<<current_safe_interval.end_timestep<<endl;
            MyFile<<current_safe_interval.id<<endl;
            // Close the file
            MyFile.close();
        }
    }
    cout<<current_interval[rtn_index].is_safe<<endl;
    int next_agent_id=-1;
    int last_move_pos=-1;
    if (rtn_index+1<current_interval.size()){
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (current_interval[rtn_index+1].is_safe== false){
            next_agent_id=current_interval[rtn_index+1].id;
            last_move_pos=current_interval[rtn_index+1].from_where;
        }
        if (outfile.is_open()){
            outfile <<current_interval[rtn_index+1].id<<endl;
            outfile <<"current agent "<<agent_id<<endl;
            outfile <<current_interval[rtn_index+1].is_safe<<endl;
            outfile <<current_interval[rtn_index+1].start_timestep<<" , "<<current_interval[rtn_index+1].end_timestep<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile <<current_interval[rtn_index+1].id<<endl;
            MyFile<<"current agent "<<agent_id<<endl;
            MyFile <<current_interval[rtn_index+1].is_safe<<endl;
            MyFile<<current_interval[rtn_index+1].start_timestep<<" , "<<current_interval[rtn_index+1].end_timestep<<endl;
            // Close the file
            MyFile.close();
        }
    }
    else{
        std::ofstream outfile("log.txt",std::ofstream::app);
        if (outfile.is_open()){
            outfile <<"current agent "<<agent_id<<endl;
            outfile.close();
        }
        else{
            std::ofstream MyFile("log.txt");
            // Write to the file
            MyFile<<"current agent "<<agent_id<<endl;
            MyFile<<agent_id<<endl;
            // Close the file
            MyFile.close();
        }
    }
    std::ofstream outfile("log.txt",std::ofstream::app);
    if (outfile.is_open()){
        outfile <<"current agent "<<agent_id<<endl;
        outfile.close();
    }
    else{
        std::ofstream MyFile("log.txt");
        // Write to the file
        MyFile<<"current agent "<<agent_id<<endl;
        MyFile<<agent_id<<endl;
        // Close the file
        MyFile.close();
    }
    for (int i=rtn_index+1;i<current_interval.size();i++){
        if (current_interval[i].is_safe== true){
            break;
        }
        else{
            if (current_interval[i].id!=-1){
                current_related_agents.push_back(current_interval[i].id);
            }
        }
    }
    vector<int> current_related_agents_copy=current_related_agents;

    //for (int i=rtn_index+1;i<current_interval.size();i++){
        //if (current_interval[i].is_safe== false){
            //next_agent_id=current_interval[i].id;
            //last_move_pos=current_interval[i].from_where;
            //break;
        //}
    //}
    int maximum_timestep = 10000;
    priority_queue<SIPPNode,vector<SIPPNode>,SIPP_cmp> open_list;
    vector<SIPPNode*> current_reference_list;
    vector<SIPPNode> SIPP_node_list_copy;
    vector<pair<int,int>> agent_interpolate_path;
    unordered_map<int,SIPPNode*> all_nodes;
    unordered_set<int> close_list;
    unordered_set<int> all_nodes_set;
    SIPPNode initial_node = SIPPNode(start_time+0, current_safe_interval, getManhattanDistance(start, end), 0, getManhattanDistance(start, end),
                                     -1,start,start_direct,last_move_pos,next_agent_id);
    open_list.push(initial_node);
    all_nodes[maximum_timestep*maximum_timestep*0+maximum_timestep*0+4*start+start_direct] = &initial_node;
    close_list.insert(maximum_timestep*maximum_timestep*0+maximum_timestep*0+4*start+start_direct);
    all_nodes_set.insert(maximum_timestep*maximum_timestep*0+maximum_timestep*0+4*start+start_direct);
    int insert_key = 0;
    bool terminate_flag = false;
    cout<<"Planning new agent"<<endl;
    SIPPNode* current_node;
    while (open_list.size()>0) {
        if (terminate_flag== true) {
            break;
        }
        SIPPNode curr_node = open_list.top();
        if (curr_node.next_agent_id!=-1 and curr_node.next_agent_id!=NONE){

            current_related_agents.push_back(curr_node.next_agent_id);
        }
        //if (agent_id==0 or agent_id==4) {
        //   cout << safe_intervals[curr_node.location].size() << endl;
        //   cout << last_move_pos[curr_node.location].size() << endl;
        //   cout << curr_node.current_interval_next_pos << endl;
        //   cout << curr_node.location << endl;
        //   for (int p=0;p<last_move_pos[curr_node.location].size();p++){
        //       cout<<last_move_pos[curr_node.location][p].first<<","<<last_move_pos[curr_node.location][p].second<<endl;
        //   }
        //   for (int p=0;p<safe_intervals[curr_node.location].size();p++){
        //      cout<<safe_intervals[curr_node.location][p].first<<","<<safe_intervals[curr_node.location][p].second<<endl;
        //  }
        //   }

        SIPPNode* curr = &curr_node;
        current_reference_list.push_back(curr);
        open_list.pop();
        SIPPNode parent_object=*curr;
        SIPP_node_list_copy.push_back(parent_object);
        if (curr->location == end) {
            int current_arr_time=curr->arrive_time;

            cout<<"find solution!"<<endl;
            *find_flag = true;
            terminate_flag = true;

            vector<SIPPNode> current_path_SIPP_node;
            int count = 0;
            current_path_SIPP_node.push_back(parent_object);
            current_related_agents=current_related_agents_copy;
            //cout<<count<<endl;
            while (curr->parent != -1){
                current_path_SIPP_node.push_back(SIPP_node_list_copy[curr->parent]);
                curr = &SIPP_node_list_copy[curr->parent];
                if (curr->safe_interval.end_timestep-(current_arr_time-1)<=3 and curr->next_agent_id!=-1){
                    current_related_agents.push_back(curr->next_agent_id);
                    current_arr_time=curr->arrive_time;
                }
                count = count+1;
            }
            std::reverse(current_path_SIPP_node.begin(), current_path_SIPP_node.end());
            for (int r=0;r<current_path_SIPP_node.size()-1;r++) {
                int current_pos = current_path_SIPP_node[r].location;
                int current_dir = current_path_SIPP_node[r].arrive_dir;
                int current_time = current_path_SIPP_node[r].arrive_time;
                int next_pos = current_path_SIPP_node[r+1].location;
                int next_dir = current_path_SIPP_node[r+1].arrive_dir;
                int next_time = current_path_SIPP_node[r+1].arrive_time;
                int direction = 0;
                int rotation_time = 0;
                int wait_time = 0;
                if (next_dir-current_dir >= 0) {
                    if (abs(next_dir-current_dir) < 4-abs(next_dir-current_dir)) {
                        direction = 0;
                        rotation_time = abs(next_dir-current_dir);
                        wait_time = next_time-current_time-1-rotation_time;
                    }
                    else {
                        direction = 1;
                        rotation_time = 4-abs(next_dir-current_dir);
                        wait_time = next_time-current_time-1-rotation_time;
                    }
                }
                else {
                    if (4+(next_dir-current_dir)<(current_dir-next_dir)) {
                        direction = 0;
                        rotation_time = 4+(next_dir-current_dir);
                        wait_time = next_time-current_time-1-rotation_time;
                    }
                    else {
                        direction = 1;
                        rotation_time = current_dir-next_dir;
                        wait_time = next_time-current_time-1-rotation_time;
                    }
                }

                agent_interpolate_path.emplace_back(current_pos,current_dir);
                current_time_list->push_back(env->curr_timestep+agent_interpolate_path.size()-1);
                for (int z=0;z<wait_time;z++) {
                    agent_interpolate_path.emplace_back(current_pos,current_dir);
                    current_time_list->push_back(env->curr_timestep+agent_interpolate_path.size()-1);
                }
                for (int z=0;z<rotation_time;z++) {
                    if (direction == 0) {
                        current_dir = current_dir+1;
                    }
                    else {
                        current_dir = current_dir-1;
                    }
                    if (current_dir<0) {
                        current_dir = 3;
                    }
                    else if (current_dir>3) {
                        current_dir= 0 ;
                    }
                    agent_interpolate_path.emplace_back(current_pos,current_dir);
                    current_time_list->push_back(env->curr_timestep+agent_interpolate_path.size()-1);
                }
            }
            agent_interpolate_path.emplace_back(current_path_SIPP_node[current_path_SIPP_node.size()-1].location,current_path_SIPP_node[current_path_SIPP_node.size()-1].arrive_dir);
            current_time_list->push_back(env->curr_timestep+agent_interpolate_path.size()-1);
            for (int z=0;z<agent_interpolate_path.size();z++) {
                cout<<"location: "<<agent_interpolate_path[z].first<<endl;
                cout<<"direction: "<<agent_interpolate_path[z].second<<endl;
            }

            cout<<"update intervals!"<<endl;
            if (update_flag== true){
                SIPP_update_safe_intervals(agent_interpolate_path, agent_id);
            }
            cout<<"finish updating intervals!"<<endl;
            current_node = curr;
            break;
        }
        map = env->map;
        SIPPNode* sipp_node = curr;

        int location = sipp_node->location;
        vector<int> all_forbid_time;
        for (int i=0;i<stay_constraints[location][0].size();i++){
            all_forbid_time.push_back(stay_constraints[location][0][i]);
        }
        int maximum_leave_time = -1;
        bool check_conflict_flag= false;
        int check_time=curr_node.arrive_time;
        for (int r=0;r<all_forbid_time.size();r++){
            if (all_forbid_time[r]==check_time){
                check_conflict_flag= true;
            }
        }
        if (check_conflict_flag== false){
            maximum_leave_time=check_time;
            for (int p=curr_node.arrive_time;p<=curr_node.safe_interval.end_timestep;p++){
                bool check_conflict_flag= false;
                int check_time=p+1;
                if (check_time>curr_node.safe_interval.end_timestep){
                    break;
                }
                for (int r=0;r<all_forbid_time.size();r++){
                    if (all_forbid_time[r]==check_time){
                        check_conflict_flag= true;
                    }
                }
                if (check_conflict_flag== false){
                    maximum_leave_time=check_time;
                }
                else{
                    break;
                }
            }
        }
        else{
            maximum_leave_time=-1;
        }

        if (maximum_leave_time==-1){
            continue;
        }
        if (sipp_node->safe_interval.end_timestep-maximum_leave_time!=0){
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<"robot "<<agent_id<< " needs to move before "<<maximum_leave_time<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile <<"robot "<<agent_id<< " needs to move before "<<maximum_leave_time<<endl;
                // Close the file
                MyFile.close();
            }
        }

        int move_time = maximum_leave_time-sipp_node->arrive_time+1;
        int maximum_rot_time = move_time-1;
        if (maximum_rot_time<0){
            continue;
        }

        int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
        int minimum_rot_step[4] = {0,0,0,0};
        int objects_num = SIPP_node_list_copy.size();


        for (int i=0;i<4;i++) {

            if (location<env->cols){
                if (i==3){
                    continue;
                }
            }
            else if (location>=env->cols*(env->rows-1)){
                if (i==1){
                    continue;
                }
            }
            else if (location%env->cols==0){
                if (i==2){
                    continue;
                }
            }
            else if ((location+1)%env->cols==0){
                if (i==0){
                    continue;
                }
            }
            int check_candidate = candidates[i];
            vector<int> current_constraint=constraints[location][i];
            if (sipp_node->arrive_dir == i) {
                minimum_rot_step[i] = 0;
            }
            else {
                if (abs(sipp_node->arrive_dir-i)>4-abs(i-sipp_node->arrive_dir)) {
                    minimum_rot_step[i] = 4-abs(i-sipp_node->arrive_dir);
                } else {
                    minimum_rot_step[i] = abs(sipp_node->arrive_dir-i);
                }
            }

            if (check_candidate < 0 or check_candidate >= env->map.size()) {
                continue;
            }

            if (minimum_rot_step[i] > maximum_rot_time or map[check_candidate] == 1){
                continue;
            }

            int minimum_leave_time = sipp_node->arrive_time+minimum_rot_step[i];
            bool conflict_flag= false;
            while (conflict_flag== false){
                conflict_flag= true;
                for (int r=0;r<current_constraint.size();r++){
                    if (minimum_leave_time==current_constraint[r]){
                        conflict_flag= false;
                        std::ofstream outfile("log.txt",std::ofstream::app);
                        if (outfile.is_open()){
                        outfile <<"the robot "<<agent_id<< " cannot move at this time "<<current_constraint[r]<<" at position "<<location<<" at direction "<<i<<endl;
                        outfile.close();
                        }
                        else{
                        std::ofstream MyFile("log.txt");
                        // Write to the file
                        MyFile <<"the robot "<<agent_id<< " cannot move at this time "<<current_constraint[r]<<" at position "<<location<<" at direction "<<i<<endl;
                        // Close the file
                        MyFile.close();
                        }
                        break;
                    }
                }
                if (conflict_flag== true){
                    break;
                }
                minimum_leave_time=minimum_leave_time+1;
                if (minimum_leave_time>sipp_node->safe_interval.end_timestep){
                    std::ofstream outfile("log.txt",std::ofstream::app);
                    if (outfile.is_open()){
                        outfile <<"must leave"<<endl;
                        outfile.close();
                    }
                    else{
                        std::ofstream MyFile("log.txt");
                        // Write to the file
                        MyFile <<"must leave"<<endl;
                        // Close the file
                        MyFile.close();
                    }
                    break;
                }
            }
            vector<node_interval> candidate_intervals = all_interval_nodes[check_candidate];

            int intervals_num = candidate_intervals.size();


            if (curr->current_interval_next_pos != -1 and check_candidate == curr->current_interval_next_pos) {
                if (maximum_leave_time>=curr_node.safe_interval.end_timestep){
                    maximum_leave_time = curr_node.safe_interval.end_timestep-1;
                }
            }
            else if (maximum_leave_time>=curr_node.safe_interval.end_timestep){
                maximum_leave_time=curr_node.safe_interval.end_timestep;
            }

            if (minimum_leave_time>maximum_leave_time){
                continue;
            }
            int current_index=0;
            compute_current_interval(candidate_intervals,env->curr_timestep, &current_index);

            for (int j=0; j<intervals_num; j++) {

                node_interval current_interval = candidate_intervals[j];

                if (current_interval.is_safe== false){
                    continue;
                }
                if (current_interval.start_timestep > maximum_leave_time+1) {
                    continue;
                } else if (current_interval.end_timestep<minimum_leave_time+1) {
                    continue;
                }
                int next_agent_id=-1;
                int last_move_pos=-1;
                if (j+1<candidate_intervals.size()){
                    if (candidate_intervals[j+1].is_safe== false){
                        next_agent_id=candidate_intervals[j+1].id;
                        last_move_pos=candidate_intervals[j+1].from_where;
                    }
                }
                //if (next_agent_id!=-1){
                    //cout<<next_agent_id<<endl;
                //}

                //for (int p=current_index;p<candidate_intervals.size();p++){
                    //if (candidate_intervals[p].is_safe== false and candidate_intervals[p].start_timestep>current_interval.end_timestep){
                        //next_agent_id=candidate_intervals[p].id;
                        //last_move_pos=candidate_intervals[p].from_where;
                        //break;
                    //}
                //
                vector<int> candidate_forbid_list;;
                vector<pair<int,int>> candidate_valid_intervals;
                for (int n=0;n<stay_constraints[check_candidate][0].size();n++){
                    if (stay_constraints[check_candidate][0][n]>=current_interval.start_timestep and stay_constraints[check_candidate][0][n]<=current_interval.end_timestep){
                        candidate_forbid_list.push_back(stay_constraints[check_candidate][0][n]);
                    }
                }
                sort(candidate_forbid_list.begin(), candidate_forbid_list.end());
                if (candidate_forbid_list.empty()){
                    candidate_valid_intervals.push_back(make_pair(current_interval.start_timestep,current_interval.end_timestep));
                }
                else {
                    int check_time=current_interval.start_timestep;
                    for (int z=0;z<candidate_forbid_list.size();z++){
                        if (check_time<candidate_forbid_list[z]-1){
                            candidate_valid_intervals.push_back(make_pair(check_time,candidate_forbid_list[z]-1));
                        }
                        check_time=candidate_forbid_list[z]+1;
                    }
                    if (check_time<current_interval.end_timestep-1){
                        candidate_valid_intervals.push_back(make_pair(check_time,current_interval.end_timestep-1));
                    }
                    //std::ofstream outfile("log.txt",std::ofstream::app);
                    //if (outfile.is_open()){
                        //outfile <<"There are "<<candidate_valid_intervals.size()<<" valid intervals"<<endl;
                        //outfile.close();
                    //}
                    //else{
                        //std::ofstream MyFile("log.txt");
                        // Write to the file
                       // outfile <<"There are "<<candidate_valid_intervals.size()<<" valid intervals"<<endl;
                        // Close the file
                        //MyFile.close();
                   // }
                }
                for (int e=0;e<candidate_valid_intervals.size();e++){
                    //cout<<e<<endl;
                    //cout<<candidate_valid_intervals[e].first<<","<<candidate_valid_intervals[e].second<<endl;
                    pair<int,int> current_candidate_interval=candidate_valid_intervals[e];
                    if (minimum_leave_time+1>current_candidate_interval.second or maximum_leave_time+1<current_candidate_interval.first){
                        continue;
                    }
                    int min_t=max(minimum_leave_time+1,current_candidate_interval.first);
                    int max_t=min(maximum_leave_time+1,current_candidate_interval.second);
                    if (min_t>max_t){
                        continue;
                    }
                    int chose_t=-1;
                    for (int candidate_arr_time=min_t;candidate_arr_time<=max_t;candidate_arr_time++){
                        conflict_flag= false;
                        for (int r=0;r<current_constraint.size();r++){
                            if (candidate_arr_time-1==current_constraint[r]){
                                conflict_flag= true;
                            }
                        }
                        if (conflict_flag== false){
                            chose_t=candidate_arr_time;
                            break;
                        }
                    }
                    if (chose_t==-1){
                        continue;
                    }
                    else{
                        int t=chose_t;
                        if (t<current_interval.start_timestep or t>current_interval.end_timestep or
                            !current_interval.is_safe){
                            continue;
                        }
                        if (t-1>maximum_leave_time){
                            continue;
                        }
                        SIPPNode new_node = SIPPNode(t, current_interval, t+getManhattanDistance(check_candidate, end), t, getManhattanDistance(check_candidate, end),
                                                     objects_num-1,check_candidate,i,last_move_pos,next_agent_id);

                        insert_key = e*maximum_timestep*maximum_timestep+j*maximum_timestep+4*new_node.location+new_node.arrive_dir;
                        auto found = close_list.find(insert_key);
                        if (found != close_list.end()) {

                        } else {
                            auto found = all_nodes_set.find(insert_key);
                            if (found == all_nodes_set.end()) {
                                all_nodes_set.insert(insert_key);
                                all_nodes[insert_key] = &new_node;
                                open_list.push(new_node);
                                close_list.insert(insert_key);
                            } else {
                                SIPPNode* exist_node = all_nodes[insert_key];
                                if (exist_node->arrive_time > new_node.arrive_time) {
                                    exist_node->arrive_time = new_node.arrive_time;
                                    exist_node->safe_interval = new_node.safe_interval;
                                    exist_node->parent = new_node.parent;
                                    exist_node->next_agent_id=new_node.next_agent_id;
                                    exist_node->current_interval_next_pos = new_node.current_interval_next_pos;
                                    exist_node->f = new_node.f;
                                    exist_node->g = new_node.g;
                                }
                            }

                        }
                    }
                }



                //SIPPNode_list_object.push_back(new_node);
                //SIPPNode_list.push_back(&new_node);
                //cout<<curr->location<<endl;
            }

        }
        //vector<SIPPNode> SIPPNode_list=SIPP_get_neighbor(curr,last_move_pos,safe_intervals,end);
        //cout<<"end getting neighborhoods"<<endl;

        //cout<<"size of neighbors"<<endl;
        //cout<<"size of neighbors"<<SIPP_list.size()<<endl;
        //for (int i=0;i<SIPPNode_list.size();i++){
        //bool flag= false;
        //insert_key=SIPPNode_list[i].arrive_time*maximum_timestep+SIPPNode_list[i].location;
        //auto found = close_list.find(insert_key);
        //if (found != close_list.end()) {

        //} else {
        //open_list.push(&SIPPNode_list[i]);
        //cout<<SIPP_list[i].parent->location<<endl;
        //close_list.insert(insert_key);
        //}

        //}
    }
    removeDuplicates(current_related_agents);
    *related_agents=current_related_agents;
    for (int i=0;i<current_related_agents.size();i++){
        cout<<"one related agent is "<<current_related_agents[i]<<endl;
    }
    return agent_interpolate_path;
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}
void MAPFPlanner::insert_safe_intervals_from_path(vector<int>agents_id,vector<pair<int,int>>* agents_path,vector<int>* agents_time){
    for (int i=0;i<agents_id.size();i++){
        int current_id=agents_id[i];
        vector<pair<int,int>> current_agent_path=agents_path[current_id];
        vector<int> current_time_list=agents_time[current_id];
        int start_time=-1;
        for (int j=0;j<current_time_list.size();j++){
            if (current_time_list[j]==env->curr_timestep){
                start_time=j;
                break;
            }
        }
        if (start_time==-1){
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<"something wrong in the timelist"<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile <<"something wrong in the timelist"<<endl;
                // Close the file
                MyFile.close();
            }
            continue;
        }

        for (int j=start_time;j<current_agent_path.size();j++){
            cout<<current_agent_path[j].first<<","<<current_time_list[j]<<endl;
            insert_safe_intervals(current_agent_path[j].first,current_time_list[j]);
        }
    }
}
void MAPFPlanner::insert_safe_intervals_from_one_path(int agents_id,vector<pair<int,int>> agents_path,vector<int> agents_time){
    int current_id=agents_id;
    vector<pair<int,int>> current_agent_path=agents_path;
    vector<int> current_time_list=agents_time;
    int start_time=-1;
    for (int j=0;j<current_time_list.size();j++){
        if (current_time_list[j]==env->curr_timestep){
            start_time=j;
            break;
        }
    }
    if (start_time==-1){
        return;
    }

    for (int j=start_time;j<current_agent_path.size();j++){
        cout<<current_agent_path[j].first<<","<<current_time_list[j]<<endl;
        insert_safe_intervals(current_agent_path[j].first,current_time_list[j]);
    }
}
bool MAPFPlanner::validateMove(int loc, int loc2) {

    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}

list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction) {

    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}
void MAPFPlanner::log_safe_intervals(vector<pair<int, int>> agent_planned_path, int agent_id){
    vector<int> last_position_list;
    last_position_list.push_back(-1);
    std::ofstream outfile("log.txt",std::ofstream::app);
    if (outfile.is_open()){
        outfile <<"log agent "<<agent_id<<endl;
        outfile.close();
    }
    else{
        std::ofstream MyFile("log.txt");
        // Write to the file
        MyFile <<"log agent "<<agent_id<<endl;
        // Close the file
        MyFile.close();
    }
    for (int i = 0; i < RHCR_w; i++) {
        int location;

        if (i == agent_planned_path.size()) {
            break;
        }

        location = agent_planned_path[i].first;

        if (location!=last_position_list[last_position_list.size()-1]){
            last_position_list.push_back(location);
        }

        int last_position=last_position_list[last_position_list.size()-2];
        if (last_position==-1){
            continue;
        }
        vector<node_interval> current_intervals=all_interval_nodes[last_position];
        for (int k=0;k<current_intervals.size();k++){
            std::ofstream outfile("log.txt",std::ofstream::app);
            if (outfile.is_open()){
                outfile <<last_position<<endl;
                outfile <<current_intervals[k].start_timestep<<" , "<<current_intervals[k].end_timestep<<endl;
                outfile <<current_intervals[k].is_safe<<endl;
                outfile.close();
            }
            else{
                std::ofstream MyFile("log.txt");
                // Write to the file
                MyFile<<last_position<<endl;
                MyFile <<current_intervals[k].start_timestep<<" , "<<current_intervals[k].end_timestep<<endl;
                MyFile<<current_intervals[k].is_safe<<endl;
                // Close the file
                MyFile.close();
            }
        }
    }
}
/**
 * @brief update the safe intervals of all vertices
 * @param planned_path The planned path of an agent, whose element is a pair of the location and the direction
 * @param current_time_step The current time step
 * @param rhcr_w The time interval we want to resolve conflicts
*/
void MAPFPlanner::SIPP_update_safe_intervals(vector<pair<int, int>> agent_planned_path, int agent_id) {
    vector<int> last_position_list;
    last_position_list.push_back(-1);
    for (int i = 0; i < RHCR_w; i++) {

        int location;

        if (i == agent_planned_path.size()) {
            break;
        }

        location = agent_planned_path[i].first;

        if (location!=last_position_list[last_position_list.size()-1]){
            last_position_list.push_back(location);
        }

        int last_position=last_position_list[last_position_list.size()-2];
        int current_time_step=env->curr_timestep+i;
        int rtn_index;
        int intervals_num=all_interval_nodes[location].size();

        compute_current_interval(all_interval_nodes[location],current_time_step,&rtn_index);
        if (all_interval_nodes[location][rtn_index].is_safe== false){
            cout<<"something wrong takes place!"<<endl;
        }
        else if (current_time_step==all_interval_nodes[location][rtn_index].start_timestep) {
            all_interval_nodes[location][rtn_index].start_timestep = all_interval_nodes[location][rtn_index].start_timestep + 1;
            if (rtn_index>=1){
                if (all_interval_nodes[location][rtn_index-1].is_safe== false){
                    /*
                         I think you should update the all_interval_nodes[location][rtn_index-1].id/from_where only if *.id==agent_id 
                    */                    
                   if (all_interval_nodes[location][rtn_index-1].id==agent_id and all_interval_nodes[location][rtn_index-1].from_where==last_position){
                        all_interval_nodes[location][rtn_index-1].end_timestep=all_interval_nodes[location][rtn_index-1].end_timestep+1;
                       all_interval_nodes[location][rtn_index].start_timestep=all_interval_nodes[location][rtn_index-1].end_timestep+1;
                       if (all_interval_nodes[location][rtn_index].start_timestep>all_interval_nodes[location][rtn_index].end_timestep){
                           all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index);
                       }
                    }

                    else{ 
                        node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                        all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
                       all_interval_nodes[location][rtn_index+1].start_timestep=all_interval_nodes[location][rtn_index].end_timestep+1;
                       if (all_interval_nodes[location][rtn_index+1].start_timestep>all_interval_nodes[location][rtn_index+1].end_timestep){
                           all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index+1);
                       }
                    }
                }
                else{
                    node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                    all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
                    all_interval_nodes[location][rtn_index+1].start_timestep=all_interval_nodes[location][rtn_index].end_timestep+1;
                    if (all_interval_nodes[location][rtn_index+1].start_timestep>all_interval_nodes[location][rtn_index+1].end_timestep){
                        all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index+1);
                    }
                }
            }
            else{
                node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
                all_interval_nodes[location][rtn_index+1].start_timestep=all_interval_nodes[location][rtn_index].end_timestep+1;
                if (all_interval_nodes[location][rtn_index+1].start_timestep>all_interval_nodes[location][rtn_index+1].end_timestep){
                    all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index+1);
                }
            }


        } else if (current_time_step==all_interval_nodes[location][rtn_index].end_timestep) {

            all_interval_nodes[location][rtn_index].end_timestep = all_interval_nodes[location][rtn_index].end_timestep - 1;
            node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
            all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+1,new_node);
            if (all_interval_nodes[location][rtn_index].start_timestep>all_interval_nodes[location][rtn_index].end_timestep){
                all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index);
            }

        } else {
            int original_first_pos=all_interval_nodes[location][rtn_index].start_timestep;
            int original_last_pos=all_interval_nodes[location][rtn_index].end_timestep;
            all_interval_nodes[location][rtn_index].end_timestep = current_time_step - 1;
            node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
            all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+1,new_node);
            if (original_last_pos>=current_time_step+1){
                node_interval insert_node2=node_interval(true,current_time_step+1,original_last_pos,-1,-1);
                all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+2,insert_node2);
            }
            if (all_interval_nodes[location][rtn_index].start_timestep>all_interval_nodes[location][rtn_index].end_timestep){
                all_interval_nodes[location].erase(all_interval_nodes[location].begin()+rtn_index);
            }

        }
    }
}

