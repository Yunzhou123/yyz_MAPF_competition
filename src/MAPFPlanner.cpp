#include <MAPFPlanner.h>
#include <random>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
using namespace std::chrono;
int a=1+1;

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
            *rtn_interval=current_safe_intervals[check_index];
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
    agents_index = new int [env->num_of_agents];
    //this->occupy_id = new vector<pair<int,int>>[env->rows*env->cols];

    //safe_intervals = new vector<pair<int,int>>[env->rows*env->cols];
    //last_move_pos = new vector<pair<int,int>>[env->rows*env->cols];
    all_interval_nodes=new vector<node_interval>[env->rows*env->cols];

    for (int i = 0; i < env->rows*env->cols; ++i) {
        pair<int, int> initial_interval;
        pair<int, int> initial_lastmove;
        initial_interval.first = 0;
        initial_interval.second = 100000;
        initial_lastmove.first = -1;
        initial_lastmove.second = -1;
        node_interval initial_node=node_interval(true,0,100000,-1,-1);
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

void MAPFPlanner::insert_safe_intervals(int location, int time, int last_pos,int agent_id){

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

    }
    else if (time==current_node_intervals[index_num].end_timestep){
        if (current_node_intervals[index_num+1].is_safe== true) {
            
            // what if the current_node_intervals[index_num].start_timestep == current_node_intervals[index_num].end_timestep
            current_node_intervals[index_num].end_timestep=current_node_intervals[index_num].end_timestep-1; 

            current_node_intervals[index_num+1].start_timestep=current_node_intervals[index_num+1].start_timestep-1;
        }
        else {
            // same problem as above
            current_node_intervals[index_num].end_timestep=current_node_intervals[index_num].end_timestep-1;
            node_interval new_node=node_interval(true,time,time,-1,-1);
            current_node_intervals.insert(current_node_intervals.begin()+index_num+1,new_node);
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
            }
            else{
                current_node_intervals[index_num].start_timestep=current_node_intervals[index_num].start_timestep+1;
                node_interval new_node=node_interval(true,time,time,-1,-1);
                current_node_intervals.insert(current_node_intervals.begin()+index_num,new_node);
            }
        }
    }
    else { //Seems impossible to happen
        cout<<"something wrong happens!"<<endl;
        int id_copy=current_node_intervals[index_num].id;
        int from_where_copy=current_node_intervals[index_num].from_where;
        int end_time_copy=current_node_intervals[index_num].end_timestep;
        current_node_intervals[index_num].end_timestep=time-1;
        /*
            should also update drom_where in the current_node_intervals[index_num]
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
    if (env->curr_timestep>300) {
        // leave empty for testing
    } else if (replan_all_flag==true) {
        /*
            should delete the old allocated space (otherwise will leave the job for the garbage collector)
        */
        delete []all_interval_nodes;
        all_interval_nodes=new vector<node_interval>[env->rows*env->cols];
        //safe_intervals = new vector<pair<int,int>>[env->rows*env->cols];
        //last_move_pos = new vector<pair<int,int>>[env->rows*env->cols];
        //delete []occupy_id;
        //occupy_id = new vector<pair<int,int>>[env->rows*env->cols];

        for (int i = 0; i < env->rows*env->cols; ++i) {
            // initialize safe_intervals
            node_interval initial_node=node_interval(true,0,100000,-1,-1);
            all_interval_nodes[i].push_back(initial_node);
            //safe_intervals[i].push_back(make_pair(1, 100000));
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
            int current_position = env->curr_states[current_agent].location;
            int current_map_h = -1;
            int current_map_w = -1;
            cout << "current agent: " << current_agent << endl;
            cout << "Begin the planning of an agent" << endl;

            vec_index_to_map_index(current_map_h, current_map_w, current_position);
            vector<pair<int, int>> path;
            agent_path_index[current_agent] = agent_path_index[current_agent] + 1;
            if (plan_first_time_flag== true){
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep,-1,current_agent);
            }
            else{
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep,agents_path[current_agent][agents_path[current_agent].size()-2].first,current_agent);
            }

            if (env->goal_locations[current_agent].empty()) {
                agents_path[current_agent].push_back(
                        {env->curr_states[current_agent].location, env->curr_states[current_agent].orientation});
            } else {
                cout << "Begin to use SIPP" << endl;
                bool find_flag = false;
                vector<int> related_agents;
                path = single_agent_plan_SIPP_with_constraints(env->curr_states[current_agent].location,
                                              env->curr_states[current_agent].orientation,
                                              env->goal_locations[current_agent].front().first, all_interval_nodes,
                                              &find_flag, current_agent,constraints,&related_agents);
                if (find_flag== true) {

                    agents_index[current_agent] = 1;
                }
                else {
                    std::ofstream myfile ("/home/joe-yan/Desktop/Yunzhou123-main/text_files/example.txt");
                    if (myfile.is_open())
                    {
                        myfile <<std::to_string(related_agents.size())<< "\n";

                    }
                    myfile.close();
                    agents_index[current_agent] = 0;
                }

                agents_path[current_agent] = path;
                cout << "begin to generate path!" << endl;
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
                }
            }
        }
        if (plan_first_time_flag== true){
            plan_first_time_flag= false;
        }

    }
    else if (replan_flag) {
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
            }
            agents_index[current_agent] = agents_index[current_agent]+1;
        }
        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent=index[i];
            if (agents_index[current_agent]==0) {
                cout<<"insert intervals"<<endl;
                for (int p=0;p<all_interval_nodes[env->curr_states[current_agent].location].size();p++){
                    cout<<all_interval_nodes[env->curr_states[current_agent].location][p].start_timestep<<","<<all_interval_nodes[env->curr_states[current_agent].location][p].end_timestep<<endl;

                }
                for (int p=0;p<all_interval_nodes[env->curr_states[current_agent].location].size();p++){
                    cout<<all_interval_nodes[env->curr_states[current_agent].location][p].from_where<<endl;

                }
                insert_safe_intervals(env->curr_states[current_agent].location, env->curr_timestep,agents_path[current_agent][agents_path[current_agent].size()-2].first,current_agent);
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
                    path = single_agent_plan_SIPP_with_constraints(env->curr_states[current_agent].location,
                                                env->curr_states[current_agent].orientation,
                                                env->goal_locations[current_agent].front().first,all_interval_nodes,&find_flag, current_agent,constraints,&related_agents);
                    if (find_flag==true) {

                        agents_index[current_agent] = 1;
                    } else {
                        std::ofstream myfile ("/home/joe-yan/Desktop/Yunzhou123-main/text_files/example.txt");
                        if (myfile.is_open())
                        {
                            myfile <<std::to_string(related_agents.size())<< "\n";

                        }
                        myfile.close();
                        agents_index[current_agent] = 0;
                    }
                    agents_path[current_agent] = path;
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
            }
            agents_index[current_agent]=agents_index[current_agent]+1;
        }
    }

  return;
}


vector<pair<int,int>> MAPFPlanner::single_agent_plan_SIPP_with_constraints(int start, int start_direct, int end,vector<node_interval>* all_interval_nodes, bool* find_flag,int agent_id, std::vector<std::vector<std::vector<int>>> constraints,vector<int>* related_agents) {

    int start_time=env->curr_timestep;
    vector<int> current_related_agents=*related_agents;
    vector<pair<int,int>> path;
    vector<node_interval> current_interval=all_interval_nodes[start];
    int rtn_index;
    *find_flag=false;
    for (int r = 0; r<current_interval.size(); r++) {
        cout<<"(non)safe interval at the start location: "<<current_interval[r].start_timestep<<","<<current_interval[r].end_timestep<<endl;
        cout<<"starting time: "<< start_time<<endl;
    }
    node_interval current_safe_interval = compute_current_interval(current_interval,start_time, &rtn_index);
    int next_agent_id=-1;
    int last_move_pos=-1;
    for (int i=rtn_index+1;i<current_interval.size();i++){
        if (current_interval[i].is_safe== false){
            next_agent_id=current_interval[i].id;
            last_move_pos=current_interval[i].from_where;
            break;
        }
    }

    int maximum_timestep = 100000;
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
    all_nodes[maximum_timestep*0+4*start+start_direct] = &initial_node;
    close_list.insert(maximum_timestep*0+4*start+start_direct);
    all_nodes_set.insert(maximum_timestep*0+4*start+start_direct);
    int insert_key = 0;
    bool terminate_flag = false;
    cout<<"Planning new agent"<<endl;
    SIPPNode* current_node;
    while (open_list.size()>0) {
        if (terminate_flag== true) {
            break;
        }
        SIPPNode curr_node = open_list.top();
        if (curr_node.next_agent_id!=-1){
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
            *find_flag = true;
            terminate_flag = true;

            vector<SIPPNode> current_path_SIPP_node;
            int count = 0;
            current_path_SIPP_node.push_back(parent_object);
            //cout<<count<<endl;
            while (curr->parent != -1){
                current_path_SIPP_node.push_back(SIPP_node_list_copy[curr->parent]);
                curr = &SIPP_node_list_copy[curr->parent];
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
                for (int z=0;z<wait_time;z++) {
                    agent_interpolate_path.emplace_back(current_pos,current_dir);
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
                }
            }
            agent_interpolate_path.emplace_back(current_path_SIPP_node[current_path_SIPP_node.size()-1].location,current_path_SIPP_node[current_path_SIPP_node.size()-1].arrive_dir);
            for (int z=0;z<agent_interpolate_path.size();z++) {
                cout<<"location: "<<agent_interpolate_path[z].first<<endl;
                cout<<"direction: "<<agent_interpolate_path[z].second<<endl;
            }

            cout<<"update intervals!"<<endl;
            SIPP_update_safe_intervals(agent_interpolate_path, agent_id);
            cout<<"finish updating intervals!"<<endl;
            current_node = curr;
            break;
        }
        map = env->map;
        SIPPNode* sipp_node = curr;
        int move_time = sipp_node->safe_interval.end_timestep-sipp_node->arrive_time+1;
        int maximum_rot_time = move_time-1;
        int location = sipp_node->location;
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
                        break;
                    }
                }
                if (conflict_flag== true){
                    break;
                }
                minimum_leave_time=minimum_leave_time+1;
                if (minimum_leave_time>=sipp_node->safe_interval.end_timestep){
                    break;
                }
            }

            vector<node_interval> candidate_intervals = all_interval_nodes[check_candidate];

            int intervals_num = candidate_intervals.size();
            int maximum_leave_time = sipp_node->safe_interval.end_timestep;
            if (curr->current_interval_next_pos != -1 and check_candidate == curr->current_interval_next_pos) {
                maximum_leave_time = maximum_leave_time-1;
            }
            if (minimum_leave_time>maximum_leave_time){
                continue;
            }

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
                for (int p=rtn_index+1;p<candidate_intervals.size();p++){
                    if (candidate_intervals[p].is_safe== false){
                        next_agent_id=candidate_intervals[p].id;
                        last_move_pos=candidate_intervals[p].from_where;
                        break;
                    }
                }
                int t = max(minimum_leave_time+1,current_interval.start_timestep);

                SIPPNode new_node = SIPPNode(t, current_interval, t+getManhattanDistance(check_candidate, end), t, getManhattanDistance(check_candidate, end),
                                             objects_num-1,check_candidate,i,last_move_pos,next_agent_id);

                insert_key = j*maximum_timestep+4*new_node.location+new_node.arrive_dir;
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
                            exist_node->current_interval_next_pos = new_node.current_interval_next_pos;
                            exist_node->f = new_node.f;
                            exist_node->g = new_node.g;
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
    *related_agents=current_related_agents;
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
                    }

                    else{ 
                        node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                        all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
                    }
                }
                else{
                    node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                    all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
                }
            }
            else{
                node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
                all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index,new_node);
            }


        } else if (current_time_step==all_interval_nodes[location][rtn_index].end_timestep) {

            all_interval_nodes[location][rtn_index].end_timestep = all_interval_nodes[location][rtn_index].end_timestep - 1;
            node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
            all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+1,new_node);


        } else {
            int original_first_pos=all_interval_nodes[location][rtn_index].start_timestep;
            int original_last_pos=all_interval_nodes[location][rtn_index].end_timestep;
            all_interval_nodes[location][rtn_index].end_timestep = current_time_step - 1;
            node_interval new_node=node_interval(false,current_time_step,current_time_step,agent_id,last_position);
            all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+1,new_node);
            node_interval insert_node2=node_interval(true,current_time_step+1,original_last_pos,-1,-1);
            all_interval_nodes[location].insert(all_interval_nodes[location].begin()+rtn_index+2,insert_node2);
        }
    }
}