#include <MAPFPlanner.h>
#include <random>
#include <algorithm>

struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};

struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};
struct SIPP_cmp
{
    bool operator()(SIPPNode a, SIPPNode b)
    {
        if(a.f == b.f) return a.g <= b.g;
        else return a.f > b.f;
    }
};
int a=1+1;

/**
 * @brief argsort
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
 * @return The corresponding safe interval
*/
pair<int,int> MAPFPlanner::compute_current_interval(vector<pair<int,int>> current_safe_intervals,int current_time,int* rtn_index){
    int intervals_num=current_safe_intervals.size();
    int low_index=0;
    int high_index=intervals_num-1;
    int check_index=int((low_index+high_index)/2);
    bool flag= false;
    pair<int,int> rtn_interval=make_pair(0,0);
    while (flag== false){
        //cout<<current_time<<endl;
        //cout<<current_safe_intervals[check_index].first<<","<<current_safe_intervals[check_index].second<<endl;
        if ((current_time<=current_safe_intervals[check_index].second) and (current_time>=current_safe_intervals[check_index].first)){
            rtn_interval=current_safe_intervals[check_index];
            flag= true;
        }
        else{
            if (high_index==low_index+1){
                if (current_time>current_safe_intervals[check_index].second){
                    check_index=high_index;
                    low_index=high_index;
                }
                else{
                    check_index=low_index;
                    high_index=low_index;
                }
            }
            else {
                if (current_time>current_safe_intervals[check_index].second){
                    low_index=check_index;
                }
                else{
                    high_index=check_index;
                }
                check_index=int((low_index+high_index)/2);
            }
        }
    }
    *rtn_index=check_index;
    return rtn_interval;
}

/// initialize the planner
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    MAPFPlanner::map=env->map;
    //Initialize a priority order
    int new_order[env->num_of_agents];
    int new_agent_path_index[env->num_of_agents];
    for (int i = 0; i < env->num_of_agents; ++i) {
        new_order[i] = i;
        new_agent_path_index[i] = 0;
    }
    std::shuffle(&new_order[0],&new_order[env->num_of_agents], std::mt19937(std::random_device()()));
    //for (int i = 0; i < env->num_of_agents; ++i) {
        //cout<<new_order[i]<<endl;
    //}
    MAPFPlanner::priority_order=new_order;
    MAPFPlanner::agent_path_index=new_agent_path_index;
    MAPFPlanner::RHCR_w=20;
    MAPFPlanner::RHCR_h=15;
    MAPFPlanner::agents_path=new vector<pair<int,int>>[env->num_of_agents];
    MAPFPlanner::safe_intervals=new vector<pair<int,int>>[env->rows*env->cols];
    MAPFPlanner::last_move_pos=new vector<pair<int,int>>[env->rows*env->cols];
    for (int i = 0; i < env->num_of_agents; ++i) {
    cout<<priority_order[i]<<endl;
    }
    for (int i = 0; i < env->rows*env->cols; ++i) {
        pair<int, int> initial_interval;
        pair<int, int> initial_lastmove;
        initial_interval.first=0;
        initial_interval.second=100000;
        initial_lastmove.first=-1;
        initial_lastmove.second=-1;
        safe_intervals[i].push_back(initial_interval);
        last_move_pos[i].push_back(initial_lastmove);
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
    vec_index=map_h*env->cols+map_w;
}
/// convert 1D index to 2D index
void MAPFPlanner::vec_index_to_map_index(int& map_h, int& map_w,int vec_index)
{
    map_h=int(vec_index/env->cols);
    map_w=vec_index-map_h*env->cols;
}

/// RHCR Algorithm
bool MAPFPlanner::decide_when_to_plan(int current_timestep,int RHCR_h){
bool flag= false;
float residue=current_timestep-RHCR_h*int(current_timestep/RHCR_h);
if (residue==0){
    flag= true;
}
else{
    flag= false;
}
return flag;
}
// plan and refine the paths, and generate actions for each agent
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{

    int current_time=env->curr_timestep;
    bool flag= decide_when_to_plan(current_time,RHCR_h);
    if (flag== true){
        cout<<"true"<<endl;
    }
    else{
        cout<<"false"<<endl;
    }
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    if (flag==true){
        //cout<<env->num_of_agents<<endl;
        //for (int i = 0; i < env->num_of_agents; ++i) {
        //cout<<priority_order[i]<<endl;
        //}
        std::vector<int> vec_data;
        for (int i = 0; i < env->num_of_agents; ++i)
            vec_data.push_back(priority_order[i]);
        MAPFPlanner::index = argsort(vec_data);
        //for (int item : index)
            //std::cout << item << endl;
        //for (int i = 0; i < env->num_of_agents; ++i) {
        //cout<<MAPFPlanner::priority_order[i]<<endl;
        //}
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int current_agent=index[i];
            int current_orientation=env->curr_states[current_agent].orientation;
            int current_position=env->curr_states[current_agent].location;
            int goal_location=env->goal_locations[current_agent].front().first;
            int current_map_h=-1;
            int current_map_w=-1;
            cout<<current_agent<<endl;
            cout<<"Begin the planning of an agent"<<endl;
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
                path=single_agent_plan_SIPP(env->curr_states[current_agent].location,
                                       env->curr_states[current_agent].orientation,
                                       env->goal_locations[current_agent].front().first,safe_intervals);
                agents_path[current_agent]=path;
                if (agents_path[current_agent][0].first != agents_path[current_agent][1].first){
                    actions[current_agent] = Action::FW; //forward action
                }
                else if (agents_path[current_agent][1].second!= agents_path[current_agent][0].second)
                {
                int incr = agents_path[current_agent][1].second - agents_path[current_agent][0].second;
                if (incr == 1 || incr == -3)
                {
                   actions[current_agent] = Action::CR; //C--counter clockwise rotate
                  }
                 else if (incr == -1 || incr == 3)
                 {
                     actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
                }
            }
            //if (path.front().first != env->curr_states[current_agent].location)
            //{
                //actions[current_agent] = Action::FW; //forward action
            }
            //else if (path.front().second!= env->curr_states[current_agent].orientation)
            //{
               // int incr = path.front().second - env->curr_states[current_agent].orientation;
                //if (incr == 1 || incr == -3)
                //{
                //    actions[current_agent] = Action::CR; //C--counter clockwise rotate
              //  }
               // else if (incr == -1 || incr == 3)
               // {
               //     actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                //}
            //}
            //agents_path[current_agent].erase(agents_path[current_agent].begin());

        }
    //}
    else{
        for (int i = 0; i < env->num_of_agents; i++) {
            int current_agent=index[i];
            int current_timestep = env->curr_timestep % RHCR_h;
            if (agents_path[current_agent][current_timestep].first != agents_path[current_agent][current_timestep+1].first) {
                actions[current_agent] = Action::FW; //forward action
            } else if (agents_path[current_agent][current_timestep+1].second != agents_path[current_agent][current_timestep].second) {
                int incr = agents_path[current_agent][current_timestep+1].second - agents_path[current_agent][current_timestep].second;
                if (incr == 1 || incr == -3) {
                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                } else if (incr == -1 || incr == 3) {
                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
            }
        }
    }
  return;
}

/**
 * @brief retrieve the valid neighbors of the current vertex
 * @param location The current vertex
 * @param last_move_pos
 * @param safe_intervals The safe intervals of all vertices
 * @param end The goal vertex
 * @return The valid neighbors of the current vertex
*/
vector<SIPPNode> MAPFPlanner::SIPP_get_neighbor(SIPPNode* sipp_node,vector<pair<int,int>>* last_move_pos,vector<pair<int,int>>* safe_intervals,int end){
    map=env->map;
    int move_time=sipp_node->safe_interval.second-sipp_node->arrive_time;
    int maximum_rot_time=move_time-1;
    int location=sipp_node->location;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int minimum_rot_step[4]={0,0,0,0};
    vector<SIPPNode*> SIPPNode_list;
    vector<SIPPNode> SIPPNode_list_object;
    SIPPNode* parent_node=sipp_node;
    SIPPNode parent_object=*parent_node;
    cout<<parent_node->location<<endl;
    for (int i=0;i<4;i++){
        cout<<parent_node->location<<endl;
        int check_candidate=candidates[i];
        if (sipp_node->arrive_dir==i){
            minimum_rot_step[i]=0;
        }
        else{
            if (abs(sipp_node->arrive_dir-i)>4-abs(i-sipp_node->arrive_dir)){
                minimum_rot_step[i]=4-abs(i-sipp_node->arrive_dir);
            }
            else{
                minimum_rot_step[i]=abs(sipp_node->arrive_dir-i);
            }
        }
        if (check_candidate<0 or check_candidate>=env->map.size()){
            continue;
        }
        if (minimum_rot_step[i]>maximum_rot_time or map[check_candidate]==1){
            continue;
        }
        //cout<<minimum_rot_step[i]<<endl;
        int minimum_leave_time=sipp_node->arrive_time+minimum_rot_step[i];
        //cout<<parent_object.location<<endl;
        vector<pair<int,int>> candidate_intervals=safe_intervals[check_candidate];

        int intervals_num=candidate_intervals.size();
        //cout<<parent_node->location<<endl;
        //cout<<"phase 1"<<endl;
        for (int j=0;j<intervals_num;j++){
            int maximum_leave_time=sipp_node->safe_interval.second;
            pair<int,int> current_interval=candidate_intervals[j];
            if (current_interval.first>maximum_leave_time+1){
                continue;
            }
            else if (current_interval.second<minimum_leave_time+1){
                continue;
            }
            int t= max(minimum_leave_time+1,current_interval.first);

            SIPPNode new_node=SIPPNode(t, current_interval, t+getManhattanDistance(check_candidate, end), t, getManhattanDistance(check_candidate, end),
                                       -1,check_candidate,i,last_move_pos[check_candidate][j].second);
            cout<<parent_node->location<<endl;
            SIPPNode_list_object.push_back(new_node);
            //SIPPNode_list.push_back(&new_node);
            cout<<parent_node->location<<endl;
        }
    }
    return SIPPNode_list_object;
}

vector<pair<int,int>> MAPFPlanner::single_agent_plan_SIPP(int start, int start_direct, int end,vector<pair<int,int>>* safe_intervals) {
    int start_time=env->curr_timestep;
    vector<pair<int,int>> path;
    vector<pair<int,int>> current_interval=safe_intervals[start];
    int rtn_index;
    pair<int,int> current_safe_interval= compute_current_interval(current_interval,start_time,&rtn_index);
    int maximum_timestep=100000;
    priority_queue<SIPPNode,vector<SIPPNode>,SIPP_cmp> open_list;
    vector<SIPPNode*> current_reference_list;
    vector<SIPPNode> SIPP_node_list_copy;
    vector<pair<int,int>> agent_interpolate_path;
    unordered_map<int,SIPPNode*> all_nodes;
    unordered_set<int> close_list;
    unordered_set<int> all_nodes_set;
    SIPPNode initial_node=SIPPNode(start_time+0, current_safe_interval, getManhattanDistance(start, end), 0, getManhattanDistance(start, end),
                                   -1,start,start_direct,last_move_pos[start][0].second) ;
    open_list.push(initial_node);
    all_nodes[maximum_timestep*0+4*start+start_direct]=&initial_node;
    close_list.insert(maximum_timestep*0+4*start+start_direct);
    all_nodes_set.insert(maximum_timestep*0+4*start+start_direct);
    int insert_key=0;
    //cout<<"["<<current_safe_interval.first<<","<<current_safe_interval.second<<"]"<<endl;
    bool terminate_flag= false;
    cout<<"Planning new agent"<<endl;
    SIPPNode* current_node;
    while (open_list.size()>0){
        if (terminate_flag== true){
            break;
        }
        SIPPNode curr_node = open_list.top();
        SIPPNode* curr=&curr_node;
        current_reference_list.push_back(curr);
        open_list.pop();
        SIPPNode parent_object=*curr;
        SIPP_node_list_copy.push_back(parent_object);
        if (curr->location==end){
            terminate_flag= true;
            //cout<<end<<endl;

            //cout<<current_reference_list.size()<<endl;
            //for (int k=0;k<SIPP_node_list_copy.size();k++){
                //cout<<SIPP_node_list_copy[k].location<<endl;
            //}
            vector<SIPPNode> current_path_SIPP_node;
            int count=0;
            //cout<<"location: "<<(curr)->location<<endl;
            //cout<<"Arrive time: "<<(curr)->arrive_time<<endl;
            current_path_SIPP_node.push_back(parent_object);
            //cout<<count<<endl;
            while (curr->parent!= -1){
                current_path_SIPP_node.push_back(SIPP_node_list_copy[curr->parent]);
                curr=&SIPP_node_list_copy[curr->parent];
                //cout<<"location: "<<(curr)->location<<endl;
                //cout<<"Arrive time: "<<(curr)->arrive_time<<endl;
                count=count+1;
                //cout<<count<<endl;
            }
            std::reverse(current_path_SIPP_node.begin(), current_path_SIPP_node.end());
            for (int r=0;r<current_path_SIPP_node.size()-1;r++){
                int current_pos=current_path_SIPP_node[r].location;
                int current_dir=current_path_SIPP_node[r].arrive_dir;
                int current_time=current_path_SIPP_node[r].arrive_time;
                int next_pos=current_path_SIPP_node[r+1].location;
                int next_dir=current_path_SIPP_node[r+1].arrive_dir;
                int next_time=current_path_SIPP_node[r+1].arrive_time;
                int direction=0;
                int rotation_time=0;
                int wait_time=0;
                if (abs(next_dir-current_dir)<4-abs(next_dir-current_dir)){
                    direction=0;
                    rotation_time=abs(next_dir-current_dir);
                    wait_time=next_time-current_time-1-rotation_time;
                }
                else {
                    direction=1;
                    rotation_time=4-abs(next_dir-current_dir);
                    wait_time=next_time-current_time-1-rotation_time;
                }
                agent_interpolate_path.emplace_back(current_pos,current_dir);
                for (int z=0;z<wait_time;z++){
                    agent_interpolate_path.emplace_back(current_pos,current_dir);
                }
                for (int z=0;z<rotation_time;z++){
                    if (direction==0){
                        current_dir=current_dir+1;
                    }
                    else{
                        current_dir=current_dir-1;
                    }
                    if (current_dir<0){
                        current_dir=3;
                    }
                    else if (current_dir>3){
                        current_dir=0;
                    }
                    agent_interpolate_path.emplace_back(current_pos,current_dir);
                }
            }
            agent_interpolate_path.emplace_back(current_path_SIPP_node[current_path_SIPP_node.size()-1].location,current_path_SIPP_node[current_path_SIPP_node.size()-1].arrive_dir);
            for (int z=0;z<agent_interpolate_path.size();z++){
                cout<<"location: "<<agent_interpolate_path[z].first<<endl;
                cout<<"direction: "<<agent_interpolate_path[z].second<<endl;
            }
            SIPP_update_safe_intervals(agent_interpolate_path);
            //cout<<curr->parent->parent->location<<endl;
            //cout<<"location: "<<curr->parent->location<<endl;
            //cout<<"location: "<<curr->parent->arrive_time<<endl;
            //cout<<"location: "<<curr->parent->parent->location<<endl;
            //cout<b<"location: "<<curr->parent->parent->arrive_time<<endl;
            current_node=curr;
            break;
        }
        map=env->map;
        SIPPNode* sipp_node=curr;
        int move_time=sipp_node->safe_interval.second-sipp_node->arrive_time;
        int maximum_rot_time=move_time-1;
        int location=sipp_node->location;
        int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
        int minimum_rot_step[4]={0,0,0,0};
        int objects_num=SIPP_node_list_copy.size();
        //for (int k=0;k<SIPP_node_list_copy.size();k++){
            //cout<<SIPP_node_list_copy[k].location<<endl;
        //}
        //cout<<parent_object.f<<endl;
        //cout<<parent_object.g<<endl;
        //cout<<parent_object.h<<endl;

        //cout<<parent_node->location<<endl;
        for (int i=0;i<4;i++){
            int check_candidate=candidates[i];
            if (sipp_node->arrive_dir==i){
                minimum_rot_step[i]=0;
            }
            else{
                if (abs(sipp_node->arrive_dir-i)>4-abs(i-sipp_node->arrive_dir)){
                    minimum_rot_step[i]=4-abs(i-sipp_node->arrive_dir);
                }
                else{
                    minimum_rot_step[i]=abs(sipp_node->arrive_dir-i);
                }
            }
            if (check_candidate<0 or check_candidate>=env->map.size()){
                continue;
            }
            if (minimum_rot_step[i]>maximum_rot_time or map[check_candidate]==1){
                continue;
            }
            //cout<<minimum_rot_step[i]<<endl;
            int minimum_leave_time=sipp_node->arrive_time+minimum_rot_step[i];
            //cout<<parent_object.location<<endl;
            vector<pair<int,int>> candidate_intervals=safe_intervals[check_candidate];

            int intervals_num=candidate_intervals.size();
            int maximum_leave_time=sipp_node->safe_interval.second;
            if (curr->current_interval_next_pos!=-1 and check_candidate==curr->current_interval_next_pos){
                maximum_leave_time=maximum_leave_time-1;
            }

            //cout<<curr->location<<endl;
            //cout<<"phase 1"<<endl;
            for (int j=0;j<intervals_num;j++){
                pair<int,int> current_interval=candidate_intervals[j];
                if (current_interval.first>maximum_leave_time+1){
                    continue;
                }
                else if (current_interval.second<minimum_leave_time+1){
                    continue;
                }
                int t= max(minimum_leave_time+1,current_interval.first);
                //cout<<t<<endl;
                //cout<<curr->location<<endl;
                //cout<<objects_num-1<<endl;
                SIPPNode new_node=SIPPNode(t, current_interval, t+getManhattanDistance(check_candidate, end), t, getManhattanDistance(check_candidate, end),
                                           objects_num-1,check_candidate,i,last_move_pos[check_candidate][j].second);
                //if (new_node.g<50){
                    //cout<<new_node.f<<endl;
                    //cout<<new_node.g<<endl;
                    //cout<<new_node.h<<endl;
                //}
                //cout<<new_node.parent->location<<endl;
                //cout<<check_candidate<<endl;
                //cout<<curr->location<<endl;
                insert_key=j*maximum_timestep+4*new_node.location+new_node.arrive_dir;
                auto found = close_list.find(insert_key);
                if (found != close_list.end()) {

                } else {
                    auto found = all_nodes_set.find(insert_key);
                    if (found ==all_nodes_set.end()) {
                        all_nodes_set.insert(insert_key);
                        all_nodes[insert_key]=&new_node;
                        open_list.push(new_node);
                        close_list.insert(insert_key);
                    }
                    else{
                        SIPPNode* exist_node=all_nodes[insert_key];
                        if (exist_node->arrive_time>new_node.arrive_time){
                            exist_node->arrive_time=new_node.arrive_time;
                            exist_node->safe_interval=new_node.safe_interval;
                            exist_node->parent=new_node.parent;
                            exist_node->current_interval_next_pos=new_node.current_interval_next_pos;
                            exist_node->f=new_node.f;
                            exist_node->g=new_node.g;
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
    //for (int i=0;i<current_interval.size();i++) {
        //cout<<"["<<current_interval.front().first<<","<<current_interval.front().second<<"]"<<endl;
    //}
    return path;
}
vector<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
{
    cout<<"Begin the A star algorithm"<<endl;
    vector<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                if (env->map[curr->location]==1){
                    cout<<"something is wrong !"<<endl;
                }
                path.insert(path.begin(),make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int loc, int loc2)
{
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


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
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
void MAPFPlanner::SIPP_update_safe_intervals(vector<pair<int, int>> agent_planned_path){
    vector<int> last_position_list;
    last_position_list.push_back(-1);
    for (int i = 0; i < RHCR_w; i++) {
        int location;
        if (i==agent_planned_path.size()){
            break;
        }
        location = agent_planned_path[i].first;
        if (location!=last_position_list[last_position_list.size()-1]){
            last_position_list.push_back(location);
        }
        int last_position=last_position_list[last_position_list.size()-2];
        int current_time_step=env->curr_timestep+i;
        int rtn_index;

        //cout<<current_time_step<<endl;
        //cout<<location<<endl;
        //for (int r=0;r<safe_intervals[location].size();r++){
            //cout<<safe_intervals[location][r].first<<","<<safe_intervals[location][r].second<<endl;
        //}
        compute_current_interval(safe_intervals[location],current_time_step,&rtn_index);
        if (current_time_step==safe_intervals[location][rtn_index].first){
            safe_intervals[location][rtn_index].first=safe_intervals[location][rtn_index].first+1;
            last_move_pos[location][rtn_index].first=last_position;
            if (safe_intervals[location][rtn_index].first>safe_intervals[location][rtn_index].second){
                safe_intervals[location].erase(safe_intervals[location].begin()+rtn_index);
                last_move_pos[location].erase(last_move_pos[location].begin()+rtn_index);
            }
        }
        else if (current_time_step==safe_intervals[location][rtn_index].second){
            safe_intervals[location][rtn_index].second=safe_intervals[location][rtn_index].second-1;
            last_move_pos[location][rtn_index].second=last_position;
            if (safe_intervals[location][rtn_index].first>safe_intervals[location][rtn_index].second){
                safe_intervals[location].erase(safe_intervals[location].begin()+rtn_index);
                last_move_pos[location].erase(last_move_pos[location].begin()+rtn_index);
            }
        }
        else{
            pair<int, int> new_interval_1 = make_pair(safe_intervals[location][rtn_index].first, current_time_step-1);
            pair<int, int> new_interval_2 = make_pair(current_time_step + 1, safe_intervals[location][rtn_index].second);
            pair<int, int> last_pos_interval_1 = make_pair(-1, last_position);
            pair<int, int> last_pos_interval_2 = make_pair(last_position, -1);
            safe_intervals[location].erase(safe_intervals[location].begin()+rtn_index);
            int sum_count=0;
            if (new_interval_1.second> new_interval_1.first){
                safe_intervals[location].insert(safe_intervals[location].begin()+rtn_index+sum_count,new_interval_1);
                last_move_pos[location].insert(last_move_pos[location].begin()+rtn_index+sum_count,last_pos_interval_1);
                sum_count=sum_count+1;
            }
            if (new_interval_2.second> new_interval_2.first){
                safe_intervals[location].insert(safe_intervals[location].begin()+rtn_index+sum_count,new_interval_2);
                last_move_pos[location].insert(last_move_pos[location].begin()+rtn_index+sum_count,last_pos_interval_2);
            }
        }
    }
}