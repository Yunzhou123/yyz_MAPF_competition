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
pair<int,int> MAPFPlanner::compute_current_interval(vector<pair<int,int>> current_safe_intervals,int current_time){
    int intervals_num=current_safe_intervals.size();
    int low_index=0;
    int high_index=intervals_num-1;
    int check_index=int((low_index+high_index)/2);
    bool flag= false;
    pair<int,int> rtn_interval=make_pair(0,0);
    while (flag== false){
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
                    low_index=high_index;
                }
                else{
                    check_index=low_index;
                }
                check_index=int((low_index+high_index)/2);
            }
        }
    }
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

/// plan using simple A* that ignores the time dimension
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
                single_agent_plan_SIPP(env->curr_states[current_agent].location,
                                       env->curr_states[current_agent].orientation,
                                       env->goal_locations[current_agent].front().first,safe_intervals);
                agents_path[current_agent]=path;
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
        //for (int i = 0; i < env->num_of_agents; i++){
         //   int current_agent=index[i];
         //   //cout<<agents_path[current_agent].size()<<endl;
          //  if (agents_path[current_agent].size()==0){
          //      agents_path[current_agent].push_back({env->curr_states[current_agent].location, env->curr_states[current_agent].orientation});
          //  }
            //cout<<agents_path[current_agent].front().first<<endl;
            //cout<<env->curr_states[current_agent].location<<endl;
            //cout<<env->curr_states[current_agent].orientation<<endl;
       //     if (agents_path[current_agent].front().first != env->curr_states[current_agent].location)
        //    {
         //       actions[current_agent] = Action::FW; //forward action
         //   }
         //   else if (agents_path[current_agent].front().second!= env->curr_states[current_agent].orientation)
         //   {
         //       int incr = agents_path[current_agent].front().second - env->curr_states[current_agent].orientation;
         //       if (incr == 1 || incr == -3)
         //       {
          //          actions[current_agent] = Action::CR; //C--counter clockwise rotate
          //      }
           //     else if (incr == -1 || incr == 3)
           //     {
            //        actions[current_agent] = Action::CCR; //CCR--clockwise rotate
            //    }
           // }
           // agents_path[current_agent].erase(agents_path[current_agent].begin());

       // }
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
vector<SIPPNode> MAPFPlanner::SIPP_get_neighbor(SIPPNode* sipp_node, vector<pair<int,int>>* last_move_pos, vector<pair<int,int>>* safe_intervals, int end){
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
        cout<<parent_object.location<<endl;
        vector<pair<int,int>> candidate_intervals=safe_intervals[check_candidate];

        int intervals_num=candidate_intervals.size();
        cout<<parent_node->location<<endl;
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
                                       nullptr,check_candidate,i);
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
    pair<int,int> current_safe_interval= compute_current_interval(current_interval,start_time);
    int maximum_timestep=100000;
    priority_queue<SIPPNode,vector<SIPPNode>,SIPP_cmp> open_list;
    vector<SIPPNode*> current_reference_list;
    vector<SIPPNode> SIPP_node_list_copy;
    unordered_map<int,SIPPNode*> all_nodes;
    unordered_set<int> close_list;
    SIPPNode initial_node=SIPPNode(0, current_safe_interval, getManhattanDistance(start, end), 0, getManhattanDistance(start, end),
                                   nullptr,start,start_direct) ;
    open_list.push(initial_node);
    close_list.insert(maximum_timestep*0+start);
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
        //curr.arrive_time=curr.g;
        //cout<<"f: "<<curr.f<<endl;
        //cout<<"g: "<<curr.g<<endl;
        //cout<<"h: "<<curr.h<<endl;
        //cout<<"location: "<<curr.location<<endl;
        //cout<<start<<endl;
        //cout<<curr->location<<endl;

        //cout<<curr->arrive_time<<endl;
        //cout<<curr->location<<endl;
        //printf("%d\n",curr->h);
        //cout<<open_list.size()<<endl;
        //cout<<open_list.size()<<endl;
        if (curr->location==end){
            terminate_flag= true;
            //cout<<end<<endl;

            //cout<<current_reference_list.size()<<endl;
            //for (int k=0;k<SIPP_node_list_copy.size();k++){
                //cout<<SIPP_node_list_copy[k].location<<endl;
            //}
            int count=0;
            cout<<"location: "<<(curr)->location<<endl;
            cout<<"Arrive time: "<<(curr)->arrive_time<<endl;
            cout<<count<<endl;
            while (curr->parent!= nullptr){
                curr=curr->parent;
                cout<<"location: "<<(curr)->location<<endl;
                cout<<"Arrive time: "<<(curr)->arrive_time<<endl;
                count=count+1;
                cout<<count<<endl;
            }

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
        SIPPNode* parent_node=sipp_node;
        SIPPNode parent_object=*curr;
        SIPP_node_list_copy.push_back(parent_object);
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
            //cout<<curr->location<<endl;
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
                //cout<<t<<endl;
                //cout<<curr->location<<endl;
                cout<<objects_num-1<<endl;
                SIPPNode new_node=SIPPNode(t, current_interval, t+getManhattanDistance(check_candidate, end), t, getManhattanDistance(check_candidate, end),
                                           &SIPP_node_list_copy[objects_num-1],check_candidate,i);
                //if (new_node.g<50){
                    //cout<<new_node.f<<endl;
                    //cout<<new_node.g<<endl;
                    //cout<<new_node.h<<endl;
                //}
                //cout<<new_node.parent->location<<endl;
                //cout<<check_candidate<<endl;
                //cout<<curr->location<<endl;
                insert_key=new_node.arrive_time*maximum_timestep+new_node.location;
                auto found = close_list.find(insert_key);
                if (found != close_list.end()) {

                } else {
                open_list.push(new_node);
                //cout<<SIPP_list[i].parent->location<<endl;
                close_list.insert(insert_key);
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

void MAPFPlanner::SIPP_update_safe_intervals(const vector<SIPPNode>& planned_path){
    
}