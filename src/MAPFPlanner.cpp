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
struct SIPPNode {
    int arrive_time;
    pair<int,int> safe_interval;
    int f,g,h;
    SIPPNode* parent;
};

struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};
int a=1+1;
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
    for (int i = 0; i < env->num_of_agents; ++i) {
    cout<<priority_order[i]<<endl;
    }
    for (int i = 0; i < env->rows*env->cols; ++i) {
        pair<int, int> initial_interval;
        initial_interval.first=0;
        initial_interval.second=100000;
        safe_intervals[i].push_back(initial_interval);
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
void MAPFPlanner::map_index_to_vec_index(int map_h, int map_w,int& vec_index)
{
    vec_index=map_h*env->cols+map_w;
}
void MAPFPlanner::vec_index_to_map_index(int& map_h, int& map_w,int vec_index)
{
    map_h=int(vec_index/env->cols);
    map_w=vec_index-map_h*env->cols;
}
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
// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{

    int current_time=env->curr_timestep;
    bool flag= decide_when_to_plan(current_time,RHCR_h);
    cout<<flag<<endl;
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
            int current_map_w=-1;            cout<<env->curr_states[current_agent].location<<endl;
            cout<<env->curr_states[current_agent].orientation<<endl;
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
                path = single_agent_plan(env->curr_states[current_agent].location,
                                         env->curr_states[current_agent].orientation,
                                         env->goal_locations[current_agent].front().first);
                single_agent_plan_SIPP(env->curr_states[current_agent].location,
                                       env->curr_states[current_agent].orientation,
                                       env->goal_locations[current_agent].front().first,safe_intervals);
                agents_path[current_agent]=path;
            }
            if (path.front().first != env->curr_states[current_agent].location)
            {
                actions[current_agent] = Action::FW; //forward action
            }
            else if (path.front().second!= env->curr_states[current_agent].orientation)
            {
                int incr = path.front().second - env->curr_states[current_agent].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                }
                else if (incr == -1 || incr == 3)
                {
                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
            }
            agents_path[current_agent].erase(agents_path[current_agent].begin());

        }
    }
    else{
        for (int i = 0; i < env->num_of_agents; i++){
            int current_agent=index[i];
            //cout<<agents_path[current_agent].size()<<endl;
            if (agents_path[current_agent].size()==0){
                agents_path[current_agent].push_back({env->curr_states[current_agent].location, env->curr_states[current_agent].orientation});
            }
            //cout<<agents_path[current_agent].front().first<<endl;
            //cout<<env->curr_states[current_agent].location<<endl;
            //cout<<env->curr_states[current_agent].orientation<<endl;
            if (agents_path[current_agent].front().first != env->curr_states[current_agent].location)
            {
                actions[current_agent] = Action::FW; //forward action
            }
            else if (agents_path[current_agent].front().second!= env->curr_states[current_agent].orientation)
            {
                int incr = agents_path[current_agent].front().second - env->curr_states[current_agent].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[current_agent] = Action::CR; //C--counter clockwise rotate
                }
                else if (incr == -1 || incr == 3)
                {
                    actions[current_agent] = Action::CCR; //CCR--clockwise rotate
                }
            }
            agents_path[current_agent].erase(agents_path[current_agent].begin());

        }
    }
  return;
}
vector<pair<int,int>> MAPFPlanner::single_agent_plan_SIPP(int start, int start_direct, int end,vector<pair<int,int>>* safe_intervals) {
    int start_time=env->curr_timestep;
    vector<pair<int,int>> path;
    vector<pair<int,int>> current_interval=safe_intervals[start];
    pair<int,int> current_safe_interval= compute_current_interval(current_interval,start_time);
    cout<<"["<<current_safe_interval.first<<","<<current_safe_interval.second<<"]"<<endl;
    for (int i=0;i<current_interval.size();i++) {
        cout<<"["<<current_interval.front().first<<","<<current_interval.front().second<<"]"<<endl;
    }
    return path;
}
vector<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
{
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
