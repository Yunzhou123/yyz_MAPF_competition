#include "MDD.h"

/// @brief traverse the levels and print the MDDNodes
void MDD::printNodes() const
{
    for (const auto& level : levels) 
    {
        cout << "[";
        for (const auto& node : level) 
        {
            cout << node->location << ",";
        }
        cout << "]," << endl;
    }
}

bool MDD::buildMDD(pair<int,int> startLoc, int numOfLevels)
{
    auto root = new MDDNode(startLoc, nullptr);
    root->cost = numOfLevels - 1;
    std::queue<MDDNode*> open;
    list<MDDNode*> closed;
    open.push(root);
    closed.push_back(root);
    levels.resize(numOfLevels);

    while(!open.empty())
    {
        auto curr = open.front();
        open.pop();

        // assuming eachh edge costs 1
        if (curr->level == numOfLevels - 1)
        {
            levels.back().push_back(curr);
            assert(open.empty());
            break;
        }

        // We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g - 2. -1 because it's the bound of the children.
        int heuristicBound = numOfLevels - curr->level - 2;
        // getNeighbors already checks if the location is valid
        list<pair<int,int>> nextLocations = mapf::MAPFPlanner::getNeighbors(startLoc, direction);
        for (const auto& nextLoc : nextLocations)
        {
            // TODO: for each agent, add a datastrcuture in MAPF.h to store heuristic values for each location
            if (heiristic[nextLoc] <= heuristicBound)
            {
                bool find = false;
                for (auto child = closed.rbegin(); child != closed.rend() && (*child)->level == curr->level + 1; ++child)
                {
                    // TODO: ensure the type of location and nextLoc; 
                    if ((*child)->location == nextLoc) // if found, add correpsonding parent link and child link
                    {
                        (*child)->parents.push_back(curr);
                        find = true;
                        break;
                    }

                    if (!find) // if not found, create a new node
                    {
                        auto childNode = new MDDNode(nextLoc, curr);
                        childNode->cost = numOfLevels - 1;
                        open.push(childNode);
                        closed.push_back(childNode);
                    }
                }
            }
        }
    }

    // Backward
    auto goalNode = levels.back().back();
    MDDNode* del = nullptr;
    for (auto parent : goalNode->parents)
    {
        if (parent->location == goalNode->location)
        {
            del = parent;
            continue;
        }
        levels[numOfLevels - 2].push_back(parent);
        parent->children.push_back(goalNode);
    }
    if (del != nullptr)
    {
        goalNode->parents.remove(del);
    }
    for (int t = numOfLevels - 2; t > 0; --t)
    {
        for (auto node : levels[t])
        {
            for (auto parent : node->parents)
            {
                if (parent->children.empty()) // a new node
                    levels[t - 1].push_back(parent);

                parent->children.push_back(node); // add forward edge
            }
        }
    }

    // delete useless nodes (have no children)
    for (auto it : closed)
    {
        if (it->children.empty() && it->level < numOfLevels - 1)
            delete it;
    }
    closed.clear();
    return true;
}

void MDD::deleteNode(MDDNode* node)
{
    levels[node->level].remove(node);
    // clear the children
    for (auto child = node->children.begin(); child != node->children.end(); ++child)
    {
        (*child)->parents.remove(node);
        if ((*child)->parents.empty())
            deleteNode(*child);
    }
    // clear the parents
    for (auto parent = node->parents.begin(); parent != node->parents.end(); ++parent)
    {
        (*parent)->children.remove(node);
        if ((*parent)->children.empty())
            deleteNode(*parent);
    }
}

void MDD::clear()
{
    if (levels.empty())
        return;
    
    for (auto& level : levels)
    {
        for (auto& node : level)
        {
            deleteNode(node);
        }
        level.clear();
    }
}

MDDNode* MDD::find(pair<int,int> loc, int level) const
{
    if (level > 0 && level < levels.size())
    {
        for (auto node : levels[level])
        {
            if (node->location == loc.first && node->direction == loc.second)
                return node;
        }
    }
    return nullptr;
}

MDD::MDD(const MDD& cpy)
{
    levels.resize(cpy.levels.size());
    auto root = new MDDNode(cpy.levels[0].front()->location, nullptr);
    root->cost = cpy.levels.size() - 1;
    levels[0].push_back(root);

    for (size_t t = 0; t < levels.size() - 1; t++)
    {
        for (auto node = levels[t].begin(); node != levels[t].end(); ++node)
        {
            MDDNode* cpyNode = cpy.find(make_pair((*node)->location, (*node)->direction), t);
            for (auto cpyChild : cpyNode->children)
            {
                MDDNode* child = find(make_pair(cpyChild->location, cpyChild->direction), t + 1);
                if (child == nullptr)
                {
                    child = new MDDNode(make_pair(cpyChild->location, cpyChild->direction), *node);
                    child->cost = cpyChild->cost;
                    levels[child->level].push_back(child);
                    (*node)->children.push_back(child);
                }
                else
                {
                    (*node)->children.push_back(child);
                    child->parents.push_back(*node);
                }
            }
        }
    }
}

MDD::~MDD()
{
    clear();
}

void MDD::increaseBy(const vector<vector<vector<int>>>* constraints, int dLevel)
{
    auto oldHeight = levels.size();
    auto numOfLevels = oldHeight + dLevel;

    for (auto& level : levels)
    {
        for (auto& node : level)
        {
            node->parents.clear();
        }
    }

    levels.resize(numOfLevels);

    for (int l = 0; l < numOfLevels; ++l)
    {
        double heuristicBound = numOfLevels - l - 2 + 0.001;
        auto node_map = collectMDDlevel(this, l + 1);

        for (auto& it: levels[l])
        {
            MDDNode* node_ptr = it;
            list<pair<int,int>> next_locations = mapf::MAPFPlanner::getNeighbors(node_ptr->location, node_ptr->direction);
            for (auto next_loc : next_locations)
            {
                if (heuristic[next_loc] <= heuristicBound)
                {
                    if (node_map.find(next_loc) == node_map.end())
                    {
                        auto child = new MDDNode(next_loc, node_ptr);
                        levels[l + 1].push_back(child);
                        node_map[next_loc] = child;
                    }
                    else
                    {
                        node_map[next_loc]->parents.push_back(node_ptr);
                    }
                }
                    
            }
        }
    }

    // backward
    for (int l = oldHeight; l < numOfLevels; ++l)
    {
        MDDNode* goalNode = nullptr;
        for (auto node : levels[l])
        {
            // TODO: check if the type of goal is correct
            if (node->location == solver->goal_location)
            {
                goalNode = node;
                break;
            }
        }

        std::queue<MDDNode*> bfs_q({goalNode});
        boost::unordered_set<MDDNode*> closed;

        while (!bfs_q.empty())
        {
            auto curr = bfs_q.front();
            bfs_q.pop();
            curr->cost = l;

            for (auto parent : curr->parents)
            {
                parent->children.push_back(curr); // add forward edge
                if (closed.find(parent) == closed.end() && parent->cost == 0)
                {
                    bfs_q.push(parent);
                    closed.insert(parent);
                }
            }
        }
    }

    // clear useless nodes
    for (int l = 0; l < numOfLevels - 1; l++)
    {
        auto it = levels[l].begin();
        while (it != levels[l].end())
        {
            if ((*it)->children.empty())
            {
                it = levels[l].erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
}

MDDNode* MDD::goalAt(int level)
{
    if (level >= levels.size()) {return nullptr;}

    for (MDDNode* ptr: lvels[level])
    {
        // TODO: need to check if the type of goal is correct
        if (ptr->location == solver->goal_location)
        {
            return ptr;
        }
    }
    return nullptr;
}

std::ostream& operator<<(std::ostream& os, const MDD& mdd)
{
    for (const auto& level : mdd.levels) 
    {
        cout << "L" << level.front()->level << ":";
        for (const auto& node : level) 
        {
            cout << node->location << " " << node->direction << ",";
        }
        cout << endl;
    }
    return os;
}

MDD* MDDTable::getMDD(CBS_node& node, int agentId, size_t MDDLevels)
{
    ConstraintsHasher c(id, &node);
    auto got = lookupTable[agentID].find(c);
    if (got != lookupTable[c.a].end())
    {
        assert(got->second->levels.size() == MDDLevels);
        return got->second;
    }
    releaseMDDMemory(id);

    clock_t t = clock();
    MDD* mdd = new MDD();
    // TODO: define ConstraintTable
    ConstraintTable ct(initialConstraints[agentID])
    ct.build(node, agentID);
    mdd->buildMDD(ct, mddLevels);
    if (!lookupTable.empty())
    {
        lookupTable[c.a][c] = mdd;
    }
    accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
    return mdd;
}

double MDDTable::getAverageWidth(CBS_node& node, int agentId, size_t MDDLevels)
{
    auto mdd = getMDD(node, agentId, MDDLevels);
    double width = 0;
    for(const auto& level : mdd->levels)
    {
        width += level.size();
    }
    return width / mdd->levels.size();
}

void MDDTable::findSingletons(CBS_node& node, int agentId, vector<pair<int, int>>& agent_planned_path)
{
    auto mdd = getMDD(node, agentId, agent_planned_path.size());
    for (size_t t = 0; t < mdd->levels.size(); t++)
    {
        agent_planned_path[t] = mdd->levels[t].size();
    }
    if (lookupTable.empty())
    {
        delete mdd;
    }
}

void MDDTable::releaseMDDMemory(int id)
{
    // check if input is valid
    if (id < 0 || lookupTable.empty() || (int) lookupTable[id].size() < maxNumOfMDDs)
    {
        return;
    }
    // TODO: 
    int minLength = MAX_TIMESTEP; // MAX_TIMESTEP = INT_MAX / 2
    for (auto mdd: lookupTable[id])
    {
        if ((int) mdd.second->levels.size() < minLength)
        {
            minLength = mdd.second->levels.size();
        }
    }

    for (auto mdd = lookupTable[id].begin(); mdd != lookupTable[id].end();)
    {
        if ((int) mdd->second->levels.size() == minLength)
        {
            delete mdd->second;
            mdd = lookupTable[id].erase(mdd);
            numReleaseMDDS++;
        }
        else
        {
            ++mdd;
        }
    }
}

void MDDTable::clear()
{
    for (auto& mdds : lookupTable)
    {
        for (auto& mdd : mdds)
        {
            delete mdd.second;
        }
    }
    lookupTable.clear();
}

unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i)
{
    unordered_map<int, MDDNode*> loc2mdd;
    for (MDDNode* it_0 : mdd->levels[i])
    {
        int loc = it_0->location;
        loc2mdd[loc] = it_0;
    }
    return loc2mdd;
}
