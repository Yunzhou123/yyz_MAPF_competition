#include "MDD.h"
#include <iostream>

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
        // TODO: need to discuss the type of location
        if (parent->location.first == goalNode->location.first)
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