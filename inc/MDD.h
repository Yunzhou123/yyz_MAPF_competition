#pragma once
#include "CBSNode.h"

class MDDNode 
{
public:
        MDDNode(pair<int,int> currloc, MDDNode* parent) 
        {
            location = currloc;
            if (parent != nullptr) {
                level = parent->level + 1;
                parents.push_back(parent);
            } else {
                level = 0;
            }
        }

        int location;
        int direction;
        int level;
        int cost = 0; // cost to reach this node

        list<MDDNode*> parents;
        list<MDDNode*> children;
};

class MDD 
{
private:

public:
        vector<list<MDDNode*> > levels;

        // TODO: discuss whether we need to express constraints with a different data structure
        bool buildMDD(const vector<vector<vector<int>>>* constraints, int numOfLevels); // construct  the MDD based on consrtaints
        
        void printNodes() const; // print the nodes of the MDD for debugging
        void deleteNode(MDDNode* node); // delete a specified node from the MDD, adjusting the parent-child
        void clear(); // clear all nodes
        void increaseBy(const vector<vector<vector<int>>>* constraints, int dLevel); // expands the MDD by a specified number of levels, adjusting for new constraints
        MDDNode* goalAt(int level); // return the node representing the goal at a specified level

        MDDNode* find(int loc, int level) const; // find a node with a specified location and level

        MDD(const MDD& cpy);
        MDD() = default;
        ~MDD();
}

std::ostream& operator<<(std::ostream& os, const MDD& mdd);

// TODO: discuss whether we need SyncNode class

class MDDTable 
{

private:
        int maxNumOfMDDs = 10000; // TODO: TBD
        vector<unordered_map<ConstraintsHasher, MDD*, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>> lookupTable; // TODO: TBD
        
        const vector< vector< vector< vector<int> > > >& initialConstraints;
        
        void releaseMDDMemory(int id);

public:
        double accumulatedRuntime = 0;
        uint64_t numReleaseMDDS = 0;

        MDDTalbe(const vector< vector< vector< vector<int> > > >& initialConstraints): initialConstraints(initialConstraints) {}
        void init(int numOfAgents) { lookupTable.resize(numOfAgents); }

        ~MDDTable() { clear(); }

        MDD* getMDD(CBS_node& node, int agentId, size_t MDDLevels);
        void findSingletons(CBS_node& node, int agentId, vector<pair<int, int>>& agent_planned_path);
        double getAverageWidth(CBS_node& node, int agentId, size_t MDDLevels);
        void releaseMDDMemory(int id);
};
