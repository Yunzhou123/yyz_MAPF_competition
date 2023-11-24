# pragma once
#include "MAPFPlanner.h"

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

        pair<int,int> location;
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
        bool buildMDD(const vector<vector<vector<int>>>* constraints, int numOfLevels);
        
        void printNodes() const;
        void deleteNode(MDDNode* node);
        void clear();
        void increaseBy(const vector<vector<vector<int>>>* constraints, int dLevel);
        MDDNode* goalAt(int level);

        MDDNode* find(int loc, int level) const;

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

};
