//
// Created by rashi on 7/26/2021.
//

#ifndef DWA_LOCAL_PLANNER_NODE_H
#define DWA_LOCAL_PLANNER_NODE_H

#include <string>
#include <vector>

namespace dwa_local_planner {


struct State{int state; double probability=1.0;};
struct Action{int action;};


struct robot_model{
    State init_state={0};
    int h=1;
    std::vector<Action> actions;
    std::vector<double> reward_list, risk_list;

    double risk(State state, Action action){return risk_list[action.action];}
    double reward(State state, Action action){return reward_list[action.action];}
    std::vector<State> Transition(State state, Action action){return *new std::vector<State>{init_state};}
    std::vector<Action> Actions(State state){return actions;}
    int get_h(){return h;}
    State get_init_state(){return init_state;}

};



struct Node{
    int BFS_idx, depth;
    double risk=0, reward=0;
    int id;
    std::vector<std::vector<double>> T_prob;
    std::vector<double> x_vals;
    std::vector<int> parents_act, parents_trans_idx, z_vals;
    // parents_act: list of act_idx of each parent, parents_trans_idx: list of state_idx of each parent

    State state;

    int best_action;
    std::vector<Node*> parents;
    std::vector<std::vector<Node*>> child_list;
    bool terminal= false;

    void find_best_action(){
        for(int i=0; i<this->z_vals.size(); i++){
            if(this->z_vals[i] == 1){
                this->best_action =  i;
                break;
            }
        }
    }


};


};


#endif //DWA_LOCAL_PLANNER_NODE_H
