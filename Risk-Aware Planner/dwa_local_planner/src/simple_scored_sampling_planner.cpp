/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#include <dwa_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>


namespace dwa_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list, std::vector<base_local_planner::TrajectoryCostFunction*>& critics, std::vector<base_local_planner::TrajectoryCostFunction*>& risk_critics,
                                                           double cc, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
    risk_critics_ = risk_critics;
    risk_cc = cc;

    if(false){ // disable risk
        for(auto critic: risk_critics_)
            critics_.push_back(critic);

        risk_critics_ = *new std::vector<base_local_planner::TrajectoryCostFunction*>;
    }


  }

  double SimpleScoredSamplingPlanner::scoreTrajectory(base_local_planner::Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost+1000000) { //this is to ignore the constriant
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }


    double SimpleScoredSamplingPlanner::riskTrajectory(base_local_planner::Trajectory& traj) {
        double traj_risk = 0;

    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = risk_critics_.begin(); score_function != risk_critics_.end(); ++score_function) {
      base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      traj_risk =  score_function_p->scoreTrajectory(traj)/254.0;
    }

        return std::max(0.0,traj_risk);
    }













  bool SimpleScoredSamplingPlanner::findBestTrajectory(base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored) {
    /* This function calls the ccssp planner: 1) genrate all trajectories with cost and risk vals, 2) if min risk traj is above risk_cc then return false 
    3) run ccssp planner (not all gurobi errors are hundled 100005)
    */

    base_local_planner::Trajectory loop_traj;
    base_local_planner::Trajectory best_traj;
    std::vector<double> traj_cost_list;
    std::vector<double> traj_risk_list;
      std::vector<base_local_planner::Trajectory> all_traj;

    double loop_traj_cost, loop_traj_risk, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    for (std::vector<base_local_planner::TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      base_local_planner::TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    for (std::vector<base_local_planner::TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      count = 0;
      count_valid = 0;
      base_local_planner::TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) {
          gen_success = gen_->nextTrajectory(loop_traj);
          if (gen_success == false) {
              // TODO use this for debugging
              continue;
          }
          loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
          loop_traj_risk = riskTrajectory(loop_traj);
          if(loop_traj_cost<0){
            continue;
          }
          all_traj.push_back(loop_traj);
          traj_cost_list.push_back(loop_traj_cost);
          traj_risk_list.push_back(loop_traj_risk);

          if (all_explored != NULL) {
              loop_traj.cost_ = loop_traj_cost;
              all_explored->push_back(loop_traj);
          }

          /*
          if (loop_traj_cost >= 0) {
            count_valid++;
            if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
              best_traj_cost = loop_traj_cost;
              best_traj = loop_traj;
            }
          }
          count++;
          if (max_samples_ > 0 && count >= max_samples_) {
            break;
          }
        }
        if (best_traj_cost >= 0) {
          traj.xv_ = best_traj.xv_;
          traj.yv_ = best_traj.yv_;
          traj.thetav_ = best_traj.thetav_;
          traj.cost_ = best_traj_cost;
          traj.resetPoints();
          double px, py, pth;
          for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
            best_traj.getPoint(i, px, py, pth);
            traj.addPoint(px, py, pth);
          }
        }
        ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
        if (best_traj_cost >= 0) {
          // do not try fallback generators
          break;
        */
      } }


      // TODO: Add ccssp planner and best traj is saved in traj ref variable
      robot_model robotModel;

      std::vector<Action> actions;
      for(int i=0; i<all_traj.size(); i++)
          actions.push_back(*new Action{i});
/*
      std::cout<<"Risk list: ";
      for(auto risk: traj_risk_list)
          std::cout<<risk<<" ";
      std::cout<<std::endl;
      

      std::cout<<"cost list: ";
      for(auto cost: traj_cost_list)
          std::cout<<cost<<" ";
      std::cout<<std::endl;
     
*/ 

      robotModel.actions = actions;
    robotModel.reward_list = traj_cost_list;
    robotModel.risk_list = traj_risk_list;
    robotModel.h = 1;


    double min_risk_val=10.;
    int min_risk_idx;
      for(int i=0;i<traj_risk_list.size();i++)
          if(traj_risk_list[i]<min_risk_val){
            min_risk_idx=i;
            min_risk_val=traj_risk_list[i];
          }

    std::cout<<"Min Risk: "<<min_risk_val<<", CC: "<<risk_cc <<"\n";



    if(min_risk_val > risk_cc){
      ROS_ERROR("Min Risk action taken.");
      return false;

      best_traj = all_traj[min_risk_idx];
      best_traj_cost = traj_cost_list[min_risk_idx];



      if (best_traj_cost >= 0) {
          traj.xv_ = best_traj.xv_;
          traj.yv_ = best_traj.yv_;
          traj.thetav_ = best_traj.thetav_;
          traj.cost_ = best_traj_cost;
          traj.resetPoints();
          double px, py, pth;
          for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
              best_traj.getPoint(i, px, py, pth);
              traj.addPoint(px, py, pth);
          }
      }


    return best_traj_cost >= 0; //return false to aviod this action

    }




    ccssp solver(robotModel, risk_cc);
    int best_action_idx=0;

      if(solver.feasiblility())
        best_action_idx = solver.get_root_best_action();
            

      best_traj = all_traj[best_action_idx];
      best_traj_cost = traj_cost_list[best_action_idx];



      int best_cost_idx;
      double best_cost_val=99999;
      for(int i=0;i<traj_cost_list.size();i++)
          if(traj_cost_list[i]<best_cost_val){
            best_cost_idx=i;
            best_cost_val=traj_cost_list[i];
          }


      std::cout<<"Best action idx: "<<best_action_idx<<" risk: "<<traj_risk_list[best_action_idx]<<" cost: "<< best_traj_cost<<"\n";
      std::cout<<"forloop action idx: "<<best_cost_idx<<" risk: "<<traj_risk_list[best_cost_idx]<<" cost: "<< best_cost_val<<"\n";



      if (best_traj_cost >= 0) {
          traj.xv_ = best_traj.xv_;
          traj.yv_ = best_traj.yv_;
          traj.thetav_ = best_traj.thetav_;
          traj.cost_ = best_traj_cost;
          traj.resetPoints();
          double px, py, pth;
          for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
              best_traj.getPoint(i, px, py, pth);
              traj.addPoint(px, py, pth);
          }
      }






    return best_traj_cost >= 0;
  }

  
}// namespace
