#ifndef DECISION_MAKING_HPP
#define DECISION_MAKING_HPP

#include <random>
#include <vector>
#include <iterator>
#include <iostream>
#include <chrono>
#include <ctime>
#include <unordered_map>
#include "POMCPOWtypes.hpp"

#include <ros/ros.h>

// I/O msgs
#include <definitions/IkaTpSampledRefPath.h>
#include <definitions/IkaTpSampledBoundaries.h>
#include <definitions/IkaRouteStatus.h>
#include <definitions/IkaRoute.h>
#include <definitions/IkaActionSequence.h>
#include <definitions/IkaEgoState.h>
#include <definitions/IkaObjectList.h>
#include <definitions/utility/ika_utilities.h> // includes all ros tf stuff

// LANELET2 Header
#include <lanelet2_utils.h> // includes allmost all lanelet headers
#include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/math/distributions/normal.hpp>
#include <boost/geometry.hpp>

class DecisionMaker
{
public:
  int _argc;
  char **_argv;

  DecisionMaker(int argc, char **argv);

  // CALLBACKS
  void callbackRouteStatus(const definitions::IkaRouteStatusConstPtr &msg);
  void callbackRoute(const definitions::IkaRouteConstPtr &msg);
  void callbackActionSequence(const definitions::IkaActionSequenceConstPtr &msg);
  void callbackEgoState(const definitions::IkaEgoStateConstPtr &msg);
  void callbackPath(const definitions::IkaRouteConstPtr &msg);
  void callbackPathStatus(const definitions::IkaRouteStatusConstPtr &msg);
  void callbackObjectList(const definitions::IkaObjectListConstPtr &msg);

  void init();
  void publishPath();
  void interpretActionSequence();
  void convertCurrentMeasurements2Obs(Observation &obs);
  void getBestSequence();

  //POMDP Model
  double sampleFromMeasurement(const double &mean, const double &std_dev);
  void initialBelief();
  void gen(const State &s, const int16_t &a, const double &dt, State &sp, double &r, Observation &o);
  void gen(const State &s, const int16_t &a, const double &dt, State &sp, double &r);
  double weightObs(const State &s, const State &sp, const int16_t &a, const Observation &o);
  void updateBelief(const Observation &obs);

  // TREE
  POMCPOWTree reserveTree(const Belief &root_belief, uint &size);
  bool isroot(const POWTreeObsNode &obsnode);
  uint push_anode(POMCPOWTree &tree, const int &h, const Action &a, bool update_lookup);
  uint n_children(const POWTreeObsNode &h);
  void clearTree(POMCPOWTree &tree);
  POWNodeBelief sr_belief(const POWTreeObsNode &obsnode);

  // SOLVER
  void search();
  uint select_best(const POWTreeObsNode &h_node);
  // void simulate(const State &s);
  void simulate_no_output(POWTreeObsNode &h_node, const State &s, const uint &d); // no output
  double simulate(POWTreeObsNode &h_node, const State &s, const uint &d); // return total reward
  void push_weighted(POWNodeBelief b, const State &s, const State &sp, const double &r);

  // public member variables
  bool b_planner_initiated_ = false;
  double time_since_last_update_ = 0.;
  Belief belief_;
  uint n_particles_;

  // UTILS
  double pdf(const double &mean, const double &std_dev, const double &x);
  bool isterminal(const State &s);
  void getAcceleration(const int16_t &action, double &acc_lon, double &acc_lat);
  void select_randomly(generated_it &start, const generated_it &end);
  void insert(CategoricalVector &cv, const std::pair<State, double> &item, const double &weight);
  std::pair<State, double> rand(const CategoricalVector &cv);
  POMCPOWTree tree_;
  size_t hashForOChildLookup(const uint &h, const Action &a);

private:
  ros::Subscriber boundariesSub;
  ros::Subscriber routeStatusSub;
  ros::Subscriber pathSub;
  ros::Subscriber pathStatusSub;
  ros::Subscriber routeSub;
  ros::Subscriber routeStateSub;
  ros::Subscriber egoStateSub;
  ros::Subscriber objectListSub;

  ros::Publisher referencePathPub;
  ros::Publisher boundariesPub;
  ros::Publisher refPathVisuPub;
  ros::Publisher actionSequencePub;

  double time_step_;

  // POMCPOW parameter
  double planning_horizon_;
  double max_time_planner_;
  double maxUCB_;
  uint8_t select_criterion_;
  uint k_observation_;
  uint k_action_;
  double alpha_observation_;
  double alpha_action_;
  double discount_;

  bool b_got_route_ = false;
  bool b_got_path_status_ = false;
  bool b_got_path_ = false;
  bool b_got_ego_state_ = false;

  definitions::IkaRouteStatus route_status_;
  definitions::IkaRoute route_;
  definitions::IkaRouteStatus path_status_;
  definitions::IkaRoute path_;
  definitions::IkaEgoState ego_state_;
  definitions::IkaObjectList object_list_;
  definitions::IkaActionSequence action_seq_;

  BasicLineString2d route_ll_line_;
  BasicLineString2d path_ll_line_map_;

  tf::TransformListener tf_listener_;

  Eigen::MatrixXf actions_matrix_;
  std::vector<uint16_t> actions_;
  std::vector<double> vel_vec_;
  std::vector<double> s_vec_;

  ObservationStdDevs obs_stddev_;

  std::default_random_engine rg_; // random generator
 std::uniform_real_distribution<double> double_dis_; //(0.0, 1.0);
};

#endif //DECISION_MAKING_HPP