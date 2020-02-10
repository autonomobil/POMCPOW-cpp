#include "ika_pomdp.hpp"

DecisionMaker::DecisionMaker(int argc, char **argv) : _argc(argc), _argv(argv) {}

void DecisionMaker::init()
{
  ros::NodeHandle n;

  // Read config from param file
  XmlRpc::XmlRpcValue ika_decision_making_config;
  if (!n.getParam("ika_decision_making", ika_decision_making_config))
  {
    ROS_ERROR("ika_decision_making config file not found");
  }
  ROS_ASSERT(ika_decision_making_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  Eigen::MatrixXf actions_matrix_ = IkaUtilities::XmlRpcMatrixToEigenMatrixXf(ika_decision_making_config["actions"]);

  for (uint16_t i = 0; i < actions_matrix_.rows(); i++)
  {
    actions_.push_back(i);
  }

  discount_ = ika_decision_making_config["discount"];
  k_observation_ = int(ika_decision_making_config["k_observation"]);
  k_action_ = int(ika_decision_making_config["k_action"]);
  alpha_observation_ = ika_decision_making_config["alpha_observation"];
  alpha_action_ = ika_decision_making_config["alpha_action_"];
  select_criterion_ = int(ika_decision_making_config["select_criterion"]);
  maxUCB_ = ika_decision_making_config["maxUCB"];
  planning_horizon_ = ika_decision_making_config["planning_horizon"];
  time_step_ = ika_decision_making_config["time_step"];
  n_particles_ = int(ika_decision_making_config["n_particles"]);
  max_time_planner_ = ika_decision_making_config["max_time_planner"];
  Eigen::MatrixXf obs_stddev = IkaUtilities::XmlRpcMatrixToEigenMatrixXf(ika_decision_making_config["observation_uncertainty"]);

  pathSub = n.subscribe("/path_extraction/dyn_path", 10, &DecisionMaker::callbackPath, this);
  pathStatusSub = n.subscribe("/path_extraction/path_status", 10, &DecisionMaker::callbackPathStatus, this);

  routeSub = n.subscribe("/route_planning/route", 10, &DecisionMaker::callbackRoute, this);
  routeStatusSub = n.subscribe("/route_planning/routeStatus", 10, &DecisionMaker::callbackRouteStatus, this);

  egoStateSub = n.subscribe("/localization/ego_state", 10, &DecisionMaker::callbackEgoState, this);

  objectListSub = n.subscribe("/fusion/ikaObjectList", 10, &DecisionMaker::callbackObjectList, this);

  actionSequencePub = n.advertise<definitions::IkaActionSequence>("/ika_decision_making/actions", 10);
  refPathVisuPub = n.advertise<visualization_msgs::Marker>("/action_interpreter/visu_refPath", 10);
  referencePathPub = n.advertise<definitions::IkaTpSampledRefPath>("/ika_tp/ika_tp_pre/in/reference_path", 10);
  boundariesPub = n.advertise<definitions::IkaTpSampledBoundaries>("/ika_tp/ika_tp_pre/in/boundaries", 10);

  belief_.reserve(n_particles_);

  obs_stddev_.x_stddev = obs_stddev(1);
  obs_stddev_.y_stddev = obs_stddev(2);
  obs_stddev_.v_stddev = obs_stddev(3);

  double_dis_ = std::uniform_real_distribution<double>(0.0, 1.0); // init 0.0-1.0 double distribution for random
}

void DecisionMaker::convertCurrentMeasurements2Obs(Observation &obs)
{
  obs.x = ego_state_.fPosX;
  obs.y = ego_state_.fPosY;
  obs.v = ego_state_.fVelocity;
}

double DecisionMaker::sampleFromMeasurement(const double &mean, const double &std_dev)
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, std_dev);
  return (distribution(generator));
}

void DecisionMaker::initialBelief()
{
  double init_weight = 1. / belief_.size();

  for (uint i = 0; i < belief_.size(); i++)
  {
    double pos_x = sampleFromMeasurement(ego_state_.fPosX, 0.5); // todo: ego state msg needs variance = std_dev²
    double pos_y = sampleFromMeasurement(ego_state_.fPosY, 0.5); // todo: ego state msg needs variance = std_dev²

    lanelet::ArcCoordinates arc = lanelet::geometry::toArcCoordinates(route_ll_line_, lanelet::BasicPoint2d(pos_x, pos_y));

    State state_particle;
    // sample from distributions
    state_particle.s = arc.length;
    state_particle.d = arc.distance;
    state_particle.v = sampleFromMeasurement(ego_state_.fVelocity, 0.5); // todo: ego state msg needs variance = std_dev²

    belief_.at(i) = std::make_pair(state_particle, init_weight);
  }
}

void DecisionMaker::getBestSequence()
{
  uint bindex = 1; // best index
  action_seq_.actions.clear();
  while (tree_.tried.at(bindex).empty())
  {
    // auto anodes = tree_.tried.at(bindex); // action nodes, needed?
    POWTreeObsNode bnode{&tree_, bindex}; // belief Node
    uint best_anode = select_best(bnode); // best action node
    action_seq_.actions.push_back(tree_.a_labels.at(best_anode));
    uint argmax = 0;
    uint i = 0;

    for (const std::pair<Observation, uint> tmp : tree_.generated.at(best_anode))
    {
      if (tree_.total_n.at(tmp.second) > argmax)
      {
        argmax = tree_.total_n.at(tmp.second);
        bindex = tree_.generated.at(best_anode).at(i).second;
      }
      i++;
    }
  }
}

void DecisionMaker::search()
{
  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds;
  elapsed_seconds = std::chrono::system_clock::now() - start;
  uint part_no = 0;
  bool all_terminal = true;

  while (elapsed_seconds.count() <= max_time_planner_)
  {
    State s = belief_.at(part_no).first;
    if (!isterminal(s))
    {
      // search POMCPOWTree here
      POWTreeObsNode obsnode = POWTreeObsNode{&tree_, 1};
      simulate_no_output(obsnode, s, planning_horizon_);
      all_terminal = false;
    }

    auto end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    part_no++;

    if (part_no > belief_.size())
    {
      part_no = 0;
    }
  }

  if (all_terminal)
  {
    perror("ALL SAMPLES TERMINAL");
  }

  // POWTreeObsNode obsnode = POWTreeObsNode{&tree_, 1};
  // uint best_node = select_best(obsnode);

  // return tree_.a_labels[best_node];
}

void DecisionMaker::gen(const State &s, const int16_t &a, const double &dt, State &sp, double &r, Observation &o)
{
}

// overloaded without observation
void DecisionMaker::gen(const State &s, const int16_t &a, const double &dt, State &sp, double &r)
{
}

double DecisionMaker::weightObs(const State &s, const State &sp, const int16_t &a, const Observation &o)
{
  BasicPoint2d sp_map = lanelet::geometry::fromArcCoordinates(route_ll_line_, ArcCoordinates{sp.s, sp.d});

  double prob = pdf(0.0, obs_stddev_.x_stddev, sp_map.x() - o.x);
  prob *= pdf(0.0, obs_stddev_.y_stddev, sp_map.y() - o.y);
  prob *= pdf(0.0, obs_stddev_.v_stddev, sp.v - o.v);
  return prob;
}

void DecisionMaker::updateBelief(const Observation &obs)
{
  for (uint i = 0; i < belief_.size(); i++)
  {
    State s = belief_.at(i).first;
    double r;
    Observation o;

    // gen(a, dt, s, r, o)

    // belief_.at(i).second *= weightObs(s, sp, a, obs);

    // TODO: normalize weights to weights_sum = 1
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_making_node");

  DecisionMaker decision_maker(argc, argv);
  decision_maker.init();
  // POMCPOWTree tree_;

  while (ros::ok())
  {
    ros::spinOnce();

    decision_maker.initialBelief(); // for now: sampling new particles every run from measurements, no update!

    if (!decision_maker.b_planner_initiated_)
    {
      decision_maker.tree_ = decision_maker.reserveTree(decision_maker.belief_, decision_maker.n_particles_);
    }
    else
    {
      decision_maker.clearTree(decision_maker.tree_);
      decision_maker.tree_.root_belief = decision_maker.belief_;
    }

    decision_maker.search();

    decision_maker.getBestSequence();

    decision_maker.interpretActionSequence();

    decision_maker.publishPath();

    Observation obs;

    decision_maker.convertCurrentMeasurements2Obs(obs);

    // decision_maker.updateBelief(obs);

    decision_maker.time_since_last_update_ = ros::Time::now().toSec() - decision_maker.time_since_last_update_;
  }

  return 0;
}
