#include "ika_pomdp.hpp"
// All callbacks for ika_decision_making

void DecisionMaker::callbackRouteStatus(const definitions::IkaRouteStatusConstPtr &msg)
{
  route_status_ = *msg;
}

void DecisionMaker::callbackRoute(const definitions::IkaRouteConstPtr &msg)
{
  route_ = *msg;
  Lanelet2Utilities::convertIkaLine2LaneletLine(route_ll_line_, route_.centerline);
  b_got_route_ = true;
}

void DecisionMaker::callbackPathStatus(const definitions::IkaRouteStatusConstPtr &msg)
{
  path_status_ = *msg;
  b_got_path_status_ = true;
}

void DecisionMaker::callbackPath(const definitions::IkaRouteConstPtr &msg)
{
  path_ = *msg;
  b_got_path_ = true;
}

void DecisionMaker::callbackEgoState(const definitions::IkaEgoStateConstPtr &msg)
{
  ego_state_ = *msg;
  b_got_ego_state_ = true;
}

void DecisionMaker::callbackObjectList(const definitions::IkaObjectListConstPtr &msg)
{
  object_list_ = *msg;
}