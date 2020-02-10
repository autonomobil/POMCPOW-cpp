#include "ika_pomdp.hpp"

void DecisionMaker::interpretActionSequence()
{
  double vel_x = ego_state_.fVelocity;
  // double vel_y = 0.0; // no lateral velocity at beginning of sequence
  // ROS_INFO_STREAM("size action seq" << action_seq_.actions.size());
  ROS_INFO_STREAM("route s: " << route_status_.s
                              << " route d: " << route_status_.d
                              << " dest s:" << route_status_.s_dest
                              << " ego x" << ego_state_.fPosX
                              << " ego y" << ego_state_.fPosY
                              << " ego vel: " << ego_state_.fVelocity);

  double s_arc = route_status_.s;
  double d_arc = route_status_.d;

  path_ll_line_map_.clear(); // path in map frame
  vel_vec_.clear();
  s_vec_.clear();

  path_ll_line_map_.reserve(action_seq_.actions.size() + 1);
  vel_vec_.reserve(action_seq_.actions.size() + 1);
  s_vec_.reserve(action_seq_.actions.size() + 1);

  path_ll_line_map_.push_back(BasicPoint2d(ego_state_.fPosX, ego_state_.fPosY));
  vel_vec_.push_back(ego_state_.fVelocity);
  s_vec_.push_back(s_arc);

  for (size_t i = 0; i < action_seq_.actions.size(); i++)
  {
    double acc_lon, acc_lat;
    getAcceleration(action_seq_.actions.at(i), acc_lon, acc_lat);

    s_arc += vel_x * time_step_ + 0.5 * acc_lon * time_step_ * time_step_;
    d_arc += 0.5 * acc_lat * time_step_ * time_step_;
    vel_x = std::max(0.0, vel_x + time_step_ * acc_lon);
    // vel_x = vel_x + time_step_ * acc_lon;
    // vel_y += time_step_ * acc_lat;

    lanelet::ArcCoordinates arc_coor;
    arc_coor.length = s_arc;
    arc_coor.distance = d_arc;

    path_ll_line_map_.push_back(lanelet::geometry::fromArcCoordinates(route_ll_line_, arc_coor));
    vel_vec_.push_back(vel_x);
    s_vec_.push_back(s_arc);

    ROS_INFO_STREAM(i << "-acc_lon:" << acc_lon << "  acc_lat:" << acc_lat << "  s.arc: " << s_arc << "  d.arc: " << d_arc << "  v:" << vel_vec_.at(i));
  }
  ROS_INFO_STREAM("\n");
}

void DecisionMaker::publishPath()
{
  definitions::IkaTpSampledRefPath ref_path;

  if (b_got_route_ && b_got_path_ && b_got_ego_state_ && b_got_path_status_ && route_status_.status == 2)
  {
    lanelet::BasicLineString2d path_ll_line_baselink; //_.clear(); // path in base_link frame

    geometry_msgs::TransformStamped geo_transform;
    tf::StampedTransform tf_transform;
    tf_listener_.lookupTransform("/base_link", "/map", ros::Time(0), tf_transform);
    tf::transformStampedTFToMsg(tf_transform, geo_transform);

    // BasicLineString2d output_line;
    // transform to base_link, "map" is input frame
    Lanelet2Utilities::transformLaneletLine2Frame(path_ll_line_map_, "/map", geo_transform, path_ll_line_baselink);

    ref_path.samples.reserve(path_ll_line_baselink.size());

    for (uint i = 0; i < path_ll_line_baselink.size(); i++)
    {
      definitions::IkaTpSamplePointRefPath ref_point;
      ref_point.x = path_ll_line_baselink.at(i).x();
      ref_point.y = path_ll_line_baselink.at(i).y();
      ref_point.s = s_vec_.at(i) - route_status_.s;
      ref_point.v = std::max(0.0, vel_vec_.at(i) * 3.6);

      ref_path.samples.push_back(ref_point);
    }
  }
  else
  {
    // Stillstand
    definitions::IkaTpSampledRefPath dummy;
    dummy.samples.resize(1);
    ref_path = dummy;
    ROS_INFO_STREAM("dummy ref path");
  }

  ref_path.header.stamp = ros::Time::now();
  ref_path.header.frame_id = "/base_link";

  visualization_msgs::Marker marker;
  IkaUtilities::convertIkaTpRefPath2Marker(ref_path, marker, "/base_link", "ref_pathvisu", {0.1, 0.7, 0.1});
  refPathVisuPub.publish(marker);

  referencePathPub.publish(ref_path);



  // TODO: BOUNDARIES
  // definitions::IkaTpSampledBoundaries boundaries;
  // boundaries.samples_left.reserve(path_.boundary_left_critical.size());
  // for (uint p = 0; p < path_.boundary_left_critical.size(); p++)
  // {
  //   boundaries.samples_left.push_back();
  // }

  // boundaries.samples_right.reserve(path_.boundary_left_critical.size());

  // for (uint p = 0; p < path_.boundary_right_critical.size(); p++)
  // {
  //   definitions::IkaTpSamplePointBoundary b_p;
  //   b_p.path_.boundary_right_critical
  //   boundaries.samples_right.push_back();
  // }

  // boundariesPub.publish(boundaries);
}