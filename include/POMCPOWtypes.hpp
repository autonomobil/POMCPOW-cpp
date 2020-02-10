#pragma once

#include <boost/functional/hash.hpp>

typedef uint16_t Action;

typedef struct State
{
  double s;
  double d;
  double v;
} State;

typedef struct Observation
{
  double x;
  double y;
  double v;
} Observation;

typedef std::vector<std::pair<State, double>> Belief;

typedef struct ObservationStdDevs
{
  double x_stddev;
  double y_stddev;
  double v_stddev;
} ObservationStdDevs;

typedef struct CategoricalVector
{
  std::vector<std::pair<State, double>> items;
  std::vector<double> cdf;
} CategoricalVector;

typedef struct POWNodeBelief
{
  Action a;
  Observation o;
  CategoricalVector dist;
} POWNodeBelief;

typedef struct POMCPOWTree
{
  // Action nodes
  std::vector<uint> n;
  std::vector<double> v;
  std::vector<std::vector<std::pair<Observation, uint>>> generated;
  // std::unordered_map<size_t, uint> a_child_lookup; // not needed, because continous observation -> no repeating observation
  std::vector<Action> a_labels;
  std::vector<uint> n_a_children;

  // Observation nodes
  std::vector<POWNodeBelief> sr_beliefs;
  std::vector<uint> total_n;
  std::vector<std::vector<uint>> tried;
  std::unordered_map<size_t, uint> o_child_lookup;
  std::vector<Observation> o_labels;

  Belief root_belief;

} POMCPOWTree;

typedef struct POWTreeObsNode
{
  POMCPOWTree *tree;
  const uint node;
} POWTreeObsNode;

typedef std::vector<std::pair<Observation, uint>>::iterator generated_it;
