#include "ika_pomdp.hpp"

double DecisionMaker::pdf(const double &mean, const double &std_dev, const double &x)
{ // http://www.cplusplus.com/reference/random/normal_distribution/
  return 1. / (std_dev * sqrt(2. * M_PI)) * exp(-(x - mean) * (x - mean) / (2. * std_dev * std_dev));
}

// TODO: get angle between route and ego
// TODO: convert last n acceleration lat long from ego coordinate system to route
// in PathExtractor?

bool DecisionMaker::isterminal(const State &s)
{
  // TODO: implement!
  return false;
}

void DecisionMaker::getAcceleration(const int16_t &action, double &acc_lon, double &acc_lat)
{
  acc_lon = actions_matrix_(action, 1);
  acc_lat = actions_matrix_(action, 2);
}

void DecisionMaker::select_randomly(generated_it &start, const generated_it &end)
{
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(rg_));
}

void DecisionMaker::insert(CategoricalVector &cv, const std::pair<State, double> &item, const double &weight)
{
  cv.items.push_back(item);
  cv.cdf.push_back(cv.cdf.back() + weight);
}

std::pair<State, double> DecisionMaker::rand(const CategoricalVector &cv)
{
  double t = double_dis_(rg_) * cv.cdf.back();
  uint large = cv.cdf.size(); // index of cdf value that is bigger than t
  uint small = 0;             // index of cdf value that is smaller than t

  while (large > small + 1)
  {
    uint new_pos = div(small + large, 2).quot;

    if (t < cv.cdf.at(new_pos))
    {
      large = new_pos;
    }
    else
    {
      small = new_pos;
    }
  }
  return cv.items.at(large);
}

size_t DecisionMaker::hashForOChildLookup(const uint &h, const Action &a)
{
  std::size_t val{0};
  boost::hash_combine(val, h);
  boost::hash_combine(val, a);
  return val;
}