#include "ika_pomdp.hpp"

bool DecisionMaker::isroot(const POWTreeObsNode &obsnode)
{
  return obsnode.node == 1;
}

// uint DecisionMaker::push_anode(POMCPOWTree &tree, int &h, Action &a, int &n = 0, double v = 0.0, bool update_lookup = true)
uint DecisionMaker::push_anode(POMCPOWTree &tree, const int &h, const Action &a, bool update_lookup = true)
{
  int n = 0;      // default, https://github.com/JuliaPOMDP/POMCPOW.jl
  double v = 0.0; // default, https://github.com/JuliaPOMDP/POMCPOW.jl
  uint anode = tree.n.size() + 1;
  tree.n.push_back(n);
  tree.v.push_back(v);
  std::vector<std::pair<Observation, uint>> tmp;
  tree.generated.push_back(tmp);
  tree.a_labels.push_back(a);
  tree.n_a_children.push_back(0);
  if (update_lookup)
  {
    size_t key = hashForOChildLookup(h,a);
    tree.o_child_lookup[key] = anode;
  }

  tree.tried.at(h).push_back(anode);
  tree.total_n.at(h) += n;
  return anode;
}

POMCPOWTree DecisionMaker::reserveTree(const Belief &root_belief, uint &size)
{
  uint size_real = std::min((uint)100000, size * 1000);
  // planning_horizon_;
  POMCPOWTree tree;
  tree.n.reserve(size_real);
  tree.v.reserve(size_real);
  tree.generated.reserve(size_real);
  // tree.a_child_lookup.reserve(size_real);
  tree.a_labels.reserve(size_real);
  tree.n_a_children.reserve(size_real);

  tree.sr_beliefs.reserve(size_real);
  tree.total_n.reserve(size_real);
  tree.tried.reserve(size_real);
  tree.o_child_lookup.reserve(size_real);
  tree.o_labels.reserve(size_real);
  tree.root_belief = root_belief;

  return tree;
}

void DecisionMaker::clearTree(POMCPOWTree &tree)
{
  // planning_horizon_;
  tree.n.clear();
  tree.v.clear();
  tree.generated.clear();
  // tree.a_child_lookup.clear();
  tree.a_labels.clear();
  tree.n_a_children.clear();

  tree.sr_beliefs.clear();
  tree.total_n.clear();
  tree.tried.clear();
  tree.o_child_lookup.clear();
  tree.o_labels.clear();
  // tree.root_belief = root_belief;
  // return tree;
}

// POWNodeBelief DecisionMaker::belief(const POWTreeObsNode &h)
// {
//   if (isroot(h))
//   {
//     return h.tree->root_belief;
//   }
//   else
//   {
//     return StateBelief(h.tree.sr_beliefs[h.node]);
//   }
// }

POWNodeBelief DecisionMaker::sr_belief(const POWTreeObsNode &h)
{
  if (isroot(h))
  {
    perror("Tried to access the sr_belief for the root node in a POMCPOW tree");
  }
  else
  {
    return h.tree->sr_beliefs.at(h.node);
  }
}

uint DecisionMaker::n_children(const POWTreeObsNode &h)
{
  return h.tree->tried.at(h.node).size();
}
