#include "ika_pomdp.hpp"

void DecisionMaker::simulate_no_output(POWTreeObsNode &h_node, const State &s, const uint &d)
{
  (void)simulate(h_node, s, d); // normally simulate() returns total reward, but ignored here
}

double DecisionMaker::simulate(POWTreeObsNode &h_node, const State &s, const uint &d)
{
  POMCPOWTree &tree = *h_node.tree;
  const uint &h = h_node.node;

  if (isterminal(s) || d <= 0)
  {
    return 0.0;
  }

  // for now: no action pruning | TODO
  if (tree.tried.at(h).empty())
  {
    // TODO here the action space could be restricted based on state/belief
    // if (h == 1)
    // {
    // std::vector<Action> action_space = actions_;
    // action_space_iter = POMDPs.actions(pomcp.problem, tree.root_belief)
    // }
    // else
    // {
    // std::vector<Action> action_space = actions_;
    // action_space_iter = POMDPs.actions(pomcp.problem, StateBelief(tree.sr_beliefs[h]));
    // }

    uint anode = tree.n.size(); // Why this? Where does this influence something?

    for (Action const &a : actions_)
    {
      push_anode(tree, h, a, false);
    }
  }

  uint &total_n = tree.total_n.at(h);
  uint best_node = select_best(h_node);
  Action &a = tree.a_labels.at(best_node);
  bool new_node = false;
  double r = 0.0;
  State sp;
  Observation o;
  uint hao;

  if (tree.n_a_children.at(best_node) <= k_observation_ * pow(tree.n.at(best_node), alpha_observation_))
  {
    gen(s, a, time_step_, sp, r, o);
    // no checking of repeating observation because continouos observations

    new_node = true;
    hao = tree.sr_beliefs.size() + 1;
    CategoricalVector cv{{std::make_pair(sp, r)}, {weightObs(s, sp, a, o)}};

    tree.sr_beliefs.push_back(POWNodeBelief{a, o, cv});

    tree.total_n.push_back(0);
    std::vector<uint> tried;
    tree.tried.push_back(tried);
    tree.o_labels.push_back(o);
    tree.n_a_children.at(best_node) += 1;

    tree.generated.at(best_node).push_back(std::make_pair(o, hao));
  }
  else
  {
    gen(s, a, time_step_, sp, r);
  }

  double R; // no need for checking r == inf

  if (new_node)
  {
    // R = r + discount_ * estimate_value(pomcp.solved_estimate, pomcp.problem, sp, POWTreeObsNode{tree, hao}, d - 1); // TODO
    // dummy
    R = r + discount_ * 1.0;
  }
  else
  {
    generated_it gen_it = tree.generated.at(best_node).begin();
    select_randomly(gen_it, tree.generated.at(best_node).end());
    o = gen_it->first;
    hao = gen_it->second;
    push_weighted(tree.sr_beliefs.at(hao), s, sp, r);

    std::pair<State, double> pair = rand(tree.sr_beliefs.at(hao).dist);

    sp = pair.first;
    r = pair.second;

    POWTreeObsNode obsnode = POWTreeObsNode{&tree, hao};

    R = r + discount_ * simulate(obsnode, sp, d - 1);
  }

  tree.n.at(best_node) += 1;
  tree.total_n.at(h) += 1;

  tree.v.at(best_node) += (R - tree.v.at(best_node)) / tree.n.at(best_node);

  return R;
}

uint DecisionMaker::select_best(const POWTreeObsNode &h_node)
{
  if (select_criterion_ == 1)
  {
    auto &tree = h_node.tree;
    uint h = h_node.node;
    uint best_node = tree->tried.at(h).at(0);
    double best_v = tree->v.at(best_node);
    assert(!std::isnan(best_v));

    for (uint n = 1; n < tree->tried.at(h).size(); n++)
    {
      auto &node = tree->tried.at(h).at(n);
      if (tree->v.at(node) >= best_v)
      {
        best_v = tree->v.at(node);
        best_node = node;
      }
    }
    return best_node;
  }
  // TODO: implement MaxUCB and MaxTries
}

void DecisionMaker::push_weighted(POWNodeBelief b, const State &s, const State &sp, const double &r)
{
  insert(b.dist, std::make_pair(sp, r), weightObs(s, sp, b.a, b.o));
}