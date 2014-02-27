#include "needle_collision_hash.hpp"
#include "utils.hpp"
#include "needle_problem_helper.hpp"

namespace Needle {
  NeedleCollisionHash::NeedleCollisionHash(NeedleProblemHelperPtr helper) : helper(helper) {}
  
  size_t NeedleCollisionHash::hash(const DblVec& x) {
    DblVec extended_x = x;
    for (int i = 0; i < helper->pis.size(); ++i) {
      for (int j = 0; j < helper->pis[i]->local_configs.size(); ++j) {
        DblVec state = toDblVec(logDown(helper->pis[i]->local_configs[j]->pose));
        extended_x.insert(extended_x.end(), state.begin(), state.end());
      }
    }
    return boost::hash_range(extended_x.begin(), extended_x.end());
  }

}
