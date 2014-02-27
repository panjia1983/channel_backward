#pragma once

#include "common.hpp"
#include "fwd.hpp"

namespace Needle {
  struct NeedleCollisionHash : CollisionHash {
    NeedleCollisionHash(NeedleProblemHelperPtr helper);
    size_t hash(const DblVec& x);

    NeedleProblemHelperPtr helper;
  };
}
