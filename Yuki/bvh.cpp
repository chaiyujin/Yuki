#include "bvh.h"

namespace Yuki {

    bool BVH::build(const vector<shared_ptr<Primitive> > &prim_list) {
        if (root) clear();
    }

    void BVH::clear() {

    }

}