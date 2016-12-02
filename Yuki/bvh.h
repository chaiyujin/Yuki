/* -----------------------------
* File   : bvh.h
* Author : Yuki Chai
* Created: 2016.12.2
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_BVH_H__
#define __YUKI_BVH_H__

#include <primitive.h>
#include <vector>
#include <memory>
using std::vector;
using std::shared_ptr;

namespace Yuki {
    struct BVHNode {
        BBox    bound;
        BVHNode *l_child, *r_child;
        shared_ptr<Primitive> primitive;
        BVHNode()
            : bound(), l_child(NULL), r_child(NULL),
              primitive(NULL) {}
        BVHNode(shared_ptr<Primitive> prim)
            : l_child(NULL), r_child(NULL),
              primitive(prim) {
            bound = primitive->world_bound();
        }
    };

    /* BVH using SAH */
    class BVH {
    private:
        BVHNode *root;
    public:
        BVH() : root(NULL) {}
        BVH(const vector<shared_ptr<Primitive> > &prim_list) { build(prim_list); }
        ~BVH() {}

        bool build(const vector<shared_ptr<Primitive> > &prim_list);
        void clear();
    };
}
#endif
