#include "bvh.h"

namespace Yuki {


    bool BVH::build(const vector<shared_ptr<Primitive> > &prim_list) {
        if (root) clear();
    }

    void BVH::clear() {
        if (!root) return;
        transverse(root, &BVH::delete_op);
        return;
    }

    bool BVH::intersect(const Ray &r) {
        if (!root) return;
        ray = r;
        // transverse to update primitive;
        transverse(root, &BVH::intersect_op);
    }

    void BVH::transverse(BVHNode *root, bool(BVH::*operation)(BVHNode *&)) {
        // todo

        return;
    }

    bool BVH::delete_op(BVHNode *&node) {
        node->l_child = node->r_child = NULL;
        delete node;
        node = NULL;
        return true;
    }

    bool BVH::intersect_op(BVHNode *&node) {
        Float t0, t1;
        bool flag = node->bound.intersect_p(ray, &t0, &t1);
        if (flag) {
            if (node->primitive != NULL) {
                if (node->primitive->intersect_p(ray) &&
                    node->primitive->intersect(ray, &insect)) {
                    
                }
            }
        }
        return flag;
    }

}