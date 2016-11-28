/* -----------------------------
* File   : transform.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_TRANSFORM_H__
#define __YUKI_TRANSFORM_H__

#include <log.h>
#include <core.h>
#include <geometry.h>
#include <intersection.h>

namespace Yuki {
  
    // matrix 4x4
    struct Matrix4x4 {
        Float m[4][4];
        // default as identity matrix
        Matrix4x4() {
            memset(m, 0, sizeof(Float) * 16);
            for (int i = 0; i < 4; ++i) m[i][i] = 1.f;
        }
        Matrix4x4(const Float f[4][4]) {
            memcpy(m, f, sizeof(Float) * 16);
        }
        Matrix4x4(
            Float t00, Float t01, Float t02, Float t03,
            Float t10, Float t11, Float t12, Float t13,
            Float t20, Float t21, Float t22, Float t23,
            Float t30, Float t31, Float t32, Float t33) {
            m[0][0] = t00; m[0][1] = t01; m[0][2] = t02; m[0][3] = t03;
            m[1][0] = t10; m[1][1] = t11; m[1][2] = t12; m[1][3] = t13;
            m[2][0] = t20; m[2][1] = t21; m[2][2] = t22; m[2][3] = t23;
            m[3][0] = t30; m[3][1] = t31; m[3][2] = t32; m[3][3] = t33;
        }
        Matrix4x4(const Matrix4x4 &mat) {
            memcpy(m, mat.m, sizeof(Float) * 16);
        }

        const Matrix4x4 &operator=(const Matrix4x4 &mat) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    m[i][j] = mat.m[i][j];
                }
            }
            return *this;
        }

        bool operator==(const Matrix4x4 &mat) const {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (!equal_zero(m[i][j] - mat.m[i][j])) return false;
                }
            }
            return true;
        }

        bool operator!=(const Matrix4x4 &mat) const {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (!equal_zero(m[i][j] - mat.m[i][j])) return true;
                }
            }
            return false;
        }

        bool is_identity() const {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (i == j) {
                        if (!equal_zero(m[i][j] - 1.0f)) return false;
                    }
                    else {
                        if (!equal_zero(m[i][j])) return false;
                    }
                }
            }
            return true;
        }
#ifdef DEBUG
        friend std::ostream &operator<<(std::ostream &out, const Matrix4x4 &mat);
#endif // DEBUG
    };

    Matrix4x4 transpose(const Matrix4x4 &m);
    Matrix4x4 inverse(const Matrix4x4 &m);
    Matrix4x4 mul(const Matrix4x4 &m1, const Matrix4x4 &m2);

    // Transform

    class Transform {
    private:
        Matrix4x4 m, m_inv;
    public:
        friend Quaternion;
        friend AnimatedTransform;
        Transform() :
            m(), m_inv() {}
        Transform(const Float mat[4][4]) {
            m = Matrix4x4(mat);
            m_inv = inverse(m);
        }
        Transform(const Matrix4x4 &mat) :
            m(mat), m_inv(inverse(mat)) {}
        Transform(const Matrix4x4 &mat, const Matrix4x4 &minv) :
            m(mat), m_inv(minv) {}
        Transform(const Transform &t) : m(t.m), m_inv(t.m_inv) {}
        friend Transform inverse(const Transform &t);

        const Transform &operator=(const Transform &t) {
            m = t.m;
            m_inv = t.m_inv;
            return *this;
        }

        bool operator==(const Transform &t) const {
            return m == t.m;
        }

        bool operator!=(const Transform &t) const {
            return m != t.m;
        }

        Vector operator()(const Vector &v) const {
            Float x = v.x, y = v.y, z = v.z;
            return Vector(
                m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
        }

        Point operator()(const Point &p) const {
            Float x = p.x, y = p.y, z = p.z;
            Float xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
            Float yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
            Float zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
            Float wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
            if (wp == 1.)	return Point(xp, yp, zp);
            else			return Point(xp / wp, yp / wp, zp / wp);
        }

        Normal operator()(const Normal &n) const {
            Float x = n.x, y = n.y, z = n.z;
            return Normal(
                m_inv.m[0][0] * x + m_inv.m[1][0] * y + m_inv.m[2][0] * z,
                m_inv.m[0][1] * x + m_inv.m[1][1] * y + m_inv.m[2][1] * z,
                m_inv.m[0][2] * x + m_inv.m[1][2] * y + m_inv.m[2][2] * z);
        }

        Ray operator()(const Ray &r) const {
            Ray ret = r;
            ret.o = (*this)(r.o);
            ret.d = (*this)(r.d);
            return ret;
        }

        RayDifferential operator()(const RayDifferential &r) const {
            RayDifferential ret = r;
            ret.o = (*this)(r.o);
            ret.d = (*this)(r.d);
            ret.rx_origin = (*this)(r.rx_origin);
            ret.ry_origin = (*this)(r.ry_origin);
            ret.rx_direction = (*this)(r.rx_direction);
            ret.ry_direction = (*this)(r.ry_direction);
            return ret;
        }

        BBox operator()(const BBox &b) const {
            const Transform &M = *this;
            BBox ret(M(Point(b.p_min.x, b.p_min.y, b.p_min.z)));
            ret = union_BBox(ret, M(Point(b.p_max.x, b.p_min.y, b.p_min.z)));
            ret = union_BBox(ret, M(Point(b.p_min.x, b.p_max.y, b.p_min.z)));
            ret = union_BBox(ret, M(Point(b.p_min.x, b.p_min.y, b.p_max.z)));
            ret = union_BBox(ret, M(Point(b.p_max.x, b.p_max.y, b.p_min.z)));
            ret = union_BBox(ret, M(Point(b.p_max.x, b.p_min.y, b.p_max.z)));
            ret = union_BBox(ret, M(Point(b.p_min.x, b.p_max.y, b.p_max.z)));
            ret = union_BBox(ret, M(Point(b.p_max.x, b.p_max.y, b.p_max.z)));
            return ret;
        }

        // pbrt-v3 implement
        Point operator()(const Point &p, Vector *p_error) const {
            Float x = p.x, y = p.y, z = p.z;
            // Compute transformed coordinates from point _pt_
            Float xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
            Float yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
            Float zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
            Float wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];

            // Compute absolute error for transformed point
            Float x_abs_sum = (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
                               std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
            Float y_abs_sum = (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
                               std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
            Float z_abs_sum = (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
                               std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
            *p_error = epsilon_gamma(3) * Vector(x_abs_sum, y_abs_sum, z_abs_sum);
            CHECK_NE(wp, 0);
            if (wp == 1)
                return Point(xp, yp, zp);
            else
                return Point(xp, yp, zp) / wp;
        }

        // pbrt-v3 implement
        Point operator()(const Point &pt, const Vector &pt_error, Vector* abs_error) const {
            Float x = pt.x, y = pt.y, z = pt.z;
            Float xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
            Float yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
            Float zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
            Float wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
            abs_error->x =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[0][0]) * pt_error.x + std::abs(m.m[0][1]) * pt_error.y +
                    std::abs(m.m[0][2]) * pt_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
                    std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
            abs_error->y =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[1][0]) * pt_error.x + std::abs(m.m[1][1]) * pt_error.y +
                    std::abs(m.m[1][2]) * pt_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
                    std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
            abs_error->z =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[2][0]) * pt_error.x + std::abs(m.m[2][1]) * pt_error.y +
                    std::abs(m.m[2][2]) * pt_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
                    std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
            CHECK_NE(wp, 0);
            if (wp == 1.)
                return Point(xp, yp, zp);
            else
                return Point(xp, yp, zp) / wp;
        }

        // pbrt-v3 implement
        Vector operator()(const Vector &v, Vector *abs_error) const {
            Float x = v.x, y = v.y, z = v.z;
            abs_error->x =
                epsilon_gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
                    std::abs(m.m[0][2] * v.z));
            abs_error->y =
                epsilon_gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
                    std::abs(m.m[1][2] * v.z));
            abs_error->z =
                epsilon_gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
                    std::abs(m.m[2][2] * v.z));
            return Vector(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                          m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                          m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
        }

        // pbrt-v3 implement
        Vector operator()(const Vector &v, const Vector &v_error, Vector *abs_error) const {
            Float x = v.x, y = v.y, z = v.z;
            abs_error->x =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[0][0]) * v_error.x + std::abs(m.m[0][1]) * v_error.y +
                    std::abs(m.m[0][2]) * v_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
                    std::abs(m.m[0][2] * v.z));
            abs_error->y =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[1][0]) * v_error.x + std::abs(m.m[1][1]) * v_error.y +
                    std::abs(m.m[1][2]) * v_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
                    std::abs(m.m[1][2] * v.z));
            abs_error->z =
                (epsilon_gamma(3) + (Float)1) *
                (std::abs(m.m[2][0]) * v_error.x + std::abs(m.m[2][1]) * v_error.y +
                    std::abs(m.m[2][2]) * v_error.z) +
                epsilon_gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
                    std::abs(m.m[2][2] * v.z));
            return Vector(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                          m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                          m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
        }

        // pbrt-v3 implement
        Ray operator()(const Ray &r, Vector *o_error, Vector *d_error) const {
            Point o = (*this)(r.o, o_error);
            Vector d = (*this)(r.d, d_error);
            Float t_max = r.max_t;
            Float length_squared = d.length_squared();
            if (length_squared > 0) {
                Float dt = (abs(d) * *o_error) / length_squared;
                o += d * dt;
            }
            return Ray(o, d, t_max, r.time);
        }

        // pbrt-v3 implement
        Ray operator()(const Ray &r, const Vector &o_error_in, const Vector &d_error_in, 
                       Vector *o_error_out, Vector *d_error_out) const {
            Point o = (*this)(r.o, o_error_in, o_error_out);
            Vector d = (*this)(r.d, d_error_in, d_error_out);
            Float t_max = r.max_t;
            Float length_squared = d.length_squared();
            if (length_squared > 0) {
                Float dt = (abs(d) * *o_error_out) / length_squared;
                o += d * dt;
            }
            return Ray(o, d, t_max, r.time);
        }

        Intersection Transform::operator()(const Intersection &si) const {
            Intersection ret;
            // Transform _p_ and _pError_ in _SurfaceInteraction_
            ret.p = (*this)(si.p, si.p_error, &ret.p_error);

            // Transform remaining members of _SurfaceInteraction_
            const Transform &t = *this;
            ret.n = t(si.n).normalized();
            ret.wo = t(si.wo).normalized();
            ret.time = si.time;
            return ret;
        }

        // translate
        static Transform translate(const Vector &delta) {
            Matrix4x4 m(
                1, 0, 0, delta.x,
                0, 1, 0, delta.y,
                0, 0, 1, delta.z,
                0, 0, 0, 1);
            Matrix4x4 minv(
                1, 0, 0, -delta.x,
                0, 1, 0, -delta.y,
                0, 0, 1, -delta.z,
                0, 0, 0, 1);
            return Transform(m, minv);
        }

        // scale
        static Transform scale(Float x, Float y, Float z) {
            Matrix4x4 m(
                x, 0, 0, 0,
                0, y, 0, 0,
                0, 0, z, 0,
                0, 0, 0, 1);
            Matrix4x4 minv(
                1.f / x,		0,		 0, 0,
                0,		  1.f / y,		 0, 0,
                0,				0, 1.f / z, 0,
                0,				0,		 0, 1);
            return Transform(m, minv);
        }

        bool has_scale() const {
            Float la2 = (*this)(Vector(1, 0, 0)).length_squared();
            Float lb2 = (*this)(Vector(0, 1, 0)).length_squared();
            Float lc2 = (*this)(Vector(0, 0, 1)).length_squared();
#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
            return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
        }

        // rotate
        static Transform rotate_x(Float angle, bool is_radians = false) {
            Float sin_t, cos_t;
            if (is_radians) {
                sin_t = sinf(angle);
                cos_t = cosf(angle);
            }
            else {
                sin_t = sinf(radians(angle));
                cos_t = cosf(radians(angle));
            }
            Matrix4x4 m(
                1, 0, 0, 0,
                0, cos_t, -sin_t, 0,
                0, sin_t, cos_t, 0,
                0, 0, 0, 1);
            return Transform(m, transpose(m));
        }
        static Transform rotate_y(Float angle, bool is_radians = false) {
            Float sin_t, cos_t;
            if (is_radians) {
                sin_t = sinf(angle);
                cos_t = cosf(angle);
            }
            else {
                sin_t = sinf(radians(angle));
                cos_t = cosf(radians(angle));
            }
            Matrix4x4 m(
                cos_t, 0,  sin_t, 0,
                0,	1,		0, 0,
                -sin_t, 0,  cos_t, 0,
                0, 0,		0, 1);
            return Transform(m, transpose(m));
        }
        static Transform rotate_z(Float angle, bool is_radians = false) {
            Float sin_t, cos_t;
            if (is_radians) {
                sin_t = sinf(angle);
                cos_t = cosf(angle);
            }
            else {
                sin_t = sinf(radians(angle));
                cos_t = cosf(radians(angle));
            }
            Matrix4x4 m(
                cos_t, -sin_t, 0, 0,
                sin_t,	cos_t, 0, 0,
                0,		0, 1, 0,
                0,		0, 0, 1);
            return Transform(m, transpose(m));
        }
        static Transform rotate(Float angle, const Vector &axis, bool is_radians = false) {
            Vector a = axis.normalized();
            Float s, c;
            if (is_radians) {
                s = sinf(angle);
                c = cosf(angle);
            }
            else {
                s = sinf(radians(angle));
                c = cosf(radians(angle));
            }
            Float m[4][4];

            m[0][0] = a.x * a.x + (1.f - a.x * a.x) * c;
            m[0][1] = a.x * a.y * (1.f - c) - a.z * s;
            m[0][2] = a.x * a.z * (1.f - c) + a.y * s;
            m[0][3] = 0;

            m[1][0] = a.y * a.x * (1.f - c) + a.z * s;
            m[1][1] = a.y * a.y + (1.f - a.y * a.y) * c;
            m[1][2] = a.y * a.z + (1.f - c) - a.x * s;
            m[1][3] = 0;

            m[2][0] = a.z * a.x * (1.f - c) - a.y * s;
            m[2][1] = a.z * a.y * (1.f - c) + a.x * s;
            m[2][2] = a.z * a.z + (1.f - a.z * a.z) * c;
            m[2][3] = 0;

            m[3][0] = 0;
            m[3][1] = 0;
            m[3][2] = 0;
            m[3][3] = 1;

            Matrix4x4 mat(m);
            return Transform(mat, transpose(mat));
        }

        // loot at
        static Transform look_at(const Point &pos, const Point &look, const Vector &up) {
            Float m[4][4];
            // translate
            m[0][3] = pos.x;
            m[1][3] = pos.y;
            m[2][3] = pos.z;
            m[3][3] = 1;
            // map axis
            Vector dir = (look - pos).normalized();
            Vector left = (up.normalized() ^ dir).normalized();
            Vector new_up = dir ^ left;
            m[0][0] = left.x;
            m[1][0] = left.y;
            m[2][0] = left.z;
            m[3][0] = 0.f;

            m[0][1] = new_up.x;
            m[1][1] = new_up.y;
            m[2][1] = new_up.z;
            m[3][1] = 0.f;

            m[0][2] = dir.x;
            m[1][2] = dir.y;
            m[2][2] = dir.z;
            m[2][3] = 0.f;

            Matrix4x4 cam_to_world(m);
            return Transform(inverse(cam_to_world), cam_to_world);
        }

        // composition of transform
        Transform operator*(const Transform &t) const {
            Matrix4x4 m1 = mul(m, t.m);
            Matrix4x4 m2 = mul(t.m_inv, m_inv);
            return Transform(m1, m2);
        }

        // check if the transform change the handedness
        bool swaps_handedness() const {
            Float det = (
                (m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1])) -
                (m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0])) +
                (m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0])));
            return det < 0.f;
        }

        bool is_identity() const {
            return m.is_identity();
        }

#ifdef DEBUG
        friend std::ostream &operator<<(std::ostream &out, const Transform &t);
#endif // DEBUG
    };


    /* Quaternion */

    class Quaternion {
    private:
        Float norm() const {
            return v.x * v.x + v.y * v.y + v.z * v.z + w * w;
        }
    public:
        // members
        Vector v;
        Float w;
        // constructors
        Quaternion() : v(0.f, 0.f, 0.f), w(1.f) {}
        Quaternion(Float a, Float b, Float c, Float d);
        Quaternion(const Vector &axis, Float angle, bool is_radians = false);
        Quaternion(const Transform &t);
        Quaternion(const Quaternion &q) : v(q.v), w(q.w) {}

        // operators
        Quaternion &operator=(const Quaternion &q) {
            v = q.v;
            w = q.w;
            return *this;
        }
        Quaternion operator+(const Quaternion &q) const {
            return Quaternion(q.v.x + v.x, q.v.y + v.y, q.v.z + v.z, q.w + w);
        }

        Quaternion operator-(const Quaternion &q) const {
            return Quaternion(q.v.x - v.x, q.v.y - v.y, q.v.z - v.z, q.w - w);
        }
        Quaternion &operator+=(const Quaternion &q) {
            v += q.v;
            w += q.w;
            return *this;
        }
        Quaternion operator*(Float f) const {
            return Quaternion(f * v.x, f * v.y, f * v.z, f * w);
        }
        Quaternion &operator*=(Float f) {
            v *= f;
            w *= f;
            return *this;
        }
        Quaternion operator/(Float f) const {
            CHECK(f != 0);
            Float inv = 1.f / f;
            return Quaternion(v.x * inv, v.y * inv, v.z * inv, w * inv);
        }
        Quaternion &operator/=(Float f) {
            CHECK(f != 0);
            Float inv = 1.f / f;
            v *= inv;
            w *= inv;
            return *this;
        }

        friend Float dot(const Quaternion &, const Quaternion &);
        Quaternion normalized() {
            Quaternion &q = *this;
            return q / sqrtf(dot(q, q));
        }

        // convert
        Transform to_transform() const;
        friend std::ostream &operator<<(std::ostream &out, const Quaternion &q);

        // slerp
        friend Quaternion slerp(Float t, const Quaternion &q1, const Quaternion &q2);
    };
    inline Quaternion operator*(Float f, const Quaternion &q) { return q * f; }
    inline Float dot(const Quaternion &q1, const Quaternion &q2) {
        return q1.v * q2.v + q1.w * q2.w;
    }


    /* Animated Transform */
    class AnimatedTransform {
    private:
        const Float start_time, end_time;
        const Transform *start_transform, *end_transform;
        const bool actually_animated;
        Vector T[2];
        Quaternion R[2];
        Matrix4x4 S[2];
    public:
        AnimatedTransform(
            const Transform *transform1, Float time1,
            const Transform *transform2, Float time2):
            start_time(time1), end_time(time2),
            start_transform(transform1), end_transform(transform2),
            actually_animated(*start_transform != *end_transform) {
            decompose(start_transform->m, &T[0], &R[0], &S[0]);
            decompose(end_transform->m, &T[1], &R[1], &S[1]);
        }

        // M = TRS.
        void decompose(const Matrix4x4 &m, Vector *translate, Quaternion *rotate, Matrix4x4 *scale);
        void interpolate(Float time, Transform *t) const;

        void operator()(const Ray &r, Ray *tr) const;
        Ray operator()(const Ray &r) const;
        void operator()(const RayDifferential &r, RayDifferential *tr) const;
        RayDifferential operator()(const RayDifferential &r) const;
        Point operator()(Float time, const Point &p) const;
        Vector operator()(Float time, const Vector &v) const;

        BBox motion_bounds(const BBox &b, bool use_inverse) const;
    };
}

#endif

