/* -----------------------------
* File   : transform.cpp
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/

#include <transform.h>

namespace Yuki {
    

    Matrix4x4 transpose(const Matrix4x4 &m) {
        return Matrix4x4(m.m);
    }

    Matrix4x4 mul(const Matrix4x4 &m1, const Matrix4x4 &m2) {
        Matrix4x4 r;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                r.m[i][j] =
                    m1.m[i][0] * m2.m[0][j] +
                    m1.m[i][1] * m2.m[1][j] +
                    m1.m[i][2] * m2.m[2][j] +
                    m1.m[i][3] * m2.m[3][j];
            }
        }
        return r;
    }

    // inverse 'in',  n x n
    void inverse(Matrix4x4 &in, Matrix4x4 &out, int n = 4) {
        memset(out.m, 0, sizeof(Float) * n * n);
        for (int i = 0; i < n; i++) {
            out.m[i][i] = 1;
        }
        for (int i = 0; i < n; i++) {
            // make sure (i, i) != 0
            if (fabs(in.m[i][i]) < 1e-10f) {
                bool has_non_zero = false;
                for (int j = i + 1; j < n; j++) {
                    if (!equal_zero(in.m[j][i])) {
                        has_non_zero = true;
                        // add j-th row to i-th row
                        for (int k = 0; k < n; k++) {
                            in.m[i][k] += in.m[j][k];
                            out.m[i][k] += out.m[j][k];
                        }
                        break;
                    }
                }
            }
            // divide a(i, i)
            Float x = in.m[i][i];
            for (int j = 0; j < n; j++) {
                in.m[i][j]	/= x;
                out.m[i][j] /= x;
            }
            // substruct
            for (int j = i + 1; j < n; j++) {
                if (!equal_zero(in.m[j][i])) {
                    Float s = in.m[j][i];
                    for (int k = 0; k < n; k++) {
                        in.m[j][k]	-= s * in.m[i][k];
                        out.m[j][k] -= s * out.m[i][k];
                    }
                }
            }
        }
        for (int i = n - 2; i >= 0; i--) {
            for (int j = i + 1; j < n; j++) {
                Float s = in.m[i][j];
                for (int k = 0; k < n; k++) {
                    in.m[i][k] -= s * in.m[j][k];
                    out.m[i][k] -= s * out.m[j][k];
                }
            }
        }
    }

    Matrix4x4 inverse(const Matrix4x4 &m) {
        Matrix4x4 ret;
        Matrix4x4 input(m);
        inverse(input, ret, 4);
        return ret;
    }

#ifdef DEBUG
    std::ostream &operator<<(std::ostream &out, const Matrix4x4 &mat) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                if (equal_zero(mat.m[i][j])) {
                    out << 0 << ",\t";
                }
                else {
                    out << mat.m[i][j] << ",\t";
                }
            }
            out << mat.m[i][3] << std::endl;
        }
        return out;
    }
#endif // DEBUG

    /* Transform */

    Transform inverse(const Transform &t) {
        return Transform(t.m_inv, t.m);
    }
#ifdef DEBUG
    std::ostream &operator<<(std::ostream &out, const Transform &t) {
        out << t.m;
        return out;
    }
#endif // DEBUG



    /* Quaternion */
    Quaternion::Quaternion(Float a, Float b, Float c, Float d) :
        v(a, b, c), w(d) {
    }

    Quaternion::Quaternion(const Vector &axis, Float angle, bool is_radians) {
        v = axis.normalized();
        Float a = angle / 2.f;
        Float sin_t, cos_t;
        if (is_radians) {
            sin_t = sin(a);
            cos_t = cos(a);
        }
        else {
            sin_t = sin(radians(a));
            cos_t = cos(radians(a));
        }
        v *= sin_t;
        w = cos_t;
    }

    Quaternion::Quaternion(const Transform &t) {
        const Matrix4x4 &m = t.m;
        Float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        Float s;
        if (trace > 0.f) {
            // Compute w from matrix trace, then xyz
            // 4w^2 = m[0][0] + m[1][1] + m[2][2] + m[3][3] (but m[3][3] == 1)
            s = (Float)std::sqrt(trace + 1.0f);
            w = s / 2.0f;
            s = 0.5f / s;
            v.x = (m.m[2][1] - m.m[1][2]) * s;
            v.y = (m.m[0][2] - m.m[2][0]) * s;
            v.z = (m.m[1][0] - m.m[0][1]) * s;
        }
        else {
            int h = 0;
            if (m.m[1][1] > m.m[h][h]) h = 1;
            if (m.m[2][2] > m.m[h][h]) h = 2;
            switch (h) {
#define caseMacro(i, j, k, I, J, K) \
			case I:\
				s = (Float)std::sqrt((m.m[I][I] - (m.m[J][J] + m.m[K][K])) + m.m[3][3]);\
				v.i = s * 0.5f;\
				s = 0.5f / s;\
				v.j = (m.m[I][J] + m.m[J][I]) * s;\
				v.k = (m.m[K][I] + m.m[I][K]) * s;\
				w = (m.m[K][J] - m.m[J][K]) * s;\
				break
                caseMacro(x, y, z, 0, 1, 2);
                caseMacro(y, z, x, 1, 2, 0);
                caseMacro(z, x, y, 2, 0, 1);
#undef caseMacro
            }
        }
    }

    Transform Quaternion::to_transform() const {
        Float m[4][4];
        Float s = 1.0f / this->norm();
        Float xx = v.x * v.x * s, yy = v.y * v.y * s, zz = v.z * v.z * s;
        Float xy = v.x * v.y * s, xz = v.x * v.z * s, yz = v.y * v.z * s;
        Float wx = v.x * w * s, wy = v.y * w * s, wz = v.z * w * s;

        m[0][0] = 1 - 2 * (yy + zz);
        m[1][0] = 2 * (xy + wz);
        m[2][0] = 2 * (xz - wy);
        m[3][0] = 0;

        m[0][1] = 2 * (xy - wz);
        m[1][1] = 1 - 2 * (xx + zz);
        m[2][1] = 2 * (yz + wx);
        m[3][1] = 0;

        m[0][2] = 2 * (xz + wy);
        m[1][2] = 2 * (yz - wx);
        m[2][2] = 1 - 2 * (xx + yy);
        m[3][2] = 0;

        m[0][3] = 0;
        m[1][3] = 0;
        m[2][3] = 0;
        m[3][3] = 1;
        Matrix4x4 mat(m);
        return Transform(mat, transpose(mat));
    }

    Quaternion slerp(Float t, const Quaternion &q1, const Quaternion &q2) {
        Float cos_theta = dot(q1, q2);
        if (cos_theta > .99995f)
            return ((1.f - t) * q1 + t * q2).normalized();
        else {
            Float theta = (Float)std::acos(clamp(cos_theta, -1.f, 1.f));
            Float thetap = theta * t;
            Quaternion qperp = (q2 - q1 * cos_theta).normalized();
            return q1 * (Float)std::cos(thetap) + qperp * (Float)std::sin(thetap);
        }
    }

    std::ostream &operator<<(std::ostream &out, const Quaternion &q) {
        out << q.v.x << " " << q.v.y << " " << q.v.z << " " << q.w;
        return out;
    }

    // M = TRS
    void AnimatedTransform::decompose(const Matrix4x4 &m, Vector *translate, Quaternion *rotate, Matrix4x4 *scale) {
        // 1. seprate the translation
        translate->x = m.m[0][3];
        translate->y = m.m[1][3];
        translate->z = m.m[2][3];

        // 2. seprate the rotation by polar decomposition.
        Matrix4x4 M = m;
        for (int i = 0; i < 3; ++i) M.m[i][3] = M.m[3][i] = 0.f;
        M.m[3][3] = 1.f;

        Float norm;
        int count = 0;
        Matrix4x4 R = M;
        do {
            Matrix4x4 R_next;
            Matrix4x4 R_it = inverse(transpose(R));
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    R_next.m[i][j] = 0.5f * (R.m[i][j] + R_it.m[i][j]);
                }
            }
            norm = 0.f;
            for (int i = 0; i < 3; ++i) {
                Float n =
                    std::abs(R.m[i][0] - R_next.m[i][0]) +
                    std::abs(R.m[i][1] - R_next.m[i][1]) +
                    std::abs(R.m[i][2] - R_next.m[i][2]);
                norm = std::max(norm, n);
            }
            R = R_next;
        } while (++count < 100 && norm > 0.0001f);
        *rotate = Quaternion(R);

        // 3. finally, get the scale
        *S = mul(inverse(R), M);
    }

    void AnimatedTransform::interpolate(Float time, Transform *t) const {
        // 1. handle boundary conditions
        if (!actually_animated || time <= start_time) {
            *t = *start_transform;
            return;
        }
        if (time >= end_time) {
            *t = *end_transform;
            return;
        }

        // 2. other conditions
        Float dt = (time - start_time) / (end_time - start_time);
        // a. interpolate translation
        Vector translation = (1.f - dt) * T[0] + dt * T[1];
        // b. interpolate rotation
        Quaternion rotation = slerp(dt, R[0], R[1]);
        // c. interpolate scale
        Matrix4x4 scale;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                scale.m[i][j] = lerp(dt, S[0].m[i][j], S[1].m[i][j]);
            }
        }
        Transform trans;
        trans.translate(translation);
        *t = trans * rotation.to_transform() * Transform(scale);
    }

    void AnimatedTransform::operator()(const Ray &r, Ray *tr) const {
        if (!actually_animated || r.time <= start_time) {
            *tr = (*start_transform)(r);
        }
        else if (r.time >= end_time) {
            *tr = (*end_transform)(r);
        }
        else {
            Transform t;
            interpolate(r.time, &t);
            *tr = t(r);
        }
    }

    Ray AnimatedTransform::operator()(const Ray &r) const {
        if (!actually_animated || r.time <= start_time) {
            return (*start_transform)(r);
        }
        else if (r.time >= end_time) {
            return (*end_transform)(r);
        }
        else {
            Transform t;
            interpolate(r.time, &t);
            return t(r);
        }
    }

    void AnimatedTransform::operator()(const RayDifferential &r, RayDifferential *tr) const {
        if (!actually_animated || r.time <= start_time) {
            *tr = (*start_transform)(r);
        }
        else if (r.time >= end_time) {
            *tr = (*end_transform)(r);
        }
        else {
            Transform t;
            interpolate(r.time, &t);
            *tr = t(r);
        }
    }

    RayDifferential AnimatedTransform::operator()(const RayDifferential &r) const {
        if (!actually_animated || r.time <= start_time) {
            return (*start_transform)(r);
        }
        else if (r.time >= end_time) {
            return (*end_transform)(r);
        }
        else {
            Transform t;
            interpolate(r.time, &t);
            return t(r);
        }
    }

    Point AnimatedTransform::operator()(Float time, const Point &p) const {
        if (!actually_animated || time <= start_time) {
            return (*start_transform)(p);
        }
        else if (time >= end_time) {
            return (*end_transform)(p);
        }
        else {
            Transform t;
            interpolate(time, &t);
            return t(p);
        }
    }

    Vector AnimatedTransform::operator()(Float time, const Vector &v) const {
        if (!actually_animated || time <= start_time) {
            return (*start_transform)(v);
        }
        else if (time >= end_time) {
            return (*end_transform)(v);
        }
        else {
            Transform t;
            interpolate(time, &t);
            return t(v);
        }
    }

    BBox AnimatedTransform::motion_bounds(const BBox &b, bool use_inverse = false) const {
        if (!actually_animated) {
            if (use_inverse)
                return inverse(*start_transform)(b);
            else
                return (*start_transform)(b);
        }
        BBox ret;
        const int n_steps = 128;
        for (int i = 0; i < n_steps; ++i) {
            Transform t;
            Float time = lerp(Float(i) / Float(n_steps - 1), start_time, end_time);
            interpolate(time, &t);
            if (use_inverse) t = inverse(t);
            ret = union_BBox(ret, t(b));
        }
        return ret;
    }

}
