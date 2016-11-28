/* -----------------------------
* efloat.h
* Author: Yuki Chai
* 2016.11.28
* Project Yuki
*/

#pragma once
#ifndef __YUKI_EFLOAT_H__
#define __YUKI_EFLOAT_H__

#include <core.h>
#include <log.h>
 
namespace Yuki {
    class EFloat {
    private:
        Float v, low, high;
#ifdef DEBUG
        long double v_precise;
#endif
    public:
        EFloat() {}
        EFloat(float v, float err = 0.f) : v(v) {
            if (err == 0.) {
                low = high = v;
            }
            else {
                low = next_float_down(v - err);
                high = next_float_up(v + err);
            }
#ifdef DEBUG
            v_precise = v;
            check();
#endif
        }
#ifdef DEBUG
        EFloat(float v, long double ld, float err) : EFloat(v, err) {
            v_precise = ld;
            check();
        }
#endif
        explicit operator float() const { return v; }
        explicit operator double() const { return v; }
        Float get_absolute_error() const { return high - low; }
#ifdef DEBUG
        Float get_relative_error() const {
            return std::abs((v_precise - v) / v_precise);
        }
        long double precise_value() const { return v_precise; }
#endif
        Float lower_bound() const { return low; }
        Float upper_bound() const { return high; }

        EFloat operator+(const EFloat &ef) const {
            EFloat r;
            r.v = v + ef.v;
#ifdef DEBUG
            r.v_precise = v_precise + ef.v_precise;
#endif
            r.low  = next_float_down(lower_bound() + ef.lower_bound());
            r.high = next_float_up  (upper_bound() + ef.upper_bound());
            r.check();
            return r;
        }

        EFloat operator-(const EFloat &ef) const {
            EFloat r;
            r.v = v - ef.v;
#ifdef DEBUG
            r.v_precise = v_precise - ef.v_precise;
#endif
            r.low  = next_float_down(lower_bound() - ef.upper_bound());
            r.high = next_float_up  (upper_bound() - ef.lower_bound());
            r.check();
            return r;
        }

        EFloat operator/(const EFloat &ef) const {
            EFloat r;
            r.v = v / ef.v;
#ifdef DEBUG
            r.v_precise = v_precise / ef.v_precise;
#endif
            if (ef.low < 0 && ef.high > 0) {
                r.low = -Infinity;
                r.high = Infinity;
            }
            else {
                Float div[4] = {
                    lower_bound() / ef.lower_bound(), upper_bound() / ef.lower_bound(),
                    lower_bound() / ef.upper_bound(), upper_bound() / ef.upper_bound()};
                r.low = next_float_down(
                    std::min(std::min(div[0], div[1]), std::min(div[2], div[3])));
                r.high = next_float_up(
                    std::max(std::max(div[0], div[1]), std::max(div[2], div[3])));
            }
            r.check();
            return r;
        }

        inline void check() {
            if (!std::isinf(low) && !std::isnan(low) && 
                !std::isinf(high) && !std::isnan(high))
                CHECK_LE(low, high);
#ifdef DEBUG
            if (!std::isinf(v) && !std::isnan(v)) {
                CHECK_LE(lower_bound(), v_precise);
                CHECK_LE(v_precise, upper_bound());
            }
#endif
        }
    };
}

#endif  // !__YUKI_EFLOAT_H__