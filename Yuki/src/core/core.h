/* -----------------------------
* File   : core.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_CORE_H__
#define __YUKI_CORE_H__

// Global define here
#ifdef _DEBUG
#define DEBUG
#endif

#define INV_PI                  0.31830988618379067154f
#define INV_TWOPI               0.15915494309189533577f
#define INV_FOURPI              0.07957747154594766788f
#define _USE_MATH_DEFINES
#define PBRT_L1_CACHE_SIZE      16
#define BLOCK_MIN_ALIGN         16
#define EPS                     1e-5

// Global typedef here
typedef unsigned char byte;
typedef double Float;

// Global include std files here
#include <algorithm>
#include <limits>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <memory>
using std::shared_ptr;

#include <iostream>
using std::cout;
using std::endl;
using std::ostream;

namespace Yuki {
    // Global simple functions

    template <class T, class U>
    inline T clamp(T x, U l, U r) {
        if (x < l) return T(l);
        if (x > r) return T(r);
        return x;
    }

    template <class T>
    inline T lerp(Float t, T v1, T v2) {
        return T((1.f - t) * v1 + t * v2);
    }

    template <class T>
    inline bool equal_zero(T x) {
        if (fabs(x) < EPS) return true;
        return false;
    }

    inline Float radians(Float x) {
        return Float(x / 180.f * M_PI);
    }

    // float
#define MachineEpsilon (std::numeric_limits<Float>::epsilon() * 0.5)
#define Infinity       std::numeric_limits<Float>::infinity()
    inline uint32_t float_to_bits(float f) {
        uint32_t ui;
        memcpy(&ui, &f, sizeof(float));
        return ui;
    }

    inline float bits_to_float(uint32_t ui) {
        float f;
        memcpy(&f, &ui, sizeof(uint32_t));
        return f;
    }

    inline uint64_t float_to_bits(double f) {
        uint64_t ui;
        memcpy(&ui, &f, sizeof(double));
        return ui;
    }

    inline double bits_to_float(uint64_t ui) {
        double f;
        memcpy(&f, &ui, sizeof(uint64_t));
        return f;
    }

    inline float next_float_up(float v) {
        if (std::isinf(v) && v > 0.) return v;
        if (v == -0.f) v = 0.f;

        uint32_t ui = float_to_bits(v);
        if (v >= 0) ++ui;
        else        --ui;
        return bits_to_float(ui);
    }

    inline float next_float_down(float v) {
        if (std::isinf(v) && v < 0.) return v;
        if (v == 0.f) v = -0.f;
        uint32_t ui = float_to_bits(v);
        if (v > 0) --ui;
        else       ++ui;
        return bits_to_float(ui);
    }

    inline double next_float_up(double v, int delta = 1) {
        if (std::isinf(v) && v > 0.) return v;
        if (v == -0.f) v = 0.f;

        uint64_t ui = float_to_bits(v);
        if (v >= 0) ui += delta;
        else        ui -= delta;
        return bits_to_float(ui);
    }

    inline double next_float_down(double v, int delta = 1) {
        if (std::isinf(v) && v < 0.) return v;
        if (v == 0.f) v = -0.f;
        uint64_t ui = float_to_bits(v);
        if (v > 0) ui -= delta;
        else       ui += delta;
        return bits_to_float(ui);
    }

    inline Float epsilon_gamma(int n) {
        return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
    }

    inline Float gamma_correct(Float value) {
        if (value <= 0.0031308f) return 12.92f * value;
        return 1.055f * std::pow(value, (Float)(1.f / 2.4f)) - 0.055f;
    }

    inline Float inverse_gamma_correct(Float value) {
        if (value <= 0.04045f) return value * 1.f / 12.92f;
        return std::pow((value + 0.055f) * 1.f / 1.055f, (Float)2.4f);
    } 

    // all core classes here
    class Vector;
    class Point;
    class Normal;
    class BBox;
    class Shape;
    class DifferentialGeometry;
    class Transform;
    class Quaternion;
    class AnimatedTransform; 

    class Primitive;
    class Intersection;
}

#endif  // !__YUKI_CORE_H__