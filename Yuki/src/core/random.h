/* -----------------------------
* File   : random.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_RANDOM_H__
#define __YUKI_RANDOM_H__

#include <log.h>
#include <core.h>

#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#include <time.h>
#include <stdlib.h>

// necessary to undefine min, max
// conflict with std::min, std::max
#ifdef min
#undef min
#undef max
#endif

namespace Yuki {
    const DWORD rand_dw_length = 2;
    class Random {
    private:
        BYTE pb_buffer[rand_dw_length];
        HCRYPTPROV h_provider;
    public:
        Random() : h_provider(0){
            init();
        }
        ~Random() {
            close();
        }

        void init() {
            DWORD result = ::CryptAcquireContextW(
                &h_provider, 0, 0,
                PROV_RSA_FULL, 
                CRYPT_VERIFYCONTEXT | CRYPT_SILENT);
            CHECK(result != 0);
        }

        void close() {
            ::CryptReleaseContext(h_provider, 0);
        }

        Float random() {
            DWORD result = ::CryptGenRandom(h_provider, rand_dw_length, pb_buffer);
            Float x = *(unsigned short *)pb_buffer;
            return x / 65535.0f;
        }

        template <class T>
        T random(T scale) {
            return random() * scale;
        }
    };
}

// _WIN32 || _WIN64
#else // other system
#include <random>
using std::random_device;
using std::seed_seq;
using std::mt19937;
using std::uniform_real_distribution;

namespace KuroYuki {
    class Random {
    private:
        random_device   r;
        seed_seq        seed;
        mt19937         engine;
        uniform_real_distribution<> dist;
    public:
        Random() : r(), 
            seed({r(), r(), r(), r(), r()}),
            engine(seed),
            dist(0, 1) {}
        ~Random() {}

        Float random() {
            return dist(engine);
        }

        template <class T>
        T random(T scale) {
            return random() * scale;
        }
    };
}
#endif // other system

#endif // !__YUKI_RANDOM_H__
