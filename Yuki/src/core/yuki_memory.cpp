/* -----------------------------
* yuki_memory.cpp
* Author: Yuki Chai
* 2016.11.28
* Project Yuki
*/
#include "yuki_memory.h"

namespace Yuki {
    void *_aligned_malloc(size_t size, int alignment) {
        if (alignment & (alignment - 1)) {
            return NULL;
        }
        const int pointer_size = sizeof(void *);
        const int required_size = (int)size + alignment - 1 + pointer_size;
        const uintptr_t alignment_1 = alignment - 1;

        void *raw = malloc(required_size);
        uintptr_t start = (uintptr_t)raw + (uintptr_t)pointer_size;
        uintptr_t aligned = (start + alignment_1) & ~(alignment_1);

        // store the raw address for free
        *((void **)(aligned - pointer_size)) = raw;

        return (void *)aligned;
    }

    void _aligned_free(void *ptr) {
        void *raw = *((void **)( (uintptr_t)ptr - (uintptr_t)sizeof(void *) ));
        free(raw);
        return;
    }
}
