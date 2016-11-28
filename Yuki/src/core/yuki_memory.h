/* -----------------------------
* File   : yuki_memory.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_MEMORY_H__
#define __YUKI_MEMORY_H__

#include "core.h"
#include <vector>
using namespace std;

namespace Yuki {
    void *_aligned_malloc(size_t size, int alignment);
    void _aligned_free(void *ptr);

    template <typename T>
    inline T *aligned_malloc(size_t size, int alignment = PBRT_L1_CACHE_SIZE) {
        return (T *)_aligned_malloc(sizeof(T) * size, alignment);
    }

    template <typename T>
    inline void aligned_free(T *ptr) {
        _aligned_free((void *)ptr);
        return;
    }

    struct Block {
        byte        *block;
        uint32_t    block_size;
        Block(uint32_t bs = 32768) {
            block_size = bs;
            block = aligned_malloc<byte>(block_size);
        }
    };

    class MemoryArena {
    private:
        Block       *cur_block;
        uintptr_t   cur_block_pos;
        uint32_t    default_block_size;
        vector<Block *> used_blocks, available_blocks;
    public:
        MemoryArena(uint32_t block_size = 32768) {
            default_block_size = block_size;
            cur_block_pos = 0;
            cur_block = new Block(default_block_size);
        }

        void *alloc(uint32_t size) {
            // round size to minimum machine alignment
            size = ((size + BLOCK_MIN_ALIGN - 1) & (~(BLOCK_MIN_ALIGN - 1)));
            if (cur_block_pos + size > cur_block->block_size) {
                // get new block of memory
                used_blocks.push_back(cur_block);
                if (available_blocks.size() && (available_blocks.back()->block_size >= size)) {
                    // the back of the available_blocks is useful
                    cur_block = available_blocks.back();
                    available_blocks.pop_back();
                }
                else {
                    cur_block = new Block(max(size, default_block_size));
                }
                cur_block_pos = 0;
            }
            void *ret = (void *)((uintptr_t)cur_block->block + cur_block_pos);
            cur_block_pos += size;
            return ret;
        }

        template <typename T>
        T *alloc(uint32_t count = 1) {
            T *ret = (T *)this->alloc(count * sizeof(T));
            for (uint32_t i = 0; i < count; ++i) {
                new (&ret[i]) T();
            }
            return ret;
        }

        void free_all() {
            cur_block_pos = 0;
            while (used_blocks.size()) {
                available_blocks.push_back(used_blocks.back());
                used_blocks.pop_back();
            }
        }
    };

    // row first blocked array
    template <class T, int log_block_size = 2>
    class BlockedArray {
    private:
        T *data;
        int rows_;
        int cols_;
        int col_blocks_;
        int block_size_;
        int block_size_2;
    public:
        BlockedArray(int rows, int cols, T *raw = NULL) {
            block_size_ = 1 << log_block_size;
            block_size_2 = block_size_ * block_size_;
            rows_ = rows;
            cols_ = cols;
            col_blocks_ = round_up(cols) >> log_block_size;
            uint32_t n_alloc = round_up(rows) * round_up(cols);
            data = aligned_malloc<T>(n_alloc);
            for (uint32_t i = 0; i < n_alloc; ++i) {
                new (&data[i]) T();
            }
            if (raw) {
                for (uint32_t r = 0; r < rows_; ++r) {
                    for (uint32_t c = 0; c < cols; ++c) {
                        (*this)(r, c) = raw[r * cols + c];
                    }
                }
            }
        }

        uint32_t block_size() const { return block_size_; }
        uint32_t round_up(int x) const { 
            return (x + block_size_ - 1) & 
                ~(block_size_ - 1);
        }
        uint32_t rows() const { return rows_; }
        uint32_t cols() const { return cols_; }
        uint32_t block(uint32_t a) const {
            return a >> log_block_size;
        }
        uint32_t offset(uint32_t a) const {
            return a & (block_size_ - 1);
        }
        T &operator()(uint32_t row, uint32_t col) {
            uint32_t b_row = block(row),
                b_col = block(col);
            uint32_t o_row = offset(row),
                o_col = offset(col);
            uint32_t off = block_size_2 * (col_blocks_ * b_row + b_col);
            off += block_size_ * o_row + o_col;
            return data[off];
        }
    };

}

#endif  // !__YUKI_MEMORY_H__