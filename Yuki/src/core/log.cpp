/* -----------------------------
 * log.cpp
 * Author: Yuki Chai
 * 2016.11.28
 * Project Yuki
 *-------------------------------
 */
#include "log.h"

namespace Yuki {
    LOG *LOG::instance = NULL;
    const char * _operations[] = {
        " <= ",
        " < ",
        " >= ",
        " > ",
        " == ",
        " != "
    };
}