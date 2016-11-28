#include <log.h>
#include <iostream>
using namespace std;
using namespace Yuki;

int main(int argc, char* argv[]) {
    struct Test {
        float v;
        float value() {return v;}
    } x, y;
    x.v = 21;
    y.v = 20;
    CHECK_LE(x, y);
    system("pause");
    return 0;
}