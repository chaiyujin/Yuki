#include <log.h>
#include <core.h>
#include <yuki_memory.h>
#include <iostream>
#include <efloat.h>
using namespace std;
using namespace Yuki;

int main(int argc, char* argv[]) {
    EFloat x(10);
    EFloat y(20);
    cout << (x / y).precise_value() << endl;
    system("pause");
    return 0;
}