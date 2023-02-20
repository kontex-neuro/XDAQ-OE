#include <span>
#include <vector>

#include "rhythm-api/rhd2000datablock.h"
int main()
{
    std::vector<unsigned char> buffer(1024 * 1024);
    Rhd2000DataBlock db(3, 128, false, &buffer[0]);
    std::vector<float> target;
    auto dv = Rhd2000DataBlockView(&target[0], &target[128], 2, 128, false, &buffer[0]);
}