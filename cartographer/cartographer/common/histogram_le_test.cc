#include "cartographer/common/math.h"

#include "cartographer/common/histogram.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(Hist, toString) {
 Histogram hist;
   hist.Add(2);
   hist.Add(1);
   hist.Add(2);
   hist.Add(4);
   hist.Add(5);
   hist.Add(5);
   hist.Add(5);
   hist.Add(6);
   hist.Add(7);
   hist.Add(8);
   std::cout<<hist.ToString(3)<<std::endl;
}
}
}
}

/*
输出信息:
Count: 10  Min: 1.000000  Max: 8.000000  Mean: 4.500000
[1.000000, 3.333333)                  ######    Count: 3 (30.000000%)   Total: 3 (30.000000%)
[3.333333, 5.666667)                ########    Count: 4 (40.000000%)   Total: 7 (70.000000%)
[5.666667, 8.000000]                  ######    Count: 3 (30.000000%)   Total: 10 (100.000000%)
*/