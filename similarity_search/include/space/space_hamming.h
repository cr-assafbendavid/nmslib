/**
 * Non-metric Space Library
 *
 * Main developers: Bilegsaikhan Naidan, Leonid Boytsov, Yury Malkov, Ben Frederickson, David Novak
 *
 * For the complete list of contributors and further details see:
 * https://github.com/searchivarius/NonMetricSpaceLib
 *
 * Copyright (c) 2013-2018
 *
 * This code is released under the
 * Apache License Version 2.0 http://www.apache.org/licenses/.
 *
 */
#ifndef _SPACE_HAMMING_H_
#define _SPACE_HAMMING_H_

#include <cstddef>

#define SPACE_HAMMING "hamming"

namespace similarity {

struct HammingDist {
    float operator()(const float* x, const float* y, size_t qty) const;
};

}  // namespace similarity

#endif 
