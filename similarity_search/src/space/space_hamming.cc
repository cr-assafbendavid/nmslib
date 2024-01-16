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
#include "space/space_hamming.h"
#include "space/space_vector_gen.h"
#include "space/space_vector_gen_mmap.h"

namespace similarity {

float HammingDist::operator()(const float* x, const float* y, size_t qty) const {
    const auto* xx = reinterpret_cast<const uint32_t*>(x);
    const auto* yy = reinterpret_cast<const uint32_t*>(y);
    int n_different = 0;
    for (size_t i = 0; i < qty; ++i)
        n_different += (int)(*xx++ != *yy++);
    return ((float)n_different) / qty;
}

template class VectorSpaceGen<float, HammingDist>;
template class VectorSpaceGenMMap<float, HammingDist>;

}  // namespace similarity
