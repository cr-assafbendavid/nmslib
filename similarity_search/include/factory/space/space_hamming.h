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

#ifndef FACTORY_SPACE_HAMMING_H
#define FACTORY_SPACE_HAMMING_H

#include <params.h>
#include <space/space_hamming.h>
#include <space/space_vector_gen.h>
#include <space/space_vector_gen_mmap.h>


namespace similarity {

Space<float>* CreateHamming(const AnyParams& AllParams) {
  AnyParamManager pmgr(AllParams);

  string filename;
  size_t element_size, max_elements;

  pmgr.GetParamOptional("mmap_filename", filename, "");
  pmgr.GetParamOptional("element_size", element_size, 0);
  pmgr.GetParamOptional("max_elements", max_elements, 0);
  pmgr.CheckUnused();

  if (filename.empty())
    return new VectorSpaceGen<float, HammingDist>();
  return new VectorSpaceGenMMap<float, HammingDist>(filename, element_size, max_elements);
}

}  // namespace similarity

#endif 
