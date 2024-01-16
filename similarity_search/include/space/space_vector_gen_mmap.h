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
#ifndef _SPACE_VECTOR_GEN_MMAP_H_
#define _SPACE_VECTOR_GEN_MMAP_H_

#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <string>

#include "idtype.h"
#include "object.h"
#include "space/space_vector_gen.h"

#define ALIGN_TO_PAGE(x) ((x) & ~(getpagesize() - 1))
#define UPPER_ALIGN_TO_PAGE(x) ALIGN_TO_PAGE((x)+(getpagesize()-1))


namespace similarity {

typedef unsigned long long element_count_t;

const char MMAP_MAGIC[8] = {'M', 'M', 'A', 'P', 'F', 'I', 'L', 'E'};
const size_t HEADER_SIZE = sizeof(MMAP_MAGIC) + sizeof(size_t) + sizeof(element_count_t);

template <typename dist_t, typename DistObjType>
class VectorSpaceGenMMap : public VectorSpaceGen<dist_t, DistObjType> {
public:
  explicit VectorSpaceGenMMap(const string &filename, size_t element_size, size_t max_elements = 0):
  fd_(-1),
  mmap_(MAP_FAILED)
  {

    if (open_file(filename)) {
      data_size_ = lseek(fd_, 0, SEEK_END);
      data_size_ = ALIGN_TO_PAGE(data_size_);
      object_size_ = element_size_ + ID_SIZE + LABEL_SIZE + DATALENGTH_SIZE;
      max_elements_ = (data_size_ - HEADER_SIZE) / object_size_;
    } else {
      element_size_ = element_size * sizeof(dist_t);
      object_size_ = element_size_ + ID_SIZE + LABEL_SIZE + DATALENGTH_SIZE;
      fd_ = open(filename.c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
      CHECK(fd_ >= 0);
      CHECK(write(fd_, MMAP_MAGIC, sizeof(MMAP_MAGIC)) == sizeof(MMAP_MAGIC));
      CHECK(write(fd_, &element_size_, sizeof(size_t)) == sizeof(size_t));
      element_count_t temp = 0;
      CHECK(write(fd_, &temp, sizeof(element_count_t)) == sizeof(element_count_t));
      data_size_ = HEADER_SIZE + object_size_ * max_elements;
      data_size_ = UPPER_ALIGN_TO_PAGE(data_size_);
      CHECK(ftruncate(fd_, data_size_) == 0);
      max_elements_ = max_elements;
    }

    lseek(fd_, 0, SEEK_SET);
    mmap_ = mmap(NULL, data_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mmap_ == MAP_FAILED) {
      close(fd_);
      fd_ = -1;
      CHECK(mmap_ != MAP_FAILED);
    }

    n_elements_ = reinterpret_cast<element_count_t *>(
            reinterpret_cast<char *>(mmap_) + HEADER_SIZE - sizeof(element_count_t)
    );
  }

  virtual ~VectorSpaceGenMMap() {
    if (mmap_ != MAP_FAILED)
      munmap(mmap_, data_size_);
    if (fd_ >= 0)
      close(fd_);
  }

  virtual std::string StrDesc() const {
    return "memmapped custom space";
  }

  virtual Object* CreateObjFromVect(IdType id, LabelType label, const std::vector<dist_t>& InpVect) const {
    if (label != -2)
      return VectorSpaceGen<dist_t, DistObjType>::CreateObjFromVect(id, label, InpVect);
    CHECK(*n_elements_ < max_elements_);
    CHECK(InpVect.size() * sizeof(dist_t) == element_size_);
    char *buffer = reinterpret_cast<char *>(mmap_) + HEADER_SIZE + object_size_ * (*n_elements_);
    (*n_elements_)++;
    return new Object(buffer, id, label, element_size_, &InpVect[0]);
  }

    virtual Object* CreateObjFromUint32Vect(IdType id, LabelType label, const std::vector<uint32_t>& InpVect) const {
        if (label != -2)
            return VectorSpaceGen<dist_t, DistObjType>::CreateObjFromUint32Vect(id, label, InpVect);
        CHECK(*n_elements_ < max_elements_);
        CHECK(InpVect.size() * sizeof(uint32_t) == element_size_); // luckily, float and uint32 are both the same size...
        char *buffer = reinterpret_cast<char *>(mmap_) + HEADER_SIZE + object_size_ * (*n_elements_);
        (*n_elements_)++;
        return new Object(buffer, id, label, element_size_, &InpVect[0]);
    }

  virtual unique_ptr<DataFileInputState> ReadObjectVectorFromBinData(ObjectVector& data,
                                                                     vector<string>& vExternIds,
                                                                     const std::string& inputFile,
                                                                     const IdTypeUnsign maxQty=MAX_DATASET_QTY) const {
    CHECK_MSG(data.empty(), "this function expects data to be empty on call");
    vExternIds.clear();
    char *buffer = reinterpret_cast<char *>(mmap_) + HEADER_SIZE;

    for (size_t i = 0; i < *n_elements_; ++i) {
      data.push_back(new Object(buffer, false));
      buffer += object_size_;
    }

    return unique_ptr<DataFileInputState>(new DataFileInputState());
  }

  virtual void WriteObjectVectorBinData(const ObjectVector& data,
                                        const vector<string>& vExternIds,
                                        const std::string& outputFile,
                                        const IdTypeUnsign MaxNumObjects = MAX_DATASET_QTY) const {}

protected:
  DISABLE_COPY_AND_ASSIGN(VectorSpaceGenMMap);

private:
  int fd_;
  void *mmap_;
  size_t max_elements_, data_size_, object_size_, element_size_;
  element_count_t *n_elements_;

  bool open_file(const string &filename) {
    struct stat statbuffer;
    if (stat(filename.c_str(), &statbuffer) != 0) // file doesn't exist
      return false;
    fd_ = open(filename.c_str(), O_RDWR);
    CHECK(fd_ >= 0);
    char buffer[sizeof(MMAP_MAGIC)];
    if (read(fd_, buffer, sizeof(MMAP_MAGIC)) != sizeof(MMAP_MAGIC))
      goto fail;
    if (memcmp(MMAP_MAGIC, buffer, sizeof(MMAP_MAGIC)) != 0)
      goto fail;
    if (read(fd_, &element_size_, sizeof(size_t)) != sizeof(size_t))
      goto fail;
    element_count_t temp_n_elements;
    if (read(fd_, &temp_n_elements, sizeof(element_count_t)) != sizeof(element_count_t))
      goto fail;
    return true;
  fail:
    close(fd_);
    fd_ = -1;
    return false;
  }

};

}  // namespace similarity

#endif
