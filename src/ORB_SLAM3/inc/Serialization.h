#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive& ar, DBoW2::BowVector& bv, const unsigned int version) {
    ar & boost::serialization::base_object<std::map<DBoW2::WordId, DBoW2::WordValue>>(bv);
}

template<class Archive>
void serialize(Archive& ar, DBoW2::FeatureVector& fv, const unsigned int version) {
    ar & boost::serialization::base_object<std::map<DBoW2::NodeId, std::vector<unsigned int>>>(fv);
}

} // namespace serialization
} // namespace boost

#endif // SERIALIZATION_H