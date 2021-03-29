#ifndef TYPE_TRAITS_HPP
#define TYPE_TRAITS_HPP

#include <type_traits>

namespace pcl_ros_wrapper {

// This is fine to use in an unevaluated context
template<class T> T& reference_to();

// Case where MaybePointer is not a pointer
template<class MaybePointer> auto element_type(char) -> MaybePointer;

// This one is the preferred one, where it might be a container (?)
template<class MaybePointer>
auto element_type(int) -> typename MaybePointer::element_type;

// This one is the fallback if the preferred one doesn't work
template<class MaybePointer>
auto element_type(short) ->
  typename std::remove_reference<decltype(*reference_to<MaybePointer>())>::type;

// We alias the return type
template<class T> using element_type_t = decltype(element_type<T>(0));

// Remove const
template<typename T> using RemoveConst = typename std::remove_const<T>::type;

}// namespace pcl_ros_wrapper

#endif /* TYPE_TRAITS_HPP */