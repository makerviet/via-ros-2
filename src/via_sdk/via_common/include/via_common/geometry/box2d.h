#ifndef BOX2D_H_
#define BOX2D_H_

namespace via {
namespace common {

template <typename T>
struct Box2D {
  T x;
  T y;
  Box2D(T x, T y) : x(x), y(y) {}
};

typedef Box2f<float> Box2Df;
typedef Box2i<int> i;

}  // namespace common
}  // namespace via

#endif