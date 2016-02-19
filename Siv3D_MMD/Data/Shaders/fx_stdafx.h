#ifndef _DEBUG
#define DEF(type,name,sem) type name : sem
#define SEMANTICS(sem) : sem
#else
#define CBUFFER_CAT(a) namespace {/##a
#define cbuffer CBUFFER_CAT(/)
#define DEF(type,name,sem) type name
#define discard return
#define SEMANTICS(sem) //
#define row_major
#ifdef NDEBUG
#define DEBUG_(a) /##a
#define DEBUG DEBUG_(/)
#else
#define DEBUG
#endif

template<class T> struct Temp1 {
  Temp1(...) {}
  T x;
  T &operator[](int i);
};
template<class T> T operator+(T, T) {}
template<class T> T operator+=(T, T) {}
template<class T> T operator*(T, T) {}
template<class T> T operator-(T, T) {}
template<class T> T operator/(T, T) {}
float4x4 operator*(float4, float4x4) {}
float4x4 operator*(float4x4, float4) {}
float2 operator*(float2, float) {}

template<class T> struct Temp2 :public Temp1<T> {
  Temp2(...) {}
  T y;
};
template<class T> struct Temp3 :public Temp2<T> {
  T z;
};
template<class T> struct Temp4 :public Temp3<T> {
  T w;
  Temp3 xyz;
  Temp4(...) {}
};
using int1 = Temp1<int>;
using int2 = Temp2<int>;
using int3 = Temp3<int>;
using int4 = Temp4<int>;
using uint = Temp1<unsigned int>;
using float1 = Temp1<float>;
using float2 = Temp2<float>;
using float3 = Temp3<float>;
using float4 = Temp4<float>;

template<class T> struct Temp2x2 {
  Temp2x2(...) {}
  Temp2<T> a[2];
  Temp2<T> &operator[](int i);
  Temp2<T> operator*(Temp2<T>) {}
};
template<class T> struct Temp4x4 {
  Temp4<T> a[4];
  Temp4<T> &operator[](int i);
};
using int4x4 = Temp4x4<int>;
using float2x2 = Temp2x2<float>;
using float4x4 = Temp4x4<float>;
struct SamplerState {
};
struct Texture2D {
  float4 Sample(SamplerState ss, float2 pos);
};

template<class T, class U> U mul(T, U) {}
template<class T> T cos(T) {}
template<class T> T sin(T) {}
template<class T> T normalize(T) {}
template<class T> T distance(T, T) {}
#endif
