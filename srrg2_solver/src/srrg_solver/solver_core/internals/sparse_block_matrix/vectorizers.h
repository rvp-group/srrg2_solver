#pragma once

#include <iostream>

#define DEBUG(var) \
  if (var)         \
  std::cerr

namespace vectorizer {

  static const int MaxVectorizerDim = 32;
  static const int vectorizer_debug = 0;

  typedef float afloat __attribute__((__aligned__(16)));
  using Float1Fn = int (*)(afloat* const __restrict__);
  using Float2Fn = int (*)(afloat* const __restrict__, const afloat* const __restrict__);

  using Float3Fn = int (*)(afloat* const __restrict__,
                           const afloat* const __restrict__,
                           const afloat* const __restrict__);

  template <int curr_idx>
  void setupIdxTable(int* table) {
  }

  template <int curr_idx, int idx, int... rest>
  void setupIdxTable(int* table) {
    if (idx < MaxVectorizerDim) {
      table[idx] = curr_idx;
    }
    setupIdxTable<curr_idx + 1, rest...>(table);
  }

  static int index_table[MaxVectorizerDim];

  static inline int getIndex(int idx) {
    if (idx >= MaxVectorizerDim) {
      return -1;
    }
    return index_table[idx];
  }

  template <typename Function_, int... Dimensions>
  struct Vectorizer1_ {
    static inline int f(afloat* const __restrict__ dest, int p1) {
      int idx = getIndex(p1);
      if (idx < 0) {
        return Function_::f(dest, p1);
      }
      return (*f_matrix[idx])(dest);
    }

    inline int operator()(afloat* const __restrict__ dest, int p1) {
      return f(dest, p1);
    }

    Vectorizer1_(const char* name_) {
      DEBUG(vectorizer_debug) << "Vectorizer1_ " << name_ << std::endl;
      memset(f_matrix, 0, sizeof(f_matrix));
      instantiate<'c', Dimensions...>();
    }

  protected:
    static constexpr int N = sizeof...(Dimensions);
    using FunctionMatrix   = Float1Fn[N];
    static FunctionMatrix f_matrix;
    // FunctionMatrix f_matrix;

    template <int p1>
    static int _vfn(afloat* const __restrict__ dest) {
      return Function_::f(dest, p1);
    }

    template <char c, int p1, int... rest>
    void instantiate() {
      int idx = getIndex(p1);
      if (idx < 0) {
        return;
      }
      f_matrix[idx] = this->_vfn<p1>;
      DEBUG(vectorizer_debug) << "<" << p1 << ">(" << idx << ") -> " << (void*) f_matrix[idx]
                              << std::endl;

      instantiate<c, rest...>();
    }

    template <char c>
    void instantiate() {
    }
  };

  template <typename Function_, int... Dimensions>
  typename Vectorizer1_<Function_, Dimensions...>::FunctionMatrix
    Vectorizer1_<Function_, Dimensions...>::f_matrix;

  template <typename Function_, int... Dimensions>
  struct Vectorizer2_ {
    static inline int
    f(afloat* const __restrict__ f1, const afloat* const __restrict__ f2, int p1, int p2) {
      int idx1 = getIndex(p1);
      int idx2 = getIndex(p2);
      if (idx1 < 0 || idx2 < 0) {
        return Function_::f(f1, f2, p1, p2);
      }
      return (*f_matrix[idx1][idx2])(f1, f2);
    }

    inline int
    operator()(afloat* const __restrict__ f1, const afloat* const __restrict__ f2, int p1, int p2) {
      return f(f1, f2, p1, p2);
    }

    Vectorizer2_(const char* name_) {
      DEBUG(vectorizer_debug) << "Vectorize2_ " << name_ << std::endl;
      memset(f_matrix, 0, sizeof(f_matrix));
      instantiate<'c', Dimensions...>();
    }

  protected:
    static constexpr int N = sizeof...(Dimensions);
    using FunctionMatrix   = Float2Fn[N][N];
    static FunctionMatrix f_matrix;

    template <int p1, int p2>
    static int _vfn(afloat* const __restrict__ f1, const afloat* const __restrict__ f2) {
      return Function_::f(f1, f2, p1, p2);
    }

    template <int p1, char c, int p2, int... rest>
    void instantiate_p2() {
      int idx1 = getIndex(p1);
      int idx2 = getIndex(p2);
      if (idx1 < 0 || idx2 < 0) {
        return;
      }

      f_matrix[idx1][idx2] = this->_vfn<p1, p2>;
      DEBUG(vectorizer_debug) << "<" << p1 << "," << p2 << "> (" << idx1 << "," << idx2 << ") -> "
                              << (void*) f_matrix[idx1][idx2] << std::endl;

      instantiate_p2<p1, c, rest...>();
    }

    template <int p1, char c>
    void instantiate_p2() {
    }

    template <char c, int p1, int... rest>
    void instantiate() {
      instantiate_p2<p1, c, Dimensions...>();
      instantiate<c, rest...>();
    }

    template <char c>
    void instantiate() {
    }
  };

  template <typename Function_, int... Dimensions>
  typename Vectorizer2_<Function_, Dimensions...>::FunctionMatrix
    Vectorizer2_<Function_, Dimensions...>::f_matrix;

  template <typename Function_, int... Dimensions>
  struct Vectorizer3_ {
    static inline int f(afloat* const __restrict__ f1,
                        const afloat* const __restrict__ f2,
                        const afloat* const __restrict__ f3,
                        int p1 = 0,
                        int p2 = 0,
                        int p3 = 0) {
      int idx1 = getIndex(p1);
      int idx2 = getIndex(p2);
      int idx3 = getIndex(p3);

      if (idx1 < 0 || idx2 < 0 || idx3 < 0) {
        return Function_::f(f1, f2, f3, p1, p2, p3);
      }
      return (*f_matrix[idx1][idx2][idx3])(f1, f2, f3);
    }

    inline int operator()(afloat* const __restrict__ f1,
                          const afloat* const __restrict__ f2,
                          const afloat* const __restrict__ f3,
                          int p1 = 0,
                          int p2 = 0,
                          int p3 = 0) {
      return f(f1, f2, f3, p1, p2, p3);
    }

    Vectorizer3_(const char* name_) {
      DEBUG(vectorizer_debug) << "Vectorizer3_ " << name_ << std::endl;
      memset(f_matrix, 0, sizeof(f_matrix));
      instantiate<'c', Dimensions...>();
    }

  protected:
    static constexpr int N = sizeof...(Dimensions);
    using FunctionMatrix   = Float3Fn[N][N][N];
    static FunctionMatrix f_matrix;
    // FunctionMatrix f_matrix;

    template <int p1, int p2, int p3>
    static int _vfn(afloat* const __restrict__ f1,
                    const afloat* const __restrict__ f2,
                    const afloat* const __restrict__ f3) {
      return Function_::f(f1, f2, f3, p1, p2, p3);
    }

    template <int p1, int p2, char c, int p3, int... rest>
    void instantiate_p3() {
      int idx1 = getIndex(p1);
      int idx2 = getIndex(p2);
      int idx3 = getIndex(p3);
      if (idx1 < 0 || idx2 < 0 || idx3 < 0) {
        DEBUG(vectorizer_debug) << "Warning, " << p1 << " x " << p2 << " x " << p3
                                << "exceed maximum vectorizable dim";
        return;
      }
      f_matrix[idx1][idx2][idx3] = this->_vfn<p1, p2, p3>;
      DEBUG(vectorizer_debug) << "<" << p1 << "," << p2 << "," << p3 << "> (" << idx1 << "," << idx2
                              << "," << idx3 << ") -> " << (void*) f_matrix[idx1][idx2][idx3]
                              << std::endl;
      instantiate_p3<p1, p2, c, rest...>();
    }

    template <int p1, int p2, char c>
    void instantiate_p3() {
    }

    template <int p1, char c, int p2, int... rest>
    void instantiate_p2() {
      instantiate_p3<p1, p2, c, Dimensions...>();
      instantiate_p2<p1, c, rest...>();
    }

    template <int p1, char c>
    void instantiate_p2() {
    }

    template <char c, int p1, int... rest>
    void instantiate() {
      instantiate_p2<p1, c, Dimensions...>();
      instantiate<c, rest...>();
    }

    template <char c>
    void instantiate() {
    }
  };

  template <typename Function_, int... Dimensions>
  typename Vectorizer3_<Function_, Dimensions...>::FunctionMatrix
    Vectorizer3_<Function_, Dimensions...>::f_matrix;

} // namespace vectorizer
