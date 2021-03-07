#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef float afloat __attribute__((__aligned__(16)));
static constexpr int max_cols=32;
static constexpr int max_rows=32;
static constexpr int max_dim=max_cols*max_rows;

inline float cMatGetValue(const float* src, int nr, int nc __attribute__((unused)), int r, int c) {
  return src[c * nr + r];
}

inline void cMatSetValue(float* dest, int nr, int nc __attribute__((unused)), int r, int c, float value) {
  dest[c * nr + r] = value;
}

inline void cVecPrint(FILE* f, const float* src, int size) {
  fprintf(f, "VEC size: %d values: [ ", size);
  for (; size > 0; ++src, --size) {
    fprintf(f, "%f ", *src);
  }
  fprintf(f, "]\n");
}

inline void cVecCopy(float* dest, const float* src, int size) {
  for (; size > 0; ++dest, ++src, --size) {
    *dest = *src;
  }
}

inline void cVecSwap(float* src1, float* src2, int size) {
  for (; size > 0; ++src1, ++src2, --size) {
    float x = *src1;
    *src1 = *src2;
    *src2 = x;
  }
}

inline void cVecFill(float* src, int size, const float value) {
  for (; size > 0; ++src, --size) {
    *src = value;
  }
}

inline void cVecFillRandom(float* src, int size) {
  for (; size > 0; ++src, --size) {
    *src = drand48();
  }
}

inline void cVecSum(float* dest, const float* src, int size) {
  for (; size > 0; ++dest, ++src, --size) {
    *dest += *src;
  }
}

inline void cVecMinus(float* dest, const float* src, int size) {
  for (; size > 0; ++dest, ++src, --size) {
    *dest -= *src;
  }
}

inline void cVecSumScaled(float* dest, const float* src, int size, float scale) {
  for (; size > 0; ++dest, ++src, --size) {
    *dest += scale * (*src);
  }
}

inline void cVecScale(float* src, int size, float scale) {
  for (; size > 0; ++src, --size) {
    (*src) *= scale;
  }
}

inline float cVecDot(const float* src1, const float* src2, int size) {
  float aux = 0.;
  for (; size > 0; ++src1, ++src2, --size) {
    aux += (*src1) * (*src2);
  }
  return aux;
}

inline float cVecSquaredNorm(const float* src, int size) {
  float aux = 0.;
  for (; size > 0; ++src, --size) {
    aux += (*src) * (*src);
  }
  return aux;
}

inline int cVecFindAbsMaxPos(const float* v, int size) {
  int pivot = -1;
  float max_v = 0;
  for (int s = 0; s < size; ++s, ++v) {
    float av = fabs(*v);
    if (av > max_v) {
      max_v = av;
      pivot = s;
    }
  }
  return pivot;
}

inline void cMatVecProd(float* const dest, const float* src_mat, const float* src_vec, int nr, int nc) {
  cVecFill(dest, nr, 0.f);
  for (; nc > 0; --nc, src_mat += nr, src_vec++) {
    cVecSumScaled(dest, src_mat, nr, *src_vec);
  }
}

inline void cMatCopySymmetric(float* const dest, const float* src, int n, int l_u) {
  cVecCopy(dest, src, n * n);
  for (int c = 0; c < n; ++c) {
    for (int r = c; r < n; ++r) {
      if (l_u) { // upper to lower
        float v = cMatGetValue(dest, n, n, r, c);
        cMatSetValue(dest, n, n, c, r, v);
      } else { // lower to upper
        float v = cMatGetValue(dest, n, n, c, r);
        cMatSetValue(dest, n, n, r, c, v);
      }
    }
  }
}

inline void cMatSwap(float* src1, float* src2, int nr, int nc) {
  cVecSwap(src1, src2, nr * nc);
}

inline void cMatFill(float* dest, int nr, int nc, float value) {
  cVecFill(dest, nr * nc, value);
}

inline void cMatSwapRows(float* dest, int nr, int nc, int r1, int r2) {
  if (r1 == r2) {
    return;
  }
  float* d1 = dest + r1;
  float* d2 = dest + r2;
  for (; nc > 0; --nc, d1 += nr, d2 += nr) {
    float aux = *d1;
    *d1 = *d2;
    *d2 = aux;
  }
}

inline void cMatSwapCols(float* dest, int nr, int nc, int c1, int c2) {
  if (c1 == c2) {
    return;
  }
  float* d1 = dest + (nr * c1);
  float* d2 = dest + (nr * c2);
  cVecSwap(d1, d2, nr);
}

inline void cMatFillRandom(float* dest, int nr, int nc) {
  cVecFillRandom(dest, nr * nc);
}

inline void cMatSetIdentity(float* dest, int n) {
  memset(dest, 0, sizeof(float) * n * n);
  for (int i = 0; i < n; ++i, dest += n) {
    dest[i] = 1.;
  }
}

inline int cMatTriangularize(float* dest, int nr, int nc) {
  //float* origin=dest;
  int r_idx = 0;
  for (int c_idx = 0; c_idx < nc && r_idx < nr; ++c_idx, dest += nr) {
    // column pointer starting from current row
    float* c_ptr = dest + r_idx;

    //scan for pivot and swap rows
    int pivot_idx = cVecFindAbsMaxPos(c_ptr, nr - r_idx);
    if (pivot_idx < 0) {
      continue;
    }

    // tricky, only copies elements that are after current column
    cMatSwapRows(dest, nr, nc - c_idx, 0, pivot_idx);

    // sum to all rows below the current and eliminate
    float inv_pivot = -1. / (*c_ptr);
    for (int rr_idx = r_idx + 1; rr_idx < nr; ++rr_idx) {
      float* cc_ptr = c_ptr;       // current row
      float* cd_ptr = dest + rr_idx; // eliminted row
      float scale = *cd_ptr * inv_pivot;
      for (int cc_idx = c_idx; cc_idx < nc; ++cc_idx, cc_ptr += nr, cd_ptr += nr) {
        *cd_ptr += (*cc_ptr) * scale;
      }
    }
    ++r_idx;
  }
  return r_idx;
}

inline void cMatMultRow(float* dest, int nr, int nc, int r, float alpha) {
  dest += r;
  for (int i = 0; i < nc; ++i, dest += nr) {
    (*dest) *= alpha;
  }
}

inline void cMatSumScaledRows(float* dest_, int nr, int nc, int r_dest, int r_src, float alpha) {
  float* src = dest_ + r_src;
  float* dest = dest_ + r_dest;
  for (int i = 0; i < nc; ++i, dest += nr, src += nr) {
    (*dest) += (*src) * alpha;
  }
}

inline int cMatRank(const float* src, int nr, int nc) {
  //float buffer[nr * nc];
  float buffer[max_rows * max_cols];
  memcpy(buffer, src, nr * nc * sizeof(float));
  return cMatTriangularize(buffer, nr, nc);
}

// Product A * B
inline int cMatMatProd(afloat* dest,
                              const afloat* src1, int nr1, int nc1,
                              const afloat* src2, int nr2, int nc2) {
  if (nc1 != nr2) {
    return -1;
  }
  for (int c = 0; c < nc2; ++c, src2 += nr2, dest += nr1) {
    cMatVecProd(dest, src1, src2, nr1, nc1);
  }
  return 0;
}

// Product A^T * B
static inline int cMatTransposeMatProduct(afloat* dest,
                             const afloat* src1, int nr1, int nc1,
                             const afloat* src2, int nr2, int nc2){

  if(nr1 != nr2){
    return -1;
  }

  float* d = dest;
  for(int c=0; c<nc1; ++c, ++d){
    const float* col1 = src1 + c*nr1;
    float* row = d;
    for(int cc=0;cc<nc2;++cc, row += nc1){
      const float* col2 = src2 + cc*nc2;
      *row = cVecDot(col1, col2, nr1);
    }
  }
  return 0;
}

inline int cMatTranspose(afloat* const __restrict__ dest_,
                    const afloat* __restrict__ src_,
                    const int nr, const int nc) {
  float* dest=dest_;
  const float* src=src_;
  // std::cerr << "transpose: " << nr << " " << nc << std::endl;
  // std::cerr << "dest: " << dest << " " << src << std::endl;
  for (int c = 0; c < nc; ++c) {
    float* d = dest + c;
    for (int r = 0; r < nr; ++r, ++src, d += nc) {
      *d = *src;
    }
  }
      
  //std::cerr <<  std::endl;
  return 0;
}

inline int cMatSolveTriangular(afloat* const x,
                                      const afloat* a,
                                      const int lower_0_upper_1,
                                      const int n) {
  if (lower_0_upper_1) {
    for (int c = n - 1; c >= 0; --c) {
      const float* a_col = a + c * n;
      // std::cerr << " a col " << a_col << std::endl;
      if (a_col[c] == 0 ) {
        return -1;
      }
      // x[c] /= a_col[c];
      const float coeff = x[c]/a_col[c];
      for (int r = 0; r < c; ++r) {
        x[r] -= a_col[r] * coeff ;
      }
      x[c] = coeff;
    }
  } else {
    for (int c = 0; c < n; ++c) {
      const float* a_col = a + c * n;
      if (a_col[c] == 0) {
        return -1;
      }
      x[c] /= a_col[c];
      for (int r = c + 1; r < n; ++r) {
        x[r] -= x[c] * a_col[r];
      }
    }
  }
  return 0;
}

inline int cMatInvert(afloat* const __restrict__ dest,
                      const afloat* const __restrict__ src_,
                      const int n) {
  cMatSetIdentity(dest, n);
  //float src[n * n];
  float src[max_rows * max_cols];
  memcpy(src, src_, n * n * sizeof(float));

  for (int c_idx = 0; c_idx < n; ++c_idx) {
    const float* s_ptr = src + c_idx * n;
    //const float* d_ptr=dest+c_idx*n;

    // column starting at element c,c
    const float* c_ptr = s_ptr + c_idx;
    int pivot_idx = cVecFindAbsMaxPos(c_ptr, n - c_idx);
    if (pivot_idx < 0) { // not invertible
      return -1;
    }

    const float pivot = c_ptr[pivot_idx];
    const float inv_pivot = 1. / pivot;

    // swap rows in dest and src
    cMatSwapRows(src, n, n, c_idx, pivot_idx + c_idx);
    cMatSwapRows(dest, n, n, c_idx, pivot_idx + c_idx);

    //multiply the row by the inverse pivot, to get a 1
    cMatMultRow(src, n, n, c_idx, inv_pivot);
    cMatMultRow(dest, n, n, c_idx, inv_pivot);

    for (int r_idx = 0; r_idx < n; ++r_idx) {
      if (r_idx == c_idx) {
        continue;
      }
      float s_val = s_ptr[r_idx];
      cMatSumScaledRows(src, n, n, r_idx, c_idx, -s_val);
      cMatSumScaledRows(dest, n, n, r_idx, c_idx, -s_val);
    }
  }
  return 0;
}


inline int cMatCopy(afloat* const __restrict__ dest,
                           const afloat* __restrict__ src,
                           const int nr, const int nc) {
  memcpy(dest, src, nr * nc * sizeof(float));
  return 0;
}

inline void cMatPrint(FILE* f, const float* src, int nr, int nc) {
  fprintf(f, "MAT r: %d , c: %d, values:\n[", nr, nc);
  for (int r = 0; r < nr; ++r) {
    const float* s = src + r;
    fprintf(f, "  ");
    for (int c = 0; c < nc; ++c, s += nr) {
      fprintf(f, " %f", *s);
    }
    fprintf(f, "\n");
  }
  fprintf(f, "]\n");
}


inline int cMatTransposeInPlace(afloat* const __restrict__ src, const int n) {
  // loop over columns
  for (int c = 0; c < n; ++c) {
    for (int r = c + 1; r < n; ++r) {
      float ap = src[n * c + r];
      src[n * c + r] = src[n * r + c];
      src[n * r + c] = ap;
    }
  }
  return 0;
}

inline int cMatLLt(afloat* const __restrict__ src, const int n) {
  // loop over columns
  for (int c = 0; c < n; ++c) {
    // col array
    float* c_ptr = src + c * n;
    // diagonal element
    float x = c_ptr[c];  // A_cc 
    for (int k = 0; k < c; ++k) {
      x -= c_ptr[k] * c_ptr[k];
    }  // L_ck L_ck

    if (x < 0) {
      return -1;
    }
    x = sqrt(x);
    c_ptr[c] = x;
    float r = 1.0 / x;
    // off diagonal
    for (int cc = c + 1; cc < n; ++cc) {
      float* cc_ptr = src + cc * n;  // other column pointer (col r)
      x = cc_ptr[c];   // A_rc
      cc_ptr[c] = 0;
      for (int k = 0; k < c; ++k) {
        x -= cc_ptr[k] * c_ptr[k];  // L_rk L_rc
      }
      cc_ptr[c] = x * r;  // L_rc = x / L_cc 
    }
    // clear lower part after using it
    for (int r = c + 1; r < n; ++r) {
      c_ptr[r] = 0;
    }
  }
  return 0;
}

inline int cMatScaleInPlace(afloat* const __restrict__ dest,
                            const afloat* const __restrict__ a,
                            const int n, const int dest_c) {
  for (int c = 0; c < dest_c; ++c) {
    float* const src_col = dest + c * n;
    //float dest_col[n];
    float dest_col[max_rows];
    
    for (int i = 0; i < n; ++i) {
      dest_col[i] = 0.;
    }
    
    const float* a_ptr = a;
    for (int cc = 0; cc < n; ++cc) {
      const float scale = src_col[cc];
      for (int rr = 0; rr < n; ++rr, ++a_ptr) {
        dest_col[rr] += (*a_ptr) * scale;
      }
    }
    
    for (int i = 0; i < n; ++i) {
      src_col[i] = dest_col[i];
        }
  }
  return 0;
}

//dest = -src1^t * src2
inline int cMatProdATransposeBNegate(float* const __restrict__ dest,
                                     const float* const __restrict__ src1,
                                     const float* const __restrict__ src2,
                                     const int cols1, const int cols2, const int rows) {
  const int dest_r = cols1;
  const int dest_c = cols2;
  float* d = dest;
  for (int c = 0; c < dest_c; ++c) {
    const float* col2 = src2 + c * rows;
    for (int r = 0; r < dest_r; ++r, ++d) {
      const float* col1 = src1 + r * rows;
      for (int k = 0; k < rows; ++k) {
        *d -= col1[k] * col2[k];
      }
    }
  }
  return 0;
}

//dest = -src1 * src2
inline int cMatProdABNegate(float* const __restrict__ dest,
                            const float* const __restrict__ src1,
                            const float* const __restrict__ src2,
                            const int rows1, const int cols1, const int cols2) {
  int dim = rows1 * cols2;
  //float buffer[dim];
  float buffer[max_dim];
  float* d = dest;
  cMatMatProd(buffer, src1, rows1, cols1, src2, cols1, cols2);
  cVecMinus(d, buffer, dim);
  return 0;
}


inline int cMatVecProdSub(afloat* const __restrict__ dest,
                          const float* __restrict__ src_mat,
                          const float* __restrict__ src_vec,
                          int nr, int nc, int unused = 0) {
  assert(unused == 0);
  for (int c = 0; c < nc; ++c, src_mat += nr, src_vec++) {
    cVecSumScaled(dest, src_mat, nr, -(*src_vec));
  }
      return 0;
}

inline int cMatMatLeftProdInPlace(afloat* const __restrict__ self,
                                  const float* __restrict__ other,
                                  const int nr, const int nc) {
  float* dest = self;
  int dim = nr * nc;
  //float buffer[dim];
  float buffer[max_dim];
  cVecCopy(buffer, self, dim); 
  return cMatMatProd(dest,other,nr,nr,buffer,nr,nc);
}

int cMatMatRightProdInPlace(afloat* const __restrict__ self,
                            const float* __restrict__ other,
                            const int nr, const int nc) {
  float* dest = self;
  int dim = nr * nc;
  //float buffer[dim];
  float buffer[max_dim];
  cVecCopy(buffer, self, dim); 
  return cMatMatProd(dest,buffer,nr,nr,other,nr,nc);
}
