#ifndef FALCON_INTERCEPT_DEEPSTREAM__FALCON_NV12_COLOR_MATRIX_HPP_
#define FALCON_INTERCEPT_DEEPSTREAM__FALCON_NV12_COLOR_MATRIX_HPP_

#include <cudaEGL.h>

struct FalconColorMatrix
{
  float values[9]{
    1.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 1.0F};
  float contrast_pivot{116.0F};
  float contrast{1.0F};
  float magenta_suppression{0.0F};
  float tone_gamma{1.0F};
  float color_mix{1.0F};
};

bool falcon_initialize_color_cuda();

bool falcon_apply_nv12_color_matrix(
  EGLImageKHR image, const FalconColorMatrix & matrix);

#endif  // FALCON_INTERCEPT_DEEPSTREAM__FALCON_NV12_COLOR_MATRIX_HPP_
