#include <cstdint>
#include <cstdio>

#include <cuda.h>
#include <cuda_runtime.h>

#include "falcon_intercept_deepstream/falcon_nv12_color_matrix.hpp"

namespace {

__device__ __forceinline__ float clamp_channel(float value)
{
  return fminf(255.0F, fmaxf(0.0F, value));
}

__device__ __forceinline__ std::uint8_t clamp_byte(float value)
{
  return static_cast<std::uint8_t>(clamp_channel(value) + 0.5F);
}

__device__ __forceinline__ void nv12_to_rgb(
  std::uint8_t y_value, std::uint8_t u_value, std::uint8_t v_value,
  float & red, float & green, float & blue)
{
  const float y = fmaxf(0.0F, static_cast<float>(y_value) - 16.0F);
  const float u = static_cast<float>(u_value) - 128.0F;
  const float v = static_cast<float>(v_value) - 128.0F;
  red = clamp_channel(1.164383F * y + 1.596027F * v);
  green = clamp_channel(1.164383F * y - 0.391762F * u - 0.812968F * v);
  blue = clamp_channel(1.164383F * y + 2.017232F * u);
}

__device__ __forceinline__ void apply_matrix(
  const FalconColorMatrix & matrix, float red, float green, float blue,
  float & output_red, float & output_green, float & output_blue)
{
  // Apply channel/matrix gains first, then contrast around the configured
  // pivot. Clipping happens before the final gray-brightness blend to match
  // the reference color-correction recipe exactly.
  float corrected_red = clamp_channel(
    (matrix.values[0] * red + matrix.values[1] * green +
    matrix.values[2] * blue - matrix.contrast_pivot) *
    matrix.contrast + matrix.contrast_pivot);
  float corrected_green = clamp_channel(
    (matrix.values[3] * red + matrix.values[4] * green +
    matrix.values[5] * blue - matrix.contrast_pivot) *
    matrix.contrast + matrix.contrast_pivot);
  float corrected_blue = clamp_channel(
    (matrix.values[6] * red + matrix.values[7] * green +
    matrix.values[8] * blue - matrix.contrast_pivot) *
    matrix.contrast + matrix.contrast_pivot);

  // Weak/no IR-cut sensors often render dark navy and black fabrics as
  // magenta. Correct only the R/B excess over green so neutral colors and
  // ordinary luminance detail remain unchanged.
  const float magenta_excess = fmaxf(
    0.0F, fminf(corrected_red, corrected_blue) - corrected_green);
  const float suppressed_magenta = matrix.magenta_suppression * magenta_excess;
  corrected_red = clamp_channel(corrected_red - 0.52F * suppressed_magenta);
  corrected_green = clamp_channel(corrected_green + 0.62F * suppressed_magenta);
  corrected_blue = clamp_channel(corrected_blue - 0.20F * suppressed_magenta);

  if (fabsf(matrix.tone_gamma - 1.0F) > 0.0001F) {
    corrected_red = 255.0F * __powf(corrected_red / 255.0F, matrix.tone_gamma);
    corrected_green = 255.0F * __powf(corrected_green / 255.0F, matrix.tone_gamma);
    corrected_blue = 255.0F * __powf(corrected_blue / 255.0F, matrix.tone_gamma);
  }

  const float gray_brightness =
    0.299F * corrected_red + 0.587F * corrected_green + 0.114F * corrected_blue;
  const float gray_mix = 1.0F - matrix.color_mix;
  output_red = clamp_channel(matrix.color_mix * corrected_red + gray_mix * gray_brightness);
  output_green = clamp_channel(matrix.color_mix * corrected_green + gray_mix * gray_brightness);
  output_blue = clamp_channel(matrix.color_mix * corrected_blue + gray_mix * gray_brightness);
}

__device__ __forceinline__ float rgb_to_y(float red, float green, float blue)
{
  return 16.0F + 0.256788F * red + 0.504129F * green + 0.097906F * blue;
}

__device__ __forceinline__ float rgb_to_u(float red, float green, float blue)
{
  return 128.0F - 0.148223F * red - 0.290993F * green + 0.439216F * blue;
}

__device__ __forceinline__ float rgb_to_v(float red, float green, float blue)
{
  return 128.0F + 0.439216F * red - 0.367788F * green - 0.071427F * blue;
}

__global__ void apply_nv12_matrix(
  std::uint8_t * y_plane, std::uint8_t * uv_plane, int pitch,
  int width, int height, FalconColorMatrix matrix)
{
  const int x = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const int y = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  if (x >= width || y >= height) {
    return;
  }

  const int uv_offset = (y / 2) * pitch + x;
  const std::uint8_t input_u = uv_plane[uv_offset];
  const std::uint8_t input_v = uv_plane[uv_offset + 1];
  float output_u_sum = 0.0F;
  float output_v_sum = 0.0F;
  int sample_count = 0;

#pragma unroll
  for (int row = 0; row < 2; ++row) {
#pragma unroll
    for (int column = 0; column < 2; ++column) {
      const int pixel_x = x + column;
      const int pixel_y = y + row;
      if (pixel_x >= width || pixel_y >= height) {
        continue;
      }

      const int y_offset = pixel_y * pitch + pixel_x;
      float red = 0.0F;
      float green = 0.0F;
      float blue = 0.0F;
      nv12_to_rgb(y_plane[y_offset], input_u, input_v, red, green, blue);

      float output_red = 0.0F;
      float output_green = 0.0F;
      float output_blue = 0.0F;
      apply_matrix(matrix, red, green, blue, output_red, output_green, output_blue);

      y_plane[y_offset] = clamp_byte(rgb_to_y(output_red, output_green, output_blue));
      output_u_sum += rgb_to_u(output_red, output_green, output_blue);
      output_v_sum += rgb_to_v(output_red, output_green, output_blue);
      ++sample_count;
    }
  }

  if (sample_count > 0) {
    uv_plane[uv_offset] = clamp_byte(output_u_sum / static_cast<float>(sample_count));
    uv_plane[uv_offset + 1] = clamp_byte(output_v_sum / static_cast<float>(sample_count));
  }
}

__global__ void apply_nv12_matrix_array(
  cudaSurfaceObject_t y_surface, cudaSurfaceObject_t uv_surface,
  int width, int height, FalconColorMatrix matrix)
{
  const int x = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const int y = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  if (x >= width || y >= height) {
    return;
  }

  const uchar2 input_uv = surf2Dread<uchar2>(
    uv_surface, (x / 2) * static_cast<int>(sizeof(uchar2)), y / 2);
  float output_u_sum = 0.0F;
  float output_v_sum = 0.0F;
  int sample_count = 0;

#pragma unroll
  for (int row = 0; row < 2; ++row) {
#pragma unroll
    for (int column = 0; column < 2; ++column) {
      const int pixel_x = x + column;
      const int pixel_y = y + row;
      if (pixel_x >= width || pixel_y >= height) {
        continue;
      }

      const std::uint8_t input_y = surf2Dread<std::uint8_t>(
        y_surface, pixel_x * static_cast<int>(sizeof(std::uint8_t)), pixel_y);
      float red = 0.0F;
      float green = 0.0F;
      float blue = 0.0F;
      nv12_to_rgb(input_y, input_uv.x, input_uv.y, red, green, blue);

      float output_red = 0.0F;
      float output_green = 0.0F;
      float output_blue = 0.0F;
      apply_matrix(matrix, red, green, blue, output_red, output_green, output_blue);

      const std::uint8_t output_y = clamp_byte(
        rgb_to_y(output_red, output_green, output_blue));
      surf2Dwrite(
        output_y, y_surface,
        pixel_x * static_cast<int>(sizeof(std::uint8_t)), pixel_y);
      output_u_sum += rgb_to_u(output_red, output_green, output_blue);
      output_v_sum += rgb_to_v(output_red, output_green, output_blue);
      ++sample_count;
    }
  }

  if (sample_count > 0) {
    const uchar2 output_uv = make_uchar2(
      clamp_byte(output_u_sum / static_cast<float>(sample_count)),
      clamp_byte(output_v_sum / static_cast<float>(sample_count)));
    surf2Dwrite(
      output_uv, uv_surface,
      (x / 2) * static_cast<int>(sizeof(uchar2)), y / 2);
  }
}

}  // namespace

bool falcon_initialize_color_cuda()
{
  return cudaFree(nullptr) == cudaSuccess;
}

bool falcon_apply_nv12_color_matrix(
  EGLImageKHR image, const FalconColorMatrix & matrix)
{
  const cudaError_t context_error = cudaSetDevice(0);
  if (context_error != cudaSuccess) {
    std::fprintf(
      stderr, "falcon color: cudaSetDevice failed: %s\n",
      cudaGetErrorString(context_error));
    return false;
  }

  CUgraphicsResource resource = nullptr;
  CUeglFrame frame{};
  CUresult status = cuGraphicsEGLRegisterImage(
    &resource, image, CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);
  if (status != CUDA_SUCCESS) {
    std::fprintf(stderr, "falcon color: cuGraphicsEGLRegisterImage failed: %d\n", status);
    return false;
  }

  status = cuGraphicsResourceGetMappedEglFrame(&frame, resource, 0, 0);
  if (status != CUDA_SUCCESS) {
    std::fprintf(stderr, "falcon color: cuGraphicsResourceGetMappedEglFrame failed: %d\n", status);
    cuGraphicsUnregisterResource(resource);
    return false;
  }

  bool success = false;
  if (frame.eglColorFormat != CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR ||
    frame.planeCount < 2)
  {
    std::fprintf(
      stderr, "falcon color: unsupported EGL color=%d planes=%u\n",
      static_cast<int>(frame.eglColorFormat), frame.planeCount);
  } else if (frame.frameType == CU_EGL_FRAME_TYPE_PITCH) {
    const dim3 threads(16, 16);
    const dim3 blocks(
      (frame.width + threads.x * 2 - 1) / (threads.x * 2),
      (frame.height + threads.y * 2 - 1) / (threads.y * 2));
    apply_nv12_matrix<<<blocks, threads>>>(
      reinterpret_cast<std::uint8_t *>(frame.frame.pPitch[0]),
      reinterpret_cast<std::uint8_t *>(frame.frame.pPitch[1]),
      static_cast<int>(frame.pitch), static_cast<int>(frame.width),
      static_cast<int>(frame.height), matrix);

    const cudaError_t launch_error = cudaGetLastError();
    if (launch_error != cudaSuccess) {
      std::fprintf(
        stderr, "falcon color: CUDA launch failed: %s\n",
        cudaGetErrorString(launch_error));
    } else {
      const cudaError_t sync_error = cudaDeviceSynchronize();
      success = sync_error == cudaSuccess;
      if (!success) {
        std::fprintf(
          stderr, "falcon color: CUDA sync failed: %s\n",
          cudaGetErrorString(sync_error));
      }
    }
  } else if (frame.frameType == CU_EGL_FRAME_TYPE_ARRAY) {
    cudaResourceDesc y_description{};
    y_description.resType = cudaResourceTypeArray;
    y_description.res.array.array = reinterpret_cast<cudaArray_t>(frame.frame.pArray[0]);
    cudaResourceDesc uv_description{};
    uv_description.resType = cudaResourceTypeArray;
    uv_description.res.array.array = reinterpret_cast<cudaArray_t>(frame.frame.pArray[1]);

    cudaSurfaceObject_t y_surface = 0U;
    cudaSurfaceObject_t uv_surface = 0U;
    cudaError_t surface_error = cudaCreateSurfaceObject(&y_surface, &y_description);
    if (surface_error == cudaSuccess) {
      surface_error = cudaCreateSurfaceObject(&uv_surface, &uv_description);
    }
    if (surface_error != cudaSuccess) {
      std::fprintf(
        stderr, "falcon color: cudaCreateSurfaceObject failed: %s\n",
        cudaGetErrorString(surface_error));
    } else {
      const dim3 threads(16, 16);
      const dim3 blocks(
        (frame.width + threads.x * 2 - 1) / (threads.x * 2),
        (frame.height + threads.y * 2 - 1) / (threads.y * 2));
      apply_nv12_matrix_array<<<blocks, threads>>>(
        y_surface, uv_surface, static_cast<int>(frame.width),
        static_cast<int>(frame.height), matrix);

      const cudaError_t launch_error = cudaGetLastError();
      if (launch_error != cudaSuccess) {
        std::fprintf(
          stderr, "falcon color: CUDA array launch failed: %s\n",
          cudaGetErrorString(launch_error));
      } else {
        const cudaError_t sync_error = cudaDeviceSynchronize();
        success = sync_error == cudaSuccess;
        if (!success) {
          std::fprintf(
            stderr, "falcon color: CUDA array sync failed: %s\n",
            cudaGetErrorString(sync_error));
        }
      }
    }

    if (uv_surface != 0U) {
      cudaDestroySurfaceObject(uv_surface);
    }
    if (y_surface != 0U) {
      cudaDestroySurfaceObject(y_surface);
    }
  } else {
    std::fprintf(
      stderr, "falcon color: unsupported EGL frame type=%d\n",
      static_cast<int>(frame.frameType));
  }

  status = cuGraphicsUnregisterResource(resource);
  if (status != CUDA_SUCCESS) {
    std::fprintf(stderr, "falcon color: cuGraphicsUnregisterResource failed: %d\n", status);
    return false;
  }
  return success;
}
