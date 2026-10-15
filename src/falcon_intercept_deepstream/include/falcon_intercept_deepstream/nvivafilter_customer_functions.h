/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 */

#ifndef FALCON_INTERCEPT_DEEPSTREAM__NVIVAFILTER_CUSTOMER_FUNCTIONS_H_
#define FALCON_INTERCEPT_DEEPSTREAM__NVIVAFILTER_CUSTOMER_FUNCTIONS_H_

#include <cudaEGL.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum {
  COLOR_FORMAT_Y8 = 0,
  COLOR_FORMAT_U8_V8,
  COLOR_FORMAT_RGBA,
  COLOR_FORMAT_NONE
} ColorFormat;

typedef struct {
  void (*fGPUProcess)(EGLImageKHR image, void ** user_ptr);
  void (*fPreProcess)(
    void ** base_address, unsigned int * memory_size, unsigned int * width,
    unsigned int * height, unsigned int * pitch, ColorFormat * format,
    unsigned int surface_count, void ** user_ptr);
  void (*fPostProcess)(
    void ** base_address, unsigned int * memory_size, unsigned int * width,
    unsigned int * height, unsigned int * pitch, ColorFormat * format,
    unsigned int surface_count, void ** user_ptr);
} CustomerFunction;

void init(CustomerFunction * functions);
void deinit(void);

#if defined(__cplusplus)
}
#endif

#endif  // FALCON_INTERCEPT_DEEPSTREAM__NVIVAFILTER_CUSTOMER_FUNCTIONS_H_
