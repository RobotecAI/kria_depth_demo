/*
 * Copyright 2019 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _XF_STEREOBM_CONFIG_H_
#define _XF_STEREOBM_CONFIG_H_

#include "hls_stream.h"
#include "vitis_common/common/xf_common.hpp"
#include "vitis_common/common/xf_utility.hpp"
#include "vitis_common/imgproc/xf_stereolbm.hpp"
#include "vitis_common/imgproc/xf_median_blur.hpp"
#include "xf_config_params.h"

// Set the input and output pixel depth:
#define IN_TYPE XF_8UC1
#define PTR_IN_WIDTH 8
#define OUT_TYPE XF_8UC1
#define PTR_OUT_WIDTH 8


// Set the optimization type:
#define NPC XF_NPPC1

// Set the optimization type:
#if NO == 1
#define NPC1 XF_NPPC1
#define PTR_WIDTH 32
#else

#if GRAY
#define NPC1 XF_NPPC8
#else
#define NPC1 XF_NPPC4
#endif

#define PTR_WIDTH 128
#endif

// Set the pixel depth:
#if GRAY
#define TYPE XF_8UC1
#else
#define TYPE XF_8UC3
#endif


/* config width and height */
#define WIDTH 1920
#define HEIGHT 1080
#endif // _XF_STEREOBM_CONFIG_H_
