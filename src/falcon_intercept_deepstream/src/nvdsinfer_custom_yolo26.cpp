#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include "nvdsinfer_custom_impl.h"

namespace {

const NvDsInferLayerInfo * find_output_layer(
  const std::vector<NvDsInferLayerInfo> & layers)
{
  for (const auto & layer : layers) {
    if (layer.layerName != nullptr && std::strcmp(layer.layerName, "output0") == 0) {
      return &layer;
    }
  }
  return layers.empty() ? nullptr : &layers.front();
}

float clip(float value, float lower, float upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace

extern "C" bool NvDsInferParseYolo26EndToEnd(
  const std::vector<NvDsInferLayerInfo> & output_layers,
  const NvDsInferNetworkInfo & network_info,
  const NvDsInferParseDetectionParams & detection_params,
  std::vector<NvDsInferObjectDetectionInfo> & object_list)
{
  const NvDsInferLayerInfo * layer = find_output_layer(output_layers);
  if (layer == nullptr || layer->buffer == nullptr) {
    std::cerr << "YOLO26 parser: output0 is missing" << std::endl;
    return false;
  }
  if (layer->dataType != FLOAT) {
    std::cerr << "YOLO26 parser: output0 must be FP32, got dataType="
              << static_cast<int>(layer->dataType) << std::endl;
    return false;
  }
  if (layer->inferDims.numElements < 6 || layer->inferDims.numElements % 6 != 0) {
    std::cerr << "YOLO26 parser: unexpected output0 element count="
              << layer->inferDims.numElements << std::endl;
    return false;
  }

  const auto * values = static_cast<const float *>(layer->buffer);
  const std::size_t row_count = layer->inferDims.numElements / 6;
  const float max_x = static_cast<float>(network_info.width - 1U);
  const float max_y = static_cast<float>(network_info.height - 1U);

  for (std::size_t row_index = 0; row_index < row_count; ++row_index) {
    const float * row = values + row_index * 6;
    const float confidence = row[4];
    const int class_id = static_cast<int>(std::lround(row[5]));
    if (class_id < 0 ||
      static_cast<unsigned int>(class_id) >= detection_params.numClassesConfigured)
    {
      continue;
    }

    const auto class_index = static_cast<std::size_t>(class_id);
    const float threshold = class_index < detection_params.perClassPreclusterThreshold.size() ?
      detection_params.perClassPreclusterThreshold[class_index] : 0.0F;
    if (!std::isfinite(confidence) || confidence < threshold) {
      continue;
    }

    const float x1 = clip(row[0], 0.0F, max_x);
    const float y1 = clip(row[1], 0.0F, max_y);
    const float x2 = clip(row[2], 0.0F, max_x);
    const float y2 = clip(row[3], 0.0F, max_y);
    if (!(x2 > x1 && y2 > y1)) {
      continue;
    }

    NvDsInferObjectDetectionInfo object{};
    object.classId = class_id;
    object.left = x1;
    object.top = y1;
    object.width = x2 - x1;
    object.height = y2 - y1;
    object.detectionConfidence = confidence;
    object_list.push_back(object);
  }
  return true;
}

CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInferParseYolo26EndToEnd);

extern "C" bool NvDsInferParseYolo26Raw(
  const std::vector<NvDsInferLayerInfo> & output_layers,
  const NvDsInferNetworkInfo & network_info,
  const NvDsInferParseDetectionParams & detection_params,
  std::vector<NvDsInferObjectDetectionInfo> & object_list)
{
  const NvDsInferLayerInfo * layer = find_output_layer(output_layers);
  if (layer == nullptr || layer->buffer == nullptr) {
    std::cerr << "YOLO26 raw parser: output0 is missing" << std::endl;
    return false;
  }
  if (layer->dataType != FLOAT) {
    std::cerr << "YOLO26 raw parser: output0 must be FP32, got dataType="
              << static_cast<int>(layer->dataType) << std::endl;
    return false;
  }
  const std::size_t class_count = detection_params.numClassesConfigured;
  const std::size_t channel_count = 4U + class_count;
  if (class_count == 0U || layer->inferDims.numElements < channel_count ||
    layer->inferDims.numElements % channel_count != 0U)
  {
    std::cerr << "YOLO26 raw parser: unexpected output0 element count="
              << layer->inferDims.numElements << " classes=" << class_count << std::endl;
    return false;
  }

  const auto * values = static_cast<const float *>(layer->buffer);
  const std::size_t candidate_count = layer->inferDims.numElements / channel_count;
  const float max_x = static_cast<float>(network_info.width - 1U);
  const float max_y = static_cast<float>(network_info.height - 1U);
  for (std::size_t candidate = 0; candidate < candidate_count; ++candidate) {
    int best_class = -1;
    float best_confidence = -1.0F;
    for (std::size_t class_index = 0; class_index < class_count; ++class_index) {
      const float confidence = values[(4U + class_index) * candidate_count + candidate];
      if (std::isfinite(confidence) && confidence > best_confidence) {
        best_confidence = confidence;
        best_class = static_cast<int>(class_index);
      }
    }
    if (best_class < 0) {
      continue;
    }
    const auto class_index = static_cast<std::size_t>(best_class);
    const float threshold = class_index < detection_params.perClassPreclusterThreshold.size() ?
      detection_params.perClassPreclusterThreshold[class_index] : 0.0F;
    if (best_confidence < threshold) {
      continue;
    }

    const float center_x = values[candidate];
    const float center_y = values[candidate_count + candidate];
    const float width = values[2U * candidate_count + candidate];
    const float height = values[3U * candidate_count + candidate];
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
      !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F || height <= 0.0F)
    {
      continue;
    }
    const float x1 = clip(center_x - width * 0.5F, 0.0F, max_x);
    const float y1 = clip(center_y - height * 0.5F, 0.0F, max_y);
    const float x2 = clip(center_x + width * 0.5F, 0.0F, max_x);
    const float y2 = clip(center_y + height * 0.5F, 0.0F, max_y);
    if (!(x2 > x1 && y2 > y1)) {
      continue;
    }

    NvDsInferObjectDetectionInfo object{};
    object.classId = best_class;
    object.left = x1;
    object.top = y1;
    object.width = x2 - x1;
    object.height = y2 - y1;
    object.detectionConfidence = best_confidence;
    object_list.push_back(object);
  }
  return true;
}

CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInferParseYolo26Raw);
