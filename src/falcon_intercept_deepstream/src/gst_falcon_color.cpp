#include <algorithm>
#include <cstdint>

#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>

#include "nvbufsurface.h"

#include "falcon_intercept_deepstream/falcon_nv12_color_matrix.hpp"

#ifndef PACKAGE
#define PACKAGE "falcon_intercept_deepstream"
#endif

namespace {

constexpr const char * kDefaultMatrix = "0.92,0,0,0,2.08,0,0,0,0.91";
constexpr gdouble kDefaultContrastPivot = 116.0;
constexpr gdouble kDefaultContrast = 1.10;
constexpr gdouble kDefaultMagentaSuppression = 1.0;
constexpr gdouble kDefaultToneGamma = 0.55;
constexpr gdouble kDefaultColorMix = 0.85;

enum PropertyId
{
  kPropertyNone = 0,
  kPropertyEnabled,
  kPropertyMatrix,
  kPropertyContrastPivot,
  kPropertyContrast,
  kPropertyMagentaSuppression,
  kPropertyToneGamma,
  kPropertyColorMix,
  kPropertyLabel,
};

struct GstFalconColor
{
  GstBaseTransform parent;
  gboolean enabled;
  gchar * matrix_text;
  gchar * label;
  FalconColorMatrix matrix;
  std::uint64_t applied_frames;
  std::uint64_t errors;
};

struct GstFalconColorClass
{
  GstBaseTransformClass parent_class;
};

bool parse_matrix(const gchar * text, FalconColorMatrix & output)
{
  if (text == nullptr) {
    return false;
  }

  gchar ** tokens = g_strsplit(text, ",", -1);
  if (tokens == nullptr) {
    return false;
  }

  // A matrix update must not reset the independently configured contrast and
  // blend properties.
  FalconColorMatrix parsed = output;
  std::size_t count = 0U;
  bool valid = true;
  for (; tokens[count] != nullptr; ++count) {
    if (count >= 9U) {
      valid = false;
      break;
    }
    gchar * token = g_strstrip(tokens[count]);
    gchar * end = nullptr;
    const double value = g_ascii_strtod(token, &end);
    if (end == token || *g_strstrip(end) != '\0') {
      valid = false;
      break;
    }
    parsed.values[count] = static_cast<float>(value);
  }

  if (count != 9U || tokens[count] != nullptr) {
    valid = false;
  }
  if (valid) {
    output = parsed;
  }
  g_strfreev(tokens);
  return valid;
}

}  // namespace

G_DEFINE_TYPE(GstFalconColor, gst_falcon_color, GST_TYPE_BASE_TRANSFORM)

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
  "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
  GST_STATIC_CAPS("video/x-raw(memory:NVMM),format=(string)NV12"));

static GstStaticPadTemplate source_template = GST_STATIC_PAD_TEMPLATE(
  "src", GST_PAD_SRC, GST_PAD_ALWAYS,
  GST_STATIC_CAPS("video/x-raw(memory:NVMM),format=(string)NV12"));

static void gst_falcon_color_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * spec)
{
  auto * self = reinterpret_cast<GstFalconColor *>(object);
  switch (property_id) {
    case kPropertyEnabled:
      self->enabled = g_value_get_boolean(value);
      break;
    case kPropertyMatrix: {
      const gchar * matrix_text = g_value_get_string(value);
      FalconColorMatrix matrix{};
      if (!parse_matrix(matrix_text, matrix)) {
        GST_WARNING_OBJECT(self, "matrix must contain exactly 9 comma-separated numbers");
        break;
      }
      self->matrix = matrix;
      g_free(self->matrix_text);
      self->matrix_text = g_strdup(matrix_text);
      break;
    }
    case kPropertyContrastPivot:
      self->matrix.contrast_pivot = static_cast<float>(g_value_get_double(value));
      break;
    case kPropertyContrast:
      self->matrix.contrast = static_cast<float>(g_value_get_double(value));
      break;
    case kPropertyMagentaSuppression:
      self->matrix.magenta_suppression = static_cast<float>(g_value_get_double(value));
      break;
    case kPropertyToneGamma:
      self->matrix.tone_gamma = static_cast<float>(g_value_get_double(value));
      break;
    case kPropertyColorMix:
      self->matrix.color_mix = static_cast<float>(g_value_get_double(value));
      break;
    case kPropertyLabel:
      g_free(self->label);
      self->label = g_value_dup_string(value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, spec);
      break;
  }
}

static void gst_falcon_color_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * spec)
{
  auto * self = reinterpret_cast<GstFalconColor *>(object);
  switch (property_id) {
    case kPropertyEnabled:
      g_value_set_boolean(value, self->enabled);
      break;
    case kPropertyMatrix:
      g_value_set_string(value, self->matrix_text);
      break;
    case kPropertyContrastPivot:
      g_value_set_double(value, self->matrix.contrast_pivot);
      break;
    case kPropertyContrast:
      g_value_set_double(value, self->matrix.contrast);
      break;
    case kPropertyMagentaSuppression:
      g_value_set_double(value, self->matrix.magenta_suppression);
      break;
    case kPropertyToneGamma:
      g_value_set_double(value, self->matrix.tone_gamma);
      break;
    case kPropertyColorMix:
      g_value_set_double(value, self->matrix.color_mix);
      break;
    case kPropertyLabel:
      g_value_set_string(value, self->label);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, spec);
      break;
  }
}

static gboolean gst_falcon_color_start(GstBaseTransform * transform)
{
  auto * self = reinterpret_cast<GstFalconColor *>(transform);
  self->applied_frames = 0U;
  self->errors = 0U;
  if (!self->enabled) {
    return TRUE;
  }
  if (!falcon_initialize_color_cuda()) {
    GST_ERROR_OBJECT(self, "failed to initialize CUDA");
    return FALSE;
  }
  g_print(
    "[falconcolor:%s] enabled=true matrix=%s pivot=%.3f contrast=%.3f "
    "magenta=%.3f gamma=%.3f color-mix=%.3f\n",
    self->label, self->matrix_text, self->matrix.contrast_pivot,
    self->matrix.contrast, self->matrix.magenta_suppression,
    self->matrix.tone_gamma, self->matrix.color_mix);
  return TRUE;
}

static gboolean gst_falcon_color_stop(GstBaseTransform * transform)
{
  auto * self = reinterpret_cast<GstFalconColor *>(transform);
  g_print(
    "[falconcolor:%s] final applied=%lu errors=%lu enabled=%s\n", self->label,
    static_cast<unsigned long>(self->applied_frames),
    static_cast<unsigned long>(self->errors), self->enabled ? "true" : "false");
  return TRUE;
}

static GstFlowReturn gst_falcon_color_transform_ip(
  GstBaseTransform * transform, GstBuffer * buffer)
{
  auto * self = reinterpret_cast<GstFalconColor *>(transform);
  if (!self->enabled) {
    return GST_FLOW_OK;
  }

  GstMapInfo map_info = GST_MAP_INFO_INIT;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_READWRITE)) {
    ++self->errors;
    GST_ERROR_OBJECT(self, "failed to map GstBuffer to NvBufSurface");
    return GST_FLOW_ERROR;
  }

  auto * surface = reinterpret_cast<NvBufSurface *>(map_info.data);
  bool success = surface != nullptr && surface->numFilled > 0U;
  if (!success) {
    GST_ERROR_OBJECT(self, "NvBufSurface is empty");
  }

  const guint surface_count = success ? surface->numFilled : 0U;
  for (guint index = 0U; index < surface_count; ++index) {
    if (NvBufSurfaceMapEglImage(surface, static_cast<int>(index)) != 0) {
      GST_ERROR_OBJECT(self, "NvBufSurfaceMapEglImage failed for surface %u", index);
      success = false;
      break;
    }

    EGLImageKHR image = reinterpret_cast<EGLImageKHR>(
      surface->surfaceList[index].mappedAddr.eglImage);
    const bool applied = image != nullptr &&
      falcon_apply_nv12_color_matrix(image, self->matrix);
    if (!applied) {
      GST_ERROR_OBJECT(self, "CUDA color matrix failed for surface %u", index);
      success = false;
    } else {
      ++self->applied_frames;
    }

    if (NvBufSurfaceUnMapEglImage(surface, static_cast<int>(index)) != 0) {
      GST_ERROR_OBJECT(self, "NvBufSurfaceUnMapEglImage failed for surface %u", index);
      success = false;
    }
    if (!success) {
      break;
    }
  }

  gst_buffer_unmap(buffer, &map_info);
  if (!success) {
    ++self->errors;
    return GST_FLOW_ERROR;
  }

  if (self->applied_frames > 0U && self->applied_frames % 300U == 0U) {
    g_print(
      "[falconcolor:%s] applied=%lu errors=%lu\n", self->label,
      static_cast<unsigned long>(self->applied_frames),
      static_cast<unsigned long>(self->errors));
  }
  return GST_FLOW_OK;
}

static void gst_falcon_color_finalize(GObject * object)
{
  auto * self = reinterpret_cast<GstFalconColor *>(object);
  g_free(self->matrix_text);
  g_free(self->label);
  G_OBJECT_CLASS(gst_falcon_color_parent_class)->finalize(object);
}

static void gst_falcon_color_class_init(GstFalconColorClass * klass)
{
  auto * object_class = G_OBJECT_CLASS(klass);
  auto * element_class = GST_ELEMENT_CLASS(klass);
  auto * transform_class = GST_BASE_TRANSFORM_CLASS(klass);

  object_class->set_property = gst_falcon_color_set_property;
  object_class->get_property = gst_falcon_color_get_property;
  object_class->finalize = gst_falcon_color_finalize;

  g_object_class_install_property(
    object_class, kPropertyEnabled,
    g_param_spec_boolean(
      "enabled", "Enabled", "Apply the configured matrix", TRUE,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyMatrix,
    g_param_spec_string(
      "matrix", "Matrix", "Row-major 3x3 RGB gain matrix", kDefaultMatrix,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyContrastPivot,
    g_param_spec_double(
      "contrast-pivot", "Contrast pivot",
      "RGB pivot used by (value - pivot) * contrast + pivot",
      0.0, 255.0, kDefaultContrastPivot,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyContrast,
    g_param_spec_double(
      "contrast", "Contrast", "Post-gain RGB contrast multiplier",
      0.0, 4.0, kDefaultContrast,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyMagentaSuppression,
    g_param_spec_double(
      "magenta-suppression", "Magenta suppression",
      "Adaptive dark R/B excess correction strength",
      0.0, 2.0, kDefaultMagentaSuppression,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyToneGamma,
    g_param_spec_double(
      "gamma", "Gamma", "Post-contrast tone-curve exponent",
      0.1, 4.0, kDefaultToneGamma,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyColorMix,
    g_param_spec_double(
      "color-mix", "Color mix",
      "Corrected-color fraction; the remainder is grayscale brightness",
      0.0, 1.0, kDefaultColorMix,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
    object_class, kPropertyLabel,
    g_param_spec_string(
      "label", "Label", "Camera label used in statistics", "camera",
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_set_static_metadata(
    element_class, "Falcon NVMM NV12 color matrix", "Filter/Video",
    "Applies shared gain, contrast and grayscale-blend correction to NVMM NV12 frames",
    "Falcon Intercept");
  gst_element_class_add_static_pad_template(element_class, &sink_template);
  gst_element_class_add_static_pad_template(element_class, &source_template);

  transform_class->start = gst_falcon_color_start;
  transform_class->stop = gst_falcon_color_stop;
  transform_class->transform_ip = gst_falcon_color_transform_ip;
}

static void gst_falcon_color_init(GstFalconColor * self)
{
  self->enabled = TRUE;
  self->matrix_text = g_strdup(kDefaultMatrix);
  self->label = g_strdup("camera");
  self->matrix = FalconColorMatrix{};
  parse_matrix(kDefaultMatrix, self->matrix);
  self->matrix.contrast_pivot = static_cast<float>(kDefaultContrastPivot);
  self->matrix.contrast = static_cast<float>(kDefaultContrast);
  self->matrix.magenta_suppression = static_cast<float>(kDefaultMagentaSuppression);
  self->matrix.tone_gamma = static_cast<float>(kDefaultToneGamma);
  self->matrix.color_mix = static_cast<float>(kDefaultColorMix);
  self->applied_frames = 0U;
  self->errors = 0U;
  gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
  gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
  gst_base_transform_set_qos_enabled(GST_BASE_TRANSFORM(self), FALSE);
}

static gboolean plugin_init(GstPlugin * plugin)
{
  return gst_element_register(
    plugin, "falconcolor", GST_RANK_NONE, gst_falcon_color_get_type());
}

GST_PLUGIN_DEFINE(
  GST_VERSION_MAJOR,
  GST_VERSION_MINOR,
  falconcolor,
  "Falcon CUDA NVMM color correction",
  plugin_init,
  "1.0.0",
  "Proprietary",
  "falcon_intercept_deepstream",
  "https://example.invalid/falcon_intercept_deepstream")
