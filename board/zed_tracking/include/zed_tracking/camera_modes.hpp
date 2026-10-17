#pragma once
#include <string_view>

namespace zed_tracking {
// Exact ZED X / X Mini sensor modes. Do not let SDK nearest-mode fallback
// silently turn a requested dataset configuration into another one.
inline bool camera_mode_supported(std::string_view resolution, int fps) {
  const bool mode = resolution == "HD1080" || resolution == "HD1200" || resolution == "SVGA";
  return mode && (fps == 15 || fps == 30 || fps == 60 || (resolution == "SVGA" && fps == 120));
}
}
