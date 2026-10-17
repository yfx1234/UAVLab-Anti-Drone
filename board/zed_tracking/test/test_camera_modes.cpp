#include "zed_tracking/camera_modes.hpp"
#include <cassert>

int main() {
  using zed_tracking::camera_mode_supported;
  for (auto mode : {"HD1080", "HD1200", "SVGA"}) {
    for (int fps : {15, 30, 60}) assert(camera_mode_supported(mode, fps));
    for (int fps : {0, 1, -1, 29, 100, 121}) assert(!camera_mode_supported(mode, fps));
  }
  assert(camera_mode_supported("SVGA", 120));
  assert(!camera_mode_supported("HD1080", 120));
  assert(!camera_mode_supported("HD1200", 120));
  for (auto mode : {"HD720", "HD2K", "VGA", "AUTO", "HD1080; exit"})
    assert(!camera_mode_supported(mode, 15));
}
