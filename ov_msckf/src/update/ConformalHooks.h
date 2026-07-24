/* Optional conformal experiment hooks. Stock runs install neither hook. */
#ifndef OV_MSCKF_CONFORMAL_HOOKS_H
#define OV_MSCKF_CONFORMAL_HOOKS_H

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace ov_core {
class Feature;
}

namespace ov_msckf {

struct MsckfFeatureDiagnostic {
  double timestamp = 0.0;
  size_t feature_id = 0;
  size_t track_measurements = 0;
  double filter_residual_norm = 0.0;
  double chi2 = 0.0;
  double chi2_threshold = 0.0;
  double sigma_pix = 0.0;
  bool passed_chi2_gate = false;
  std::shared_ptr<const ov_core::Feature> feature;
};

using MsckfDiagnosticCallback = std::function<void(const MsckfFeatureDiagnostic &)>;
using MsckfVisualInput = std::array<double, 8>;
using MsckfFrameContext = std::array<double, 6>;
using MsckfSigmaProvider =
    std::function<std::vector<double>(const std::vector<MsckfVisualInput> &features,
                                      const MsckfFrameContext &frame_context)>;

} // namespace ov_msckf

#endif // OV_MSCKF_CONFORMAL_HOOKS_H
