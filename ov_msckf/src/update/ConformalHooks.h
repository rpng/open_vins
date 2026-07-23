/* Optional conformal experiment hooks. Stock runs install neither hook. */
#ifndef OV_MSCKF_CONFORMAL_HOOKS_H
#define OV_MSCKF_CONFORMAL_HOOKS_H

#include <cstddef>
#include <functional>
#include <memory>

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
using MsckfSigmaProvider = std::function<double(size_t feature_id, double timestamp)>;

} // namespace ov_msckf

#endif // OV_MSCKF_CONFORMAL_HOOKS_H
