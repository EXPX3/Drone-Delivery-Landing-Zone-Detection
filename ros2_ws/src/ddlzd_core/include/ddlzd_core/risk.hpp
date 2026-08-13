#pragma once

#include "ddlzd_core/types.hpp"

namespace ddlzd
{

struct RiskConfig
{
  double safest_threshold{0.33};
  double safe_threshold{0.62};
  double minimum_clearance_coverage{0.55};

  double slope_good_deg{0.0};
  double slope_bad_deg{8.0};
  double relief_good_m{0.10};
  double relief_bad_m{4.50};
  double roughness_good_m{0.03};
  double roughness_bad_m{0.55};
  double clearance_bad_m{2.0};
  double clearance_good_m{12.0};
  double obstacle_bad_count{80.0};
  double observed_bad_fraction{0.70};
  double observed_good_fraction{0.95};
};

class RiskClassifier
{
public:
  explicit RiskClassifier(RiskConfig config);

  void classify(Candidate & candidate) const;

private:
  RiskConfig config_;
};

}  // namespace ddlzd
