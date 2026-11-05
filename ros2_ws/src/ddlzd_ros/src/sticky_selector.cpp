#include "ddlzd_ros/sticky_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ddlzd_ros
{

StickySelector::StickySelector(StickySelectorConfig config)
: config_(config)
{
  if (config_.dropout_timeout_sec <= 0.0 || config_.max_missed_detections == 0U ||
    config_.soft_release_observations == 0U)
  {
    throw std::invalid_argument("sticky selector parameters must be positive");
  }
}

bool StickySelector::isAcquirable(const ddlzd::Candidate & candidate)
{
  const bool acceptable_category = candidate.category == ddlzd::Category::kSafest ||
    candidate.category == ddlzd::Category::kSafe;
  return acceptable_category && candidate.geometry_valid &&
         candidate.temporally_stable && candidate.rejection_reason.empty() &&
         std::isfinite(candidate.risk_score);
}

const ddlzd::Candidate * StickySelector::bestAcquirable(
  const std::vector<ddlzd::Candidate> & candidates,
  const std::optional<std::uint64_t> excluded_id) const
{
  const ddlzd::Candidate * best = nullptr;
  for (const auto & candidate : candidates) {
    if ((excluded_id && candidate.id == *excluded_id) || !isAcquirable(candidate)) {
      continue;
    }
    if (best == nullptr || candidate.risk_score < best->risk_score ||
      (candidate.risk_score == best->risk_score && candidate.id < best->id))
    {
      best = &candidate;
    }
  }
  return best;
}

void StickySelector::acquire(const ddlzd::Candidate & candidate, const double stamp_sec)
{
  selected_ = candidate;
  state_ = SelectionState::kLocked;
  ++generation_;
  missed_detections_ = 0U;
  soft_failures_ = 0U;
  last_seen_sec_ = stamp_sec;
  reason_ = "confirmed";
}

void StickySelector::release(const std::string & reason)
{
  selected_.reset();
  state_ = SelectionState::kNone;
  missed_detections_ = 0U;
  soft_failures_ = 0U;
  reason_ = reason;
}

SelectionResult StickySelector::result() const
{
  return SelectionResult{state_, generation_, selected_, reason_};
}

SelectionResult StickySelector::update(
  const std::vector<ddlzd::Candidate> & candidates, const double stamp_sec)
{
  if (!std::isfinite(stamp_sec)) {
    throw std::invalid_argument("selection timestamp must be finite");
  }
  if (last_update_sec_ != 0.0 && stamp_sec < last_update_sec_) {
    reset();
  }
  last_update_sec_ = stamp_sec;

  if (!selected_) {
    if (const auto * candidate = bestAcquirable(candidates)) {
      acquire(*candidate, stamp_sec);
    }
    return result();
  }

  const std::uint64_t selected_id = selected_->id;
  const auto current = std::find_if(
    candidates.begin(), candidates.end(), [selected_id](const ddlzd::Candidate & candidate) {
      return candidate.id == selected_id;
    });

  if (current == candidates.end()) {
    ++missed_detections_;
    const bool timed_out = stamp_sec - last_seen_sec_ > config_.dropout_timeout_sec;
    if (!timed_out && missed_detections_ <= config_.max_missed_detections) {
      state_ = SelectionState::kCoasting;
      reason_ = "temporarily_not_observed";
      return result();
    }
    release("track_lost");
    if (const auto * replacement = bestAcquirable(candidates, selected_id)) {
      acquire(*replacement, stamp_sec);
    }
    return result();
  }

  if (!current->geometry_valid || !current->rejection_reason.empty()) {
    const std::string reason = current->rejection_reason.empty() ?
      "hard_geometry_failure" : current->rejection_reason;
    release(reason);
    if (const auto * replacement = bestAcquirable(candidates, selected_id)) {
      acquire(*replacement, stamp_sec);
    }
    return result();
  }

  last_seen_sec_ = stamp_sec;
  missed_detections_ = 0U;
  selected_ = *current;
  if (current->temporally_stable &&
    (current->category == ddlzd::Category::kSafest ||
    current->category == ddlzd::Category::kSafe))
  {
    state_ = SelectionState::kLocked;
    soft_failures_ = 0U;
    reason_ = "confirmed";
    return result();
  }

  ++soft_failures_;
  if (soft_failures_ < config_.soft_release_observations) {
    state_ = SelectionState::kCoasting;
    reason_ = "risk_or_evidence_degraded";
    return result();
  }

  release("sustained_risk_or_evidence_degradation");
  if (const auto * replacement = bestAcquirable(candidates, selected_id)) {
    acquire(*replacement, stamp_sec);
  }
  return result();
}

void StickySelector::reset()
{
  state_ = SelectionState::kNone;
  selected_.reset();
  generation_ = 0U;
  missed_detections_ = 0U;
  soft_failures_ = 0U;
  last_seen_sec_ = 0.0;
  last_update_sec_ = 0.0;
  reason_ = "no_confirmed_candidate";
}

}  // namespace ddlzd_ros
