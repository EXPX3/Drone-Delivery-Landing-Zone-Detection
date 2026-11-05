#pragma once

#include "ddlzd_core/types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ddlzd_ros
{

enum class SelectionState : std::uint8_t
{
  kNone = 0,
  kLocked = 1,
  kCoasting = 2,
};

struct StickySelectorConfig
{
  double dropout_timeout_sec{1.5};
  std::uint32_t max_missed_detections{3U};
  std::uint32_t soft_release_observations{2U};
};

struct SelectionResult
{
  SelectionState state{SelectionState::kNone};
  std::uint64_t generation{0U};
  std::optional<ddlzd::Candidate> candidate;
  std::string reason{"no_confirmed_candidate"};

  [[nodiscard]] bool valid() const noexcept
  {
    return state == SelectionState::kLocked && candidate.has_value();
  }
};

class StickySelector
{
public:
  explicit StickySelector(StickySelectorConfig config);

  [[nodiscard]] SelectionResult update(
    const std::vector<ddlzd::Candidate> & candidates, double stamp_sec);
  void reset();

private:
  [[nodiscard]] static bool isAcquirable(const ddlzd::Candidate & candidate);
  [[nodiscard]] const ddlzd::Candidate * bestAcquirable(
    const std::vector<ddlzd::Candidate> & candidates,
    std::optional<std::uint64_t> excluded_id = std::nullopt) const;
  void acquire(const ddlzd::Candidate & candidate, double stamp_sec);
  void release(const std::string & reason);
  [[nodiscard]] SelectionResult result() const;

  StickySelectorConfig config_;
  SelectionState state_{SelectionState::kNone};
  std::optional<ddlzd::Candidate> selected_;
  std::uint64_t generation_{0U};
  std::uint32_t missed_detections_{0U};
  std::uint32_t soft_failures_{0U};
  double last_seen_sec_{0.0};
  double last_update_sec_{0.0};
  std::string reason_{"no_confirmed_candidate"};
};

}  // namespace ddlzd_ros
