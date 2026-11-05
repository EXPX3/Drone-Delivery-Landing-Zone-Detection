#include "ddlzd_ros/sticky_selector.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace
{

ddlzd::Candidate confirmedCandidate(
  const std::uint64_t id, const double risk,
  const ddlzd::Category category = ddlzd::Category::kSafest)
{
  ddlzd::Candidate candidate;
  candidate.id = id;
  candidate.geometry_valid = true;
  candidate.temporally_stable = true;
  candidate.risk_score = risk;
  candidate.category = category;
  candidate.consecutive_observations = 3U;
  return candidate;
}

}  // namespace

TEST(StickySelector, KeepsCurrentCandidateInsteadOfChasingSmallRiskChanges)
{
  ddlzd_ros::StickySelector selector(ddlzd_ros::StickySelectorConfig{});
  auto result = selector.update({confirmedCandidate(1U, 0.20)}, 1.0);
  ASSERT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 1U);

  result = selector.update(
    {confirmedCandidate(1U, 0.25), confirmedCandidate(2U, 0.10)}, 1.5);
  ASSERT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 1U);
}

TEST(StickySelector, AcquiresBestConfirmedSafeCandidate)
{
  ddlzd_ros::StickySelector selector(ddlzd_ros::StickySelectorConfig{});
  auto result = selector.update(
    {confirmedCandidate(1U, 0.55, ddlzd::Category::kSafe),
      confirmedCandidate(2U, 0.40, ddlzd::Category::kSafe)}, 1.0);

  ASSERT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 2U);
}

TEST(StickySelector, CoastsAcrossShortDropoutAndThenReacquires)
{
  ddlzd_ros::StickySelector selector(ddlzd_ros::StickySelectorConfig{1.5, 3U, 2U});
  auto result = selector.update({confirmedCandidate(4U, 0.10)}, 1.0);
  ASSERT_TRUE(result.valid());

  result = selector.update({}, 1.5);
  EXPECT_EQ(result.state, ddlzd_ros::SelectionState::kCoasting);
  EXPECT_FALSE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 4U);

  result = selector.update({confirmedCandidate(4U, 0.12)}, 2.0);
  EXPECT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 4U);
}

TEST(StickySelector, ReplacesHardRejectedCandidateImmediately)
{
  ddlzd_ros::StickySelector selector(ddlzd_ros::StickySelectorConfig{});
  auto result = selector.update({confirmedCandidate(1U, 0.10)}, 1.0);
  ASSERT_TRUE(result.valid());

  auto rejected = confirmedCandidate(1U, 1.0, ddlzd::Category::kRisky);
  rejected.geometry_valid = false;
  rejected.rejection_reason = "obstacle";
  result = selector.update({rejected, confirmedCandidate(2U, 0.20)}, 1.5);

  ASSERT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 2U);
  EXPECT_EQ(result.generation, 2U);
}

TEST(StickySelector, RequiresSustainedSoftFailureBeforeReplacement)
{
  ddlzd_ros::StickySelector selector(ddlzd_ros::StickySelectorConfig{1.5, 3U, 2U});
  auto result = selector.update({confirmedCandidate(1U, 0.10)}, 1.0);
  ASSERT_TRUE(result.valid());

  result = selector.update(
    {confirmedCandidate(1U, 0.80, ddlzd::Category::kRisky),
      confirmedCandidate(2U, 0.20)}, 1.5);
  EXPECT_EQ(result.state, ddlzd_ros::SelectionState::kCoasting);
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 1U);

  result = selector.update(
    {confirmedCandidate(1U, 0.80, ddlzd::Category::kRisky),
      confirmedCandidate(2U, 0.20)}, 2.0);
  ASSERT_TRUE(result.valid());
  ASSERT_TRUE(result.candidate);
  EXPECT_EQ(result.candidate->id, 2U);
}
