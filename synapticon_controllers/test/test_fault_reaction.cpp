// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for FaultReaction with the built-in QuickStopPolicy and with a fake policy.

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>

#include "synapticon_controllers/fault_reaction.hpp"
#include "synapticon_controllers/quick_stop_policy.hpp"

namespace
{
using synapticon_controllers::ClearResult;
using synapticon_controllers::FaultPolicyI;
using synapticon_controllers::FaultReaction;
using synapticon_controllers::JointCondition;
using synapticon_controllers::PolicyDecision;
using synapticon_controllers::QuickStopPolicy;
using synapticon_controllers::ReactionEvent;
using synapticon_hardware::DriveAction;
using ::testing::ElementsAre;

constexpr std::size_t JOINTS = 3;
const std::vector<JointCondition> ALL_HEALTHY(JOINTS, JointCondition::HEALTHY);

std::vector<JointCondition> with(std::size_t joint, JointCondition condition)
{
  std::vector<JointCondition> conditions = ALL_HEALTHY;
  conditions[joint] = condition;
  return conditions;
}

/// Disables only the joint that entered a condition and refuses a clear while any joint is in a condition.
class FakePolicy : public FaultPolicyI
{
public:
  bool configure(const std::vector<std::string>&, std::string&) override
  {
    return true;
  }
  bool step(const std::vector<JointCondition>& conditions, const std::vector<std::size_t>& entered,
            std::vector<DriveAction>& actions, ReactionEvent& event) override
  {
    if (entered.empty())
    {
      return false;
    }
    (void)conditions;
    event.action = DriveAction::DISABLE;
    for (const std::size_t j : entered)
    {
      actions[j] = event.action;
      event.acted_on.push_back(j);
    }
    return true;
  }
  PolicyDecision on_event(std::string_view, const std::vector<std::size_t>&, const std::vector<JointCondition>&,
                          std::vector<DriveAction>&, ReactionEvent&) override
  {
    return { false, "no" };
  }
  PolicyDecision may_clear(const std::vector<std::size_t>&, const std::vector<JointCondition>& conditions) const override
  {
    for (const JointCondition c : conditions)
    {
      if (c != JointCondition::HEALTHY)
      {
        return { false, "a joint is still in a condition" };
      }
    }
    return { true, "" };
  }
};

class FaultReactionTest : public ::testing::Test
{
protected:
  bool step(const std::vector<JointCondition>& conditions)
  {
    actions.assign(JOINTS, DriveAction::NONE);
    return reaction.step(conditions, actions);
  }

  FaultReaction reaction{ std::make_unique<QuickStopPolicy>(), JOINTS };
  std::vector<DriveAction> actions;
};

TEST_F(FaultReactionTest, TheFirstCycleIsTheBaselineAndNeverFires)
{
  EXPECT_FALSE(step(with(0, JointCondition::FAULTED)));
  EXPECT_FALSE(reaction.any_latched());
  EXPECT_THAT(actions, ElementsAre(DriveAction::NONE, DriveAction::NONE, DriveAction::NONE));
}

TEST_F(FaultReactionTest, AJointEnteringAConditionQuickStopsEveryJointAndLatchesThem)
{
  for (const JointCondition condition :
       { JointCondition::FAULTED, JointCondition::UNREACHABLE, JointCondition::FOLLOWING_ERROR })
  {
    FaultReaction fresh{ std::make_unique<QuickStopPolicy>(), JOINTS };
    std::vector<DriveAction> fresh_actions(JOINTS, DriveAction::NONE);
    fresh.step(ALL_HEALTHY, fresh_actions);
    ASSERT_TRUE(fresh.step(with(1, condition), fresh_actions));
    EXPECT_THAT(fresh_actions, ElementsAre(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP, DriveAction::QUICK_STOP));
    EXPECT_THAT(fresh.event().triggered, ElementsAre(1));
    EXPECT_THAT(fresh.event().conditions, ElementsAre(condition));
    EXPECT_EQ(fresh.event().action, DriveAction::QUICK_STOP);
    EXPECT_THAT(fresh.event().acted_on, ElementsAre(0, 1, 2));
    for (std::size_t j = 0; j < JOINTS; ++j)
    {
      EXPECT_TRUE(fresh.latched(j));
    }
  }
}

TEST_F(FaultReactionTest, AJointStayingInItsConditionDoesNotFireAgain)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_FALSE(step(with(0, JointCondition::FAULTED)));
  EXPECT_THAT(actions, ElementsAre(DriveAction::NONE, DriveAction::NONE, DriveAction::NONE));
  EXPECT_TRUE(reaction.any_latched());
}

TEST_F(FaultReactionTest, AChangeBetweenConditionsIsNotANewEntry)
{
  // A faulted drive dropping off the bus is still the same event.
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_FALSE(step(with(0, JointCondition::UNREACHABLE)));
}

TEST_F(FaultReactionTest, ALatchedJointsConditionFlickeringDoesNotRefire)
{
  // A following error can come and go while the drive settles into its quick stop.
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FOLLOWING_ERROR)));
  EXPECT_FALSE(step(ALL_HEALTHY));
  EXPECT_FALSE(step(with(0, JointCondition::FOLLOWING_ERROR)));
  EXPECT_FALSE(step(ALL_HEALTHY));
  // After a clear, the next entry is a new event.
  reaction.clear({}, ALL_HEALTHY);
  EXPECT_FALSE(step(ALL_HEALTHY));
  EXPECT_TRUE(step(with(0, JointCondition::FOLLOWING_ERROR)));
}

TEST_F(FaultReactionTest, ASecondJointEnteringFiresAgainWhileLatched)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  std::vector<JointCondition> two = with(0, JointCondition::FAULTED);
  two[2] = JointCondition::UNREACHABLE;
  ASSERT_TRUE(step(two));
  EXPECT_THAT(reaction.event().triggered, ElementsAre(2));
  EXPECT_THAT(reaction.event().conditions, ElementsAre(JointCondition::UNREACHABLE));
}

TEST_F(FaultReactionTest, ClearEndsTheLatchAndAStillFaultedJointDoesNotRefire)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_EQ(reaction.clear({}, with(0, JointCondition::FAULTED)), ClearResult::CLEARED);
  EXPECT_FALSE(reaction.any_latched());
  EXPECT_FALSE(step(with(0, JointCondition::FAULTED)));
  EXPECT_FALSE(reaction.any_latched());
  EXPECT_EQ(reaction.clear({}, ALL_HEALTHY), ClearResult::NOTHING_LATCHED);
}

TEST_F(FaultReactionTest, ClearCanNameJoints)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_EQ(reaction.clear({ 1, 2 }, ALL_HEALTHY), ClearResult::CLEARED);
  EXPECT_TRUE(reaction.latched(0));
  EXPECT_FALSE(reaction.latched(1));
  EXPECT_FALSE(reaction.latched(2));
  EXPECT_EQ(reaction.clear({ 1 }, ALL_HEALTHY), ClearResult::NOTHING_LATCHED);
  EXPECT_TRUE(reaction.any_latched());
}

TEST_F(FaultReactionTest, AJointThatRecoversAndFaultsAgainFiresAgain)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  reaction.clear({}, ALL_HEALTHY);
  EXPECT_FALSE(step(ALL_HEALTHY));
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_TRUE(reaction.latched(0));
}

TEST_F(FaultReactionTest, AFiringDoesNotAllocate)
{
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  const auto capacity = reaction.event().acted_on.capacity();
  reaction.clear({}, ALL_HEALTHY);
  step(ALL_HEALTHY);
  ASSERT_TRUE(step(with(0, JointCondition::FAULTED)));
  EXPECT_EQ(reaction.event().acted_on.capacity(), capacity);
}

TEST_F(FaultReactionTest, AnEmergencyStopEventQuickStopsAndLatchesTheNamedJoints)
{
  step(ALL_HEALTHY);
  actions.assign(JOINTS, DriveAction::NONE);
  const PolicyDecision all = reaction.inject("emergency_stop", {}, ALL_HEALTHY, actions);
  EXPECT_TRUE(all.accepted);
  EXPECT_THAT(actions, ElementsAre(DriveAction::QUICK_STOP, DriveAction::QUICK_STOP, DriveAction::QUICK_STOP));
  EXPECT_STREQ(reaction.event().event_name, "emergency_stop");
  EXPECT_THAT(reaction.event().acted_on, ElementsAre(0, 1, 2));
  EXPECT_TRUE(reaction.event().triggered.empty());
  for (std::size_t j = 0; j < JOINTS; ++j)
  {
    EXPECT_TRUE(reaction.latched(j));
  }

  reaction.clear({}, ALL_HEALTHY);
  actions.assign(JOINTS, DriveAction::NONE);
  EXPECT_TRUE(reaction.inject("emergency_stop", { 2 }, ALL_HEALTHY, actions).accepted);
  EXPECT_THAT(actions, ElementsAre(DriveAction::NONE, DriveAction::NONE, DriveAction::QUICK_STOP));
  EXPECT_FALSE(reaction.latched(0));
  EXPECT_TRUE(reaction.latched(2));
}

TEST_F(FaultReactionTest, AnUnknownEventIsRefusedAndLatchesNothing)
{
  step(ALL_HEALTHY);
  actions.assign(JOINTS, DriveAction::NONE);
  const PolicyDecision decision = reaction.inject("coffee_break", {}, ALL_HEALTHY, actions);
  EXPECT_FALSE(decision.accepted);
  EXPECT_STREQ(decision.reason, "the built-in policy knows only the event 'emergency_stop'");
  EXPECT_THAT(actions, ElementsAre(DriveAction::NONE, DriveAction::NONE, DriveAction::NONE));
  EXPECT_FALSE(reaction.any_latched());
}

TEST(FaultReactionWithAPolicy, TheRunnerLatchesOnlyWhatThePolicyActsOn)
{
  FaultReaction reaction{ std::make_unique<FakePolicy>(), JOINTS };
  std::vector<DriveAction> actions(JOINTS, DriveAction::NONE);
  reaction.step(ALL_HEALTHY, actions);
  ASSERT_TRUE(reaction.step(with(1, JointCondition::FAULTED), actions));
  EXPECT_THAT(actions, ElementsAre(DriveAction::NONE, DriveAction::DISABLE, DriveAction::NONE));
  EXPECT_EQ(reaction.event().action, DriveAction::DISABLE);
  EXPECT_THAT(reaction.event().acted_on, ElementsAre(1));
  EXPECT_FALSE(reaction.latched(0));
  EXPECT_TRUE(reaction.latched(1));
  EXPECT_FALSE(reaction.latched(2));
}

TEST(FaultReactionWithAPolicy, ARefusedClearKeepsTheLatch)
{
  FaultReaction reaction{ std::make_unique<FakePolicy>(), JOINTS };
  std::vector<DriveAction> actions(JOINTS, DriveAction::NONE);
  reaction.step(ALL_HEALTHY, actions);
  ASSERT_TRUE(reaction.step(with(1, JointCondition::FAULTED), actions));
  EXPECT_EQ(reaction.clear({}, with(1, JointCondition::FAULTED)), ClearResult::REFUSED);
  EXPECT_STREQ(reaction.last_decision().reason, "a joint is still in a condition");
  EXPECT_TRUE(reaction.any_latched());
  EXPECT_EQ(reaction.clear({}, ALL_HEALTHY), ClearResult::CLEARED);
  EXPECT_FALSE(reaction.any_latched());
}

}  // namespace
