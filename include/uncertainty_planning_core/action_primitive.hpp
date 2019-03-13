#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace uncertainty_planning_core
{
namespace task_planner_adapter
{
/// Base type for all action primitives
template<typename State, typename StateAlloc=std::allocator<State>>
class ActionPrimitiveInterface
{
public:
  virtual ~ActionPrimitiveInterface() {}

  /// Returns true if the provided state is a candidare for the primitive
  virtual bool IsCandidate(const State& state) const = 0;

  /// Returns all possible outcomes of executing the primitive
  /// Each outcome is in a pair<State, bool> where the bool identifies
  /// if that outcome state is "nominally independent" - i.e. if the primitive
  /// is performed once and reches that outcome, is it possible for a repeat
  /// of the primitive to produce a different outcome?
  /// For example, a sensing primitive could produce the following result:
  /// [<Dog, false>, <Cat, false>, <None, true>]
  /// This result means that once the primitive has identified a Dog or Cat,
  /// it will identify the same Dog or Cat if repeated. However, in the "None"
  /// case where no animal is identified, calling it again may identify an
  /// animal that was not previously seen.
  virtual std::vector<std::pair<State, bool>>
  GetOutcomes(const State& state) = 0;

  /// Executes the primitive and returns the resulting state(s)
  /// Multiple returned states are only valid if *all* states are real,
  /// and you want the policy system to select easiest outcome to pursue.
  /// For example, if your actions operate at the object-level,
  /// a sensing primitive might return multiple states, each corresponding
  /// to a different object in the scene, so that the policy can select the
  /// easiest object to manipulate first.
  virtual std::vector<State, StateAlloc>
  Execute(const State& state) = 0;

  /// Returns the ranking of the primitive
  /// When multiple primitives can be applied to a given state, the planner
  /// will select the highest-ranked primitive. If multiple primitives with
  /// the same ranking are available, the planner will select the most
  /// recently added primitive.
  virtual double Ranking() const = 0;

  /// Returns the name of the primitive
  virtual std::string Name() const = 0;
};

template<typename State, typename StateAlloc=std::allocator<State>>
using ActionPrimitivePtr
  = std::shared_ptr<ActionPrimitiveInterface<State, StateAlloc>>;

/// Wrapper type to generate action primitive types from std::functions
/// Use this if you want to assemble primitives from a number of existing
/// functions or members and don't want to define a new type each time.
template<typename State, typename StateAlloc=std::allocator<State>>
class ActionPrimitiveWrapper
    : public ActionPrimitiveInterface<State, StateAlloc>
{
private:

  std::function<bool(const State&)> is_candidate_fn_;
  std::function<std::vector<std::pair<State, bool>>(
      const State&)> get_outcomes_fn_;
  std::function<std::vector<State, StateAlloc>(
      const State&)> execute_fn_;
  double ranking_;
  std::string name_;

public:

  ActionPrimitiveWrapper(
      const std::function<bool(const State&)>& is_candidate_fn,
      const std::function<std::vector<std::pair<State, bool>>(
                const State&)>& get_outcomes_fn,
      const std::function<std::vector<State, StateAlloc>(
                const State&)>& execute_fn,
      const double ranking, const std::string& name)
    : ActionPrimitiveInterface<State, StateAlloc>(),
      is_candidate_fn_(is_candidate_fn),
      get_outcomes_fn_(get_outcomes_fn),
      execute_fn_(execute_fn),
      ranking_(ranking), name_(name) {}

  virtual bool IsCandidate(const State& state) const
  {
    return is_candidate_fn_(state);
  }

  virtual std::vector<std::pair<State, bool>>
  GetOutcomes(const State& state)
  {
    return get_outcomes_fn_(state);
  }

  virtual std::vector<State, StateAlloc>
  Execute(const State& state)
  {
    return execute_fn_(state);
  }

  virtual double Ranking() const { return ranking_; }

  virtual std::string Name() const { return name_; }
};

}  // namespace task_planner_adapter
}  // namespace uncertainty_planning_core
