#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <random>
#include <atomic>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/conversions.hpp>
#include <common_robotics_utilities/print.hpp>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>
#include <uncertainty_planning_core/action_primitive.hpp>
#include <omp.h>

namespace uncertainty_planning_core
{
namespace task_planner_adapter
{
using DisplayFn = std::function<void(const visualization_msgs::MarkerArray&)>;

template<typename State, typename StateAlloc=std::allocator<State>>
using TaskPlannerClustering
  = SimpleOutcomeClusteringInterface<State, StateAlloc>;

template<typename State>
using TaskPlannerSampling = SimpleSamplerInterface<State, PRNG>;

template<typename State, typename StateAlloc=std::allocator<State>>
using TaskPlannerSimulator
  = SimpleSimulatorInterface<State, PRNG, StateAlloc>;

using common_robotics_utilities::simple_robot_model_interface
    ::SimpleRobotModelInterface;

/// Helper class to implement robot model interface for task planning problems
/// You should never need to directly instantiate or use a TaskStateRobot.
template<typename State, typename StateAlloc=std::allocator<State>>
class TaskStateRobot : public SimpleRobotModelInterface<State, StateAlloc>
{
private:

  State state_;
  std::function<uint64_t(const State&)> compute_readiness_fn_;

public:

  TaskStateRobot(
      const State& state,
      const std::function<uint64_t(const State&)> compute_readiness_fn)
    : SimpleRobotModelInterface<State, StateAlloc>(),
      state_(state), compute_readiness_fn_(compute_readiness_fn) {}

  virtual TaskStateRobot* Clone() const
  {
    return new TaskStateRobot(static_cast<const TaskStateRobot&>(*this));
  }

  virtual const State& GetPosition() const
  {
    return state_;
  }

  virtual const State& SetPosition(const State& state)
  {
    state_ = state;
    return GetPosition();
  }

  virtual std::vector<std::string> GetLinkNames() const
  {
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual Eigen::Isometry3d GetLinkTransform(
      const int64_t link_index) const
  {
    UNUSED(link_index);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual Eigen::Isometry3d GetLinkTransform(
      const std::string& link_name) const
  {
    UNUSED(link_name);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual common_robotics_utilities::math::VectorIsometry3d
  GetLinkTransforms() const
  {
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual common_robotics_utilities::math::MapStringIsometry3d
  GetLinkTransformsMap() const
  {
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual double ComputeConfigurationDistance(
      const State& state1, const State& state2) const
  {
    const double state1_readiness
        = static_cast<double>(compute_readiness_fn_(state1));
    const double state2_readiness
        = static_cast<double>(compute_readiness_fn_(state2));
    return std::abs(state2_readiness - state1_readiness);
  }

  virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(
      const State& state1, const State& state2) const
  {
    const double state1_readiness
        = static_cast<double>(compute_readiness_fn_(state1));
    const double state2_readiness
        = static_cast<double>(compute_readiness_fn_(state2));
    Eigen::VectorXd distances(1);
    distances(0) = state2_readiness - state1_readiness;
    return distances;
  }

  virtual State InterpolateBetweenConfigurations(
      const State& start,
      const State& end,
      const double ratio) const
  {
    UNUSED(start);
    UNUSED(end);
    UNUSED(ratio);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual State AverageConfigurations(
  const std::vector<State, StateAlloc>& configurations) const
  {
    UNUSED(configurations);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual Eigen::Matrix<double, 3, Eigen::Dynamic>
  ComputeLinkPointTranslationJacobian(
      const std::string& link_name,
      const Eigen::Vector4d& link_relative_point) const
  {
    UNUSED(link_name);
    UNUSED(link_relative_point);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }

  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  ComputeLinkPointJacobian(
      const std::string& link_name,
      const Eigen::Vector4d& link_relative_point) const
  {
    UNUSED(link_name);
    UNUSED(link_relative_point);
    throw std::runtime_error("Not a valid operation on TaskStateRobot");
  }
};

template<typename State, typename StateSerializer,
         typename StateAlloc=std::allocator<State>>
class TaskPlannerAdapter: public TaskPlannerClustering<State, StateAlloc>,
                          public TaskPlannerSampling<State>,
                          public TaskPlannerSimulator<State, StateAlloc>
{
private:

  typedef SimpleRobotModelInterface<State, StateAlloc> TaskStateRobotBaseType;

  typedef std::shared_ptr<TaskStateRobotBaseType> TaskStateRobotBasePtr;

  typedef ForwardSimulationStepTrace<State, StateAlloc> StateStepTrace;

  typedef UncertaintyPlannerState<
            State, StateSerializer, StateAlloc> TaskPlanningState;

  typedef UncertaintyPlanningTree<
            State, StateSerializer, StateAlloc> TaskPlanningTree;

  typedef ExecutionPolicy<
            State, StateSerializer, StateAlloc> TaskPlanningPolicy;

  typedef PolicyQueryResult<State> TaskPlanningPolicyQuery;

  typedef UncertaintyPlanningSpace<
            State, StateSerializer, StateAlloc, PRNG> TaskPlanningSpace;

  static void
  DeleteSamplerPtrFn(TaskPlannerSampling<State>* ptr)
  {
    UNUSED(ptr);
  }

  static void
  DeleteClusteringPtrFn(TaskPlannerClustering<State, StateAlloc>* ptr)
  {
    UNUSED(ptr);
  }

  static void
  DeleteSimulatorPtrFn(TaskPlannerSimulator<State, StateAlloc>* ptr)
  {
    UNUSED(ptr);
  }

  std::vector<ActionPrimitivePtr<State, StateAlloc>> primitives_;
  std::function<uint32_t(const State&)> state_readiness_fn_;

  std::function<bool(const State&)> single_execution_completed_fn_;
  std::function<bool(const State&)> task_completed_fn_;

  std::function<void(const std::string&, const int32_t)> logging_fn_;
  DisplayFn drawing_fn_;
  int32_t debug_level_;

  uint64_t state_counter_;
  uint64_t transition_id_;
  uint64_t split_id_;

  mutable std::vector<uncertainty_planning_core::PRNG> rngs_;
  mutable std::vector<std::uniform_real_distribution<double>>
    unit_real_distributions_;

  static inline size_t GetNumOMPThreads()
  {
    #if defined(_OPENMP)
    size_t num_threads = 0;
    #pragma omp parallel
    {
      num_threads = (size_t)omp_get_num_threads();
    }
    return num_threads;
    #else
    return 1;
    #endif
  }

  inline void ResetGenerators(const int64_t prng_seed)
  {
    // Prepare the default RNG
    uncertainty_planning_core::PRNG prng(prng_seed);
    // Temp seed distribution
    std::uniform_int_distribution<int64_t>
        seed_dist(0, std::numeric_limits<int64_t>::max());
    // Get the number of threads we're using
    const size_t num_threads = GetNumOMPThreads();
    // Prepare a number of PRNGs for each thread
    rngs_.clear();
    unit_real_distributions_.clear();
    for (size_t tidx = 0; tidx < num_threads; tidx++)
    {
      rngs_.push_back(uncertainty_planning_core::PRNG(seed_dist(prng)));
      unit_real_distributions_.push_back(
            std::uniform_real_distribution<double>(0.0, 1.0));
    }
  }

  std::vector<std::vector<size_t>> ClusterParticlesImpl(
      const std::vector<SimulationResult<State>>& particles)
  {
    std::map<uint64_t, std::vector<size_t>> cluster_map;
    for (size_t idx = 0; idx < particles.size(); idx++)
    {
      const State& config = particles[idx].ResultConfig();
      const uint64_t particle_readiness
          = ComputeStateReadiness(config);
      cluster_map[particle_readiness].push_back(idx);
    }
    std::vector<std::vector<size_t>> clusters;
    for (auto itr = cluster_map.begin(); itr != cluster_map.end(); ++itr)
    {
      clusters.push_back(itr->second);
    }
    return clusters;
  }

  std::vector<uint8_t> IdentifyClusterMembersImpl(
      const std::vector<State, StateAlloc>& cluster,
      const std::vector<SimulationResult<State>>& particles)
  {
    if (cluster.size() > 0)
    {
      const uint64_t parent_cluster_readiness
          = ComputeStateReadiness(cluster[0]);
      for (size_t idx = 1; idx < cluster.size(); idx++)
      {
        const uint64_t current_particle_readiness
            = ComputeStateReadiness(cluster[idx]);
        if (parent_cluster_readiness != current_particle_readiness)
        {
          throw std::runtime_error("Invalid parent cluster");
        }
      }
      std::vector<uint8_t> particle_cluster_membership(particles.size(), 0x00);
      for (size_t idx = 0; idx < particles.size(); idx++)
      {
        const State& config = particles[idx].ResultConfig();
        const uint64_t particle_readiness
            = ComputeStateReadiness(config);
        if (parent_cluster_readiness == particle_readiness)
        {
          particle_cluster_membership[idx] = 0x01;
        }
        else
        {
          particle_cluster_membership[idx] = 0x00;
        }
      }
      return particle_cluster_membership;
    }
    else
    {
      throw std::runtime_error("Invalid parent cluster with zero particles");
    }
  }

  std::vector<SimulationResult<State>> ForwardSimulatePrimitives(
      const std::vector<State, StateAlloc>& start_positions,
      const std::vector<State, StateAlloc>& target_positions)
  {
    Log("Starting primitive forward simulation...", 1);
    if (start_positions.size() > 0)
    {
      if ((target_positions.size() != 1)
             && (target_positions.size() != start_positions.size()))
      {
        throw std::invalid_argument(
              "target_positions.size() must be 1 or start_positions.size()");
      }
    }
    std::vector<SimulationResult<State>> propagated_points;
    propagated_points.reserve(start_positions.size());
    for (size_t idx = 0; idx < start_positions.size(); idx++)
    {
      const State& initial_particle = start_positions[idx];
      const std::vector<SimulationResult<State>> results
          = PerformBestAvailablePrimitive(initial_particle);
      propagated_points.insert(propagated_points.end(),
                               results.begin(),
                               results.end());
    }
    propagated_points.shrink_to_fit();
    Log("...finished primitive forward simulation", 1);
    return propagated_points;
  }

  std::vector<SimulationResult<State>>
  ReverseSimulatePrimitives(
      const std::vector<State, StateAlloc>& start_positions,
      const std::vector<State, StateAlloc>& target_positions)
  {
    Log("Starting primitive reverse simulation...", 1);
    if (start_positions.size() > 0)
    {
      if ((target_positions.size() != 1)
             && (target_positions.size() != start_positions.size()))
      {
        throw std::invalid_argument(
              "target_positions.size() must be 1 or start_positions.size()");
      }
    }
    std::vector<SimulationResult<State>> propagated_points;
    propagated_points.reserve(start_positions.size());
    for (size_t idx = 0; idx < start_positions.size(); idx++)
    {
      const State& initial_particle = start_positions[idx];
      const State& target_position
          = (target_positions.size() > 1) ? target_positions[idx]
                                          : target_positions[0];
      const std::vector<SimulationResult<State>> results
          = PerformTargettedPrimitive(initial_particle, target_position);
      propagated_points.insert(propagated_points.end(),
                               results.begin(),
                               results.end());
    }
    propagated_points.shrink_to_fit();
    Log("...finished particle reverse simulation", 1);
    return propagated_points;
  }

  std::vector<SimulationResult<State>>
  MakePrimitiveResults(
      const std::vector<State, StateAlloc>& raw_primitive_results,
      const bool is_primitive_outcome_nominally_independent) const
  {
    std::vector<SimulationResult<State>> primitive_results;
    primitive_results.reserve(raw_primitive_results.size());
    for (size_t idx = 0; idx < raw_primitive_results.size(); idx++)
    {
      primitive_results.emplace_back(
            SimulationResult<State>(raw_primitive_results[idx],
                             raw_primitive_results[idx],
                             false,
                             is_primitive_outcome_nominally_independent));
    }
    primitive_results.shrink_to_fit();
    return primitive_results;
  }

  int64_t GetBestPrimitiveIndex(const State& start)
  {
    int64_t best_primitive_idx = -1;
    double best_primitive_ranking = 0.0;
    for (size_t idx = 0; idx < primitives_.size(); idx++)
    {
      const ActionPrimitivePtr<State, StateAlloc>& primitive = primitives_[idx];
      if (primitive->IsCandidate(start))
      {
        const double primitive_ranking = primitive->Ranking();
        Log("Considering available primitive ["
            + primitive->Name() + "] with ranking "
            + std::to_string(primitive_ranking), 1);
        if (primitive_ranking >= best_primitive_ranking)
        {
          best_primitive_ranking = primitive_ranking;
          best_primitive_idx = static_cast<int64_t>(idx);
        }
      }
    }
    return best_primitive_idx;
  }

  std::vector<SimulationResult<State>>
  PerformBestAvailablePrimitive(const State& start)
  {
    const int64_t best_primitive_idx = GetBestPrimitiveIndex(start);
    if (best_primitive_idx >= 0)
    {
      const ActionPrimitivePtr<State, StateAlloc>& best_primitive
          = primitives_[static_cast<size_t>(best_primitive_idx)];
      Log("Performing best available primitive ["
          + best_primitive->Name() + "] with ranking "
          + std::to_string(best_primitive->Ranking()), 2);
      const std::vector<std::pair<State, bool>> primitive_results
          = best_primitive->GetOutcomes(start);
      // Package the results
      std::vector<SimulationResult<State>> complete_results;
      complete_results.reserve(primitive_results.size());
      for (size_t idx = 0; idx < primitive_results.size(); idx++)
      {
        const State& result_state = primitive_results[idx].first;
        const bool outcome_is_nominally_independent
            = primitive_results[idx].second;
        const bool did_contact = false; // Contact has no meaning here
        complete_results.emplace_back(
              SimulationResult<State>(result_state,
                               result_state,
                               did_contact,
                               outcome_is_nominally_independent));
      }
      complete_results.shrink_to_fit();
      return complete_results;
    }
    else
    {
      throw std::runtime_error(
          "No available primitive to handle state "
          + common_robotics_utilities::print::Print(start));
    }
  }

  std::vector<State, StateAlloc>
  ExecuteBestAvailablePrimitive(const State& start)
  {
    const int64_t best_primitive_idx = GetBestPrimitiveIndex(start);
    if (best_primitive_idx >= 0)
    {
      const ActionPrimitivePtr<State, StateAlloc>& best_primitive
          = primitives_[static_cast<size_t>(best_primitive_idx)];
      Log("Executing best available primitive ["
          + best_primitive->Name() + "] with ranking "
          + std::to_string(best_primitive->Ranking()), 2);
      if (GetDebugLevel() >= 1)
      {
        Log("Press ENTER to continue...", 4);
        std::cin.get();
      }
      return best_primitive->Execute(start);
    }
    else
    {
      throw std::runtime_error(
          "No available primitive to handle state "
          + common_robotics_utilities::print::Print(start));
    }
  }

  std::vector<SimulationResult<State>>
  PerformTargettedPrimitive(const State& start, const State& target)
  {
    const uint64_t start_readiness = ComputeStateReadiness(start);
    const uint64_t target_readiness = ComputeStateReadiness(target);
    if (start_readiness < target_readiness)
    {
      Log("We are less ready than our parent", 2);
      return PerformBestAvailablePrimitive(start);
    }
    else if (start_readiness > target_readiness)
    {
      Log("We are more ready than our parent", 2);
      const std::vector<State, StateAlloc> outcome_configs(1, start);
      return MakePrimitiveResults(outcome_configs, true);
    }
    else
    {
      Log("Performed no-op reverse", 2);
      const std::vector<State, StateAlloc> outcome_configs(1, start);
      return MakePrimitiveResults(outcome_configs, true);
    }
  }

  int64_t NearestNeighborsFn(
      const TaskPlanningTree& tree,
      const TaskPlanningState& sampled_state) const
  {
    UNUSED(sampled_state);
    // We only consider the start state if nothing has been expanded further!
    if (tree.size() == 1)
    {
      return 0;
    }
    // Get the nearest neighbor (ignoring the disabled states)
    std::vector<std::pair<int64_t, uint64_t>>
        per_thread_bests(GetNumOMPThreads(),
                         std::pair<int64_t, uint64_t>(-1, 0));
    // Greedy best-first expansion strategy
    #pragma omp parallel for
    for (size_t idx = 1; idx < tree.size(); idx++)
    {
      auto& current_state = tree[idx];
      // Only check against states enabled for NN checks
      if (current_state.GetValueImmutable().UseForNearestNeighbors())
      {
        auto particles
            = current_state.GetValueImmutable().GetParticlePositionsImmutable();
        const State& representative_particle = particles.Value().at(0);
        const uint64_t state_readiness
            = ComputeStateReadiness(representative_particle);
  #if defined(_OPENMP)
        const size_t current_thread_id = (size_t)omp_get_thread_num();
  #else
        const size_t current_thread_id = 0;
  #endif
        if (state_readiness > per_thread_bests[current_thread_id].second)
        {
          per_thread_bests[current_thread_id].first = (int64_t)idx;
          per_thread_bests[current_thread_id].second = state_readiness;
        }
      }
    }
    int64_t best_index = -1;
    uint64_t best_state_readiness = 0;
    for (size_t idx = 0; idx < per_thread_bests.size(); idx++)
    {
      const uint64_t thread_best_state_readiness = per_thread_bests[idx].second;
      const int64_t thread_best_index = per_thread_bests[idx].first;
      if (thread_best_index >= 0)
      {
        if (thread_best_state_readiness > best_state_readiness)
        {
          best_index = thread_best_index;
          best_state_readiness = thread_best_state_readiness;
        }
      }
    }
    if (best_index >= 0)
    {
      const TaskPlanningState& best_state
          = tree.at(best_index).GetValueImmutable();
      Log("Selected node " + std::to_string(best_index)
          + " with state " + common_robotics_utilities::print::Print(best_state)
          + " as best neighbor (Qnear)", 2);
      return best_index;
    }
    else
    {
      throw std::runtime_error("Planner should already have terminated");
    }
  }

  std::vector<SimulationResult<State>>
  PerformParticlePropagation(const TaskPlanningState& nearest,
                             const TaskPlanningState& target,
                             const bool simulate_reverse)
  {
    // Get the initial particles - we use dynamic # of particles based on
    // the number of multiple outcomes we encounter
    const std::vector<State, StateAlloc> initial_particles
        = nearest.CollectParticles(nearest.GetNumParticles());
    // Forward propagate each of the particles
    std::vector<State, StateAlloc> target_position;
    target_position.reserve(1);
    target_position.push_back(target.GetExpectation());
    target_position.shrink_to_fit();
    std::vector<SimulationResult<State>> propagated_points;
    if (simulate_reverse == false)
    {
      propagated_points = ForwardSimulatePrimitives(initial_particles,
                                                    target_position);
    }
    else
    {
      propagated_points = ReverseSimulatePrimitives(initial_particles,
                                                    target_position);
    }
    return propagated_points;
  }

  std::vector<std::vector<SimulationResult<State>>>
  PerformStateClustering(
      const std::vector<SimulationResult<State>>& particles)
  {
    // Make sure there are particles to cluster
    if (particles.size() == 0)
    {
      Log("Single cluster with no states", 1);
      return
          std::vector<std::vector<SimulationResult<State>>>();
    }
    else if (particles.size() == 1)
    {
      Log("Single cluster with one state "
          + common_robotics_utilities::print::Print(particles.front()), 1);
      return std::vector<std::vector<SimulationResult<State>>>{
                particles};
    }
    const std::vector<std::vector<size_t>> index_clusters
        = ClusterParticlesImpl(particles);
    // Before we return, we need to convert the index clusters to State clusters
    std::vector<std::vector<SimulationResult<State>>> clusters;
    clusters.reserve(index_clusters.size());
    size_t total_particles = 0;
    Log("Clustering produced " + std::to_string(index_clusters.size())
            + " clusters from " + std::to_string(particles.size())
            + " propagated states", 1);
    for (size_t cluster_idx = 0;
         cluster_idx < index_clusters.size();
         cluster_idx++)
    {
      const std::vector<size_t>& cluster = index_clusters[cluster_idx];
      std::vector<SimulationResult<State>> final_cluster;
      final_cluster.reserve(cluster.size());
      for (size_t element_idx = 0; element_idx < cluster.size(); element_idx++)
      {
        total_particles++;
        const size_t particle_idx = cluster[element_idx];
        const SimulationResult<State>& particle = particles.at(particle_idx);
        final_cluster.push_back(particle);
      }
      final_cluster.shrink_to_fit();
      Log("Cluster " + std::to_string(cluster_idx) + " with "
          + std::to_string(final_cluster.size()) + " states "
          + common_robotics_utilities::print::Print(final_cluster), 1);
      clusters.push_back(final_cluster);
    }
    clusters.shrink_to_fit();
    if (total_particles != particles.size())
    {
      throw std::runtime_error("total_particles != particles.size()");
    }
    return clusters;
  }

  std::pair<uint32_t, uint32_t>
  ComputeReverseEdgeProbability(const TaskPlanningState& parent,
                                const TaskPlanningState& child)
  {
    const std::vector<SimulationResult<State>>
        simulation_result = PerformParticlePropagation(child, parent, true);
    std::vector<uint8_t> parent_cluster_membership;
    if (parent.HasParticles())
    {
      parent_cluster_membership
          = IdentifyClusterMembersImpl(
              parent.GetParticlePositionsImmutable().Value(),
              simulation_result);
    }
    else
    {
      const std::vector<State, StateAlloc> parent_cluster(
            1, parent.GetExpectation());
      parent_cluster_membership
          = IdentifyClusterMembersImpl(parent_cluster, simulation_result);
    }
    uint32_t reached_parent = 0u;
    // Get the target position;
    for (size_t ndx = 0; ndx < parent_cluster_membership.size(); ndx++)
    {
      if (parent_cluster_membership[ndx] > 0)
      {
        reached_parent++;
      }
    }
    const uint32_t simulated_particles = (uint32_t)simulation_result.size();
    return std::make_pair(simulated_particles, reached_parent);
  }

  std::vector<std::pair<TaskPlanningState, int64_t>>
  PerformStatePropagation(const TaskPlanningState& nearest,
                          const TaskPlanningState& target,
                          const TaskStateRobotBasePtr& robot_ptr,
                          const double step_size,
                          const uint32_t planner_action_try_attempts)
  {
    // Increment the transition ID
    transition_id_++;
    const uint64_t current_forward_transition_id = transition_id_;
    // Forward propagate each of the particles
    std::vector<SimulationResult<State>> propagated_points
        = PerformParticlePropagation(nearest, target, false);
    // Cluster the live particles into (potentially) multiple states
    const std::vector<std::vector<SimulationResult<State>>>&
        particle_clusters = PerformStateClustering(propagated_points);
    bool is_split_child = false;
    if (particle_clusters.size() > 1)
    {
      is_split_child = true;
      split_id_++;
    }
    // Build the forward-propagated states
    const State control_target = target.GetExpectation();
    std::vector<std::pair<TaskPlanningState, int64_t>> result_states(
          particle_clusters.size());
    for (size_t idx = 0; idx < particle_clusters.size(); idx++)
    {
      const std::vector<SimulationResult<State>>& current_cluster
          = particle_clusters[idx];
      if (current_cluster.size() > 0)
      {
        state_counter_++;
        const uint32_t attempt_count = (uint32_t)propagated_points.size();
        const uint32_t reached_count = (uint32_t)current_cluster.size();
        std::vector<State, StateAlloc> particle_locations;
        particle_locations.reserve(current_cluster.size());
        bool action_is_nominally_independent = true;
        for (size_t pdx = 0; pdx < current_cluster.size(); pdx++)
        {
          particle_locations.push_back(current_cluster[pdx].ResultConfig());
          if (current_cluster[pdx].OutcomeIsNominallyIndependent() == false)
          {
            action_is_nominally_independent = false;
          }
        }
        particle_locations.shrink_to_fit();
        const uint32_t reverse_attempt_count = (uint32_t)current_cluster.size();
        const uint32_t reverse_reached_count = 0u;
        const double effective_edge_feasibility
            = (double)reached_count / (double)attempt_count;
        transition_id_++;
        const uint64_t new_state_reverse_transtion_id = transition_id_;
        TaskPlanningState propagated_state(
              state_counter_, particle_locations, attempt_count, reached_count,
              effective_edge_feasibility,
              reverse_attempt_count, reverse_reached_count,
              nearest.GetMotionPfeasibility(), step_size, control_target,
              current_forward_transition_id, new_state_reverse_transtion_id,
              ((is_split_child) ? split_id_ : 0u),
              action_is_nominally_independent);
        propagated_state.UpdateStatistics(robot_ptr);
        // Compute reversibility
        const std::pair<uint32_t, uint32_t> reverse_edge_check
            = ComputeReverseEdgeProbability(nearest, propagated_state);
        propagated_state.UpdateReverseAttemptAndReachedCounts(
              reverse_edge_check.first, reverse_edge_check.second);
        // Store the state
        result_states[idx].first = propagated_state;
        result_states[idx].second = -1;
      }
    }
    Log("Forward simultation produced " + std::to_string(result_states.size())
        + " states", 1);
    // We only do further processing if a split happened
    if (result_states.size() > 1)
    {
      // Now that we have the forward-propagated states, we go back and update
      // their effective edge P(feasibility)
      for (size_t idx = 0; idx < result_states.size(); idx++)
      {
        TaskPlanningState& current_state = result_states[idx].first;
        double percent_active = 1.0;
        double p_reached = 0.0;
        for (uint32_t try_attempt = 0;
             try_attempt < planner_action_try_attempts;
             try_attempt++)
        {
          // How many particles got to our state on this attempt?
          p_reached
              += (percent_active * current_state.GetRawEdgePfeasibility());
          // Update the percent of particles that are still usefully active
          double updated_percent_active = 0.0;
          for (size_t other_idx = 0;
               other_idx < result_states.size();
               other_idx++)
          {
            if (other_idx != idx)
            {
              const TaskPlanningState& other_state
                  = result_states[other_idx].first;
              // Only if this state has nominally independent outcomes can we
              // expect particles that return to the parent to actually reach
              // a different outcome in future repeats
              if (other_state.IsActionOutcomeNominallyIndependent())
              {
                const double p_reached_other
                    = percent_active * other_state.GetRawEdgePfeasibility();
                const double p_returned_to_parent
                    = p_reached_other
                      * other_state.GetReverseEdgePfeasibility();
                updated_percent_active += p_returned_to_parent;
              }
            }
          }
          percent_active = updated_percent_active;
        }
        if ((p_reached >= 0.0) && (p_reached <= 1.0))
        {
          current_state.SetEffectiveEdgePfeasibility(p_reached);
        }
        else if ((p_reached >= 0.0) && (p_reached <= 1.001))
        {
          Log("WARNING - P(reached) = " + std::to_string(p_reached)
              + " > 1.0 (probably numerical error)", 1);
          p_reached = 1.0;
          current_state.SetEffectiveEdgePfeasibility(p_reached);
        }
        else
        {
          throw std::runtime_error("p_reached out of range [0, 1]");
        }
        Log("Computed effective edge P(feasibility) of "
            + std::to_string(p_reached) + " for "
            + std::to_string(planner_action_try_attempts)
            + " try/retry attempts", 1);
      }
    }
    return result_states;
  }

  bool IsSingleExecutionCompleted(const State& state) const
  {
    return single_execution_completed_fn_(state);
  }

  bool IsTaskCompleted(const State& state) const
  {
    return task_completed_fn_(state);
  }

  void Log(const std::string& message, const int32_t level) const
  {
    logging_fn_(message, level);
  }

  void Draw(const visualization_msgs::MarkerArray& markers)
  {
    drawing_fn_(markers);
  }

  uint64_t ComputeStateReadiness(const State& state) const
  {
    return state_readiness_fn_(state);
  }

public:

  /// Constructs are task planning adapter for the given task
  /// Parameters:
  /// - state_readiness_fn: function to compute resdiness of a given state
  ///   Readiness is roughly equivalent to progress, with higher-readiness
  ///   state corresponding to more progress through the task. The planner will
  ///   always explore the highest-readiness state that is available to expand.
  /// - single_execution_completed_fn: function to determine if a single
  ///   execution through the policy has been completed.
  /// - task_completed_fn: function to determine if the entire task has been
  ///   completed.
  /// - logging_fn: function for text logging.
  /// - display_fn: function to display visualization_msgs::MarkerArray
  ///   visualizations.
  /// - prng_seed: seed value to random number generators.
  /// - debug_level: internal level to control logging verbosity.
  TaskPlannerAdapter(
      const std::function<uint64_t(const State&)>& state_readiness_fn,
      const std::function<bool(const State&)>& single_execution_completed_fn,
      const std::function<bool(const State&)>& task_completed_fn,
      const std::function<void(const std::string&, const int32_t)>& logging_fn,
      const DisplayFn& drawing_fn,
      const int64_t prng_seed,
      const int32_t debug_level)
    : TaskPlannerClustering<State, StateAlloc>(),
      TaskPlannerSampling<State>(),
      TaskPlannerSimulator<State, StateAlloc>(),
      state_readiness_fn_(state_readiness_fn),
      single_execution_completed_fn_(single_execution_completed_fn),
      task_completed_fn_(task_completed_fn),
      logging_fn_(logging_fn),
      drawing_fn_(drawing_fn),
      debug_level_(debug_level)
  {
    ResetGenerators(prng_seed);
    ResetStatistics();
  }

  /// Plan a task policy
  /// Returns task policy and <string, double> dictionary of planning statistics
  /// Parameters:
  /// - Starting state
  /// - Time limit
  /// - Threshold of P(task completed) at which to stop planning
  /// - Max number of repeats of each action to consider when computing
  ///   edge transition probabilities
  /// - Max number of repeats of each action to consider when computing
  ///   expected edge costs in the policy
  std::pair<TaskPlanningPolicy, std::map<std::string, double>>
  PlanPolicy(const State& start_state,
             const double time_limit,
             const double p_task_done_termination_threshold,
             const double minimum_goal_candiate_probability,
             const uint32_t edge_attempt_count,
             const uint32_t policy_action_attempt_count)
  {
    std::function<uint64_t(const State&)> compute_readiness_fn
        = [&] (const State& state)
    {
      return ComputeStateReadiness(state);
    };
    std::shared_ptr<TaskStateRobot<State, StateAlloc>> robot_ptr(
          new TaskStateRobot<State, StateAlloc>(start_state,
                                                compute_readiness_fn));
    std::shared_ptr<TaskPlannerSampling<State>> sampling_ptr(
          this, DeleteSamplerPtrFn);
    std::shared_ptr<TaskPlannerSimulator<State, StateAlloc>> simulator_ptr(
          this, DeleteSimulatorPtrFn);
    std::shared_ptr<TaskPlannerClustering<State, StateAlloc>> clustering_ptr(
          this, DeleteClusteringPtrFn);
    const double step_size = std::numeric_limits<double>::infinity();
    TaskPlanningSpace planning_space(debug_level_, 0,
                                     step_size,
                                     0.0,
                                     minimum_goal_candiate_probability,
                                     0.75, 0.75, false,
                                     robot_ptr,
                                     sampling_ptr,
                                     simulator_ptr,
                                     clustering_ptr,
                                     logging_fn_);
    const std::chrono::duration<double> planner_time_limit(time_limit);
    const std::function<int64_t(const TaskPlanningTree&,
                                const TaskPlanningState&)> nearest_neighbor_fn
        = [&] (const TaskPlanningTree& tree, const TaskPlanningState& sample)
    {
      return NearestNeighborsFn(tree, sample);
    };
    const std::function<std::vector<std::pair<TaskPlanningState, int64_t>>(
          const TaskPlanningState&,
          const TaskPlanningState&)> forward_propagation_fn
        = [&] (const TaskPlanningState& start, const TaskPlanningState& target)
    {
      return PerformStatePropagation(
            start, target, robot_ptr, step_size, edge_attempt_count);
    };
    const std::function<bool(const State&)> goal_reached_fn
        = [&] (const State& candidate)
    {
      return IsSingleExecutionCompleted(candidate);
    };
    const std::function<double(const TaskPlanningState&)> goal_probability_fn
        = [&] (const TaskPlanningState& candidate_goal_state)
    {
      return uncertainty_planning_core::UserGoalCheckWrapperFn(
            candidate_goal_state, goal_reached_fn);
    };
    ResetStatistics();
    return planning_space.PlanGoalSampling(start_state, 0.0,
                                           nearest_neighbor_fn,
                                           forward_propagation_fn,
                                           goal_probability_fn,
                                           planner_time_limit,
                                           edge_attempt_count,
                                           policy_action_attempt_count,
                                           true, true,
                                           0.0,
                                           p_task_done_termination_threshold,
                                           drawing_fn_);
  }

  /// Execute a task policy by repeatedly executing the policy until the task
  /// has been completed
  /// Returns the policy with edge transition probabilites updated from the
  /// results of policy executions, as well as a <string, double> dictionary
  /// of policy execution statistics
  /// Parameters:
  /// - Task policy
  /// - Function to initialize each execution of the policy and return the
  ///   initial state to start policy execution from
  /// - User-defined pre-action callback function
  /// - User-defined post-action callback function
  /// - Max number of execution steps in each single policy execution
  /// - Max number of policy executions to perform to complete the task
  /// - Allow transition probability learning accross all policy executions
  ///   or limit learning to a single policy execution at a time
  std::pair<TaskPlanningPolicy, std::map<std::string, double>>
  ExecutePolicy(const TaskPlanningPolicy& starting_policy,
                const std::function<State(void)>& exec_initialization_fn,
                const std::function<void(const State&, const State&)>&
                    pre_action_callback_fn,
                const std::function<void(
                    const std::vector<State, StateAlloc>&, const int64_t)>&
                        post_outcome_callback_fn,
                const int64_t max_policy_exec_steps,
                const int64_t max_policy_executions,
                const bool allow_branch_jumping,
                const bool enable_cumulative_learning)
  {
    TaskPlanningPolicy policy = starting_policy;
    // We don't know what logging function the policy has, but we want it to use
    // ours for consistency
    policy.RegisterLoggingFunction(logging_fn_);
    int64_t num_executions = 0;
    int64_t successful_executions = 0;
    bool task_execution_successful = false;
    // Make outcome clustering function used in policy queries
    std::function<bool(const std::vector<State, StateAlloc>&, const State&)>
        policy_outcome_clustering_fn
        = [&] (const std::vector<State, StateAlloc>& particles,
               const State& result_state)
    {
      std::vector<SimulationResult<State>> result_particles;
      result_particles.emplace_back(
            SimulationResult<State>(result_state, result_state, false, false));
      const std::vector<uint8_t> cluster_membership
          = IdentifyClusterMembersImpl(particles, result_particles);
      const uint8_t parent_cluster_membership = cluster_membership.at(0);
      if (parent_cluster_membership > 0x00)
      {
          return true;
      }
      else
      {
          return false;
      }
    };
    // Execute until done or out of iterations
    while ((task_execution_successful == false)
           && (num_executions < max_policy_executions))
    {
      num_executions++;
      TaskPlanningPolicy working_policy = policy;
      uint64_t desired_transition_id = 0;
      int64_t policy_exec_steps = 0;
      bool policy_execution_successful = false;
      // Perform initialization & get a starting state
      const State starting_state = exec_initialization_fn();
      if (IsTaskCompleted(starting_state))
      {
        Log("Initial state for execution " + std::to_string(num_executions + 1)
            + " meets task completion conditions", 3);
        task_execution_successful = true;
        policy_execution_successful = true;
      }
      else if (IsSingleExecutionCompleted(starting_state))
      {
        Log("Initial state for execution " + std::to_string(num_executions + 1)
            + " meets execution completion conditions", 3);
        policy_execution_successful = true;
      }
      std::vector<State, StateAlloc> execution_trace;
      execution_trace.push_back(starting_state);
      // Step until done or out of iterations
      while ((policy_execution_successful == false)
             && (task_execution_successful == false)
             && (policy_exec_steps < max_policy_exec_steps))
      {
        policy_exec_steps++;
        const State& current_state = execution_trace.back();
        // Real policy query
        const TaskPlanningPolicyQuery policy_query_response
            = working_policy.QueryBestAction(
                desired_transition_id, current_state, allow_branch_jumping,
                true, policy_outcome_clustering_fn);
        desired_transition_id = policy_query_response.DesiredTransitionId();
        const State& action = policy_query_response.Action();
        const bool is_reverse_action = policy_query_response.IsReverseAction();
        pre_action_callback_fn(current_state, action);
        // Perform the action
        std::vector<State, StateAlloc> action_results;
        if (is_reverse_action)
        {
          const uint64_t start_readiness = ComputeStateReadiness(current_state);
          const uint64_t target_readiness = ComputeStateReadiness(action);
          if (start_readiness < target_readiness)
          {
            Log("Less ready than parent, ExecuteBestAvailablePrimitive", 2);
            action_results = ExecuteBestAvailablePrimitive(current_state);
          }
          else if (start_readiness > target_readiness)
          {
            Log("More ready than parent, ExecuteBestAvailablePrimitive", 2);
            action_results = ExecuteBestAvailablePrimitive(current_state);
          }
          else
          {
            Log("Performed no-op reverse", 2);
            action_results.push_back(current_state);
          }
        }
        else
        {
          Log("Moving forwards, ExecuteBestAvailablePrimitive", 2);
          action_results = ExecuteBestAvailablePrimitive(current_state);
        }
        // Identify the "ideal" outcome
        // Because we want purely speculative queries, we make a copy
        auto speculative_policy_copy = working_policy;
        std::function<void(const std::string&, const int32_t)> null_logger
            = [] (const std::string&, const int32_t) {};
        speculative_policy_copy.RegisterLoggingFunction(null_logger);
        Log("Speculatively querying the policy to identify best outcome of "
            + std::to_string(action_results.size()) + " outcomes...", 2);
        double best_outcome_cost = std::numeric_limits<double>::infinity();
        int64_t best_outcome_idx = -1;
        for (size_t idx = 0; idx < action_results.size(); idx++)
        {
          const State& candidate_outcome = action_results[idx];
          const TaskPlanningPolicyQuery speculative_query_response
              = speculative_policy_copy.QueryBestAction(
                  desired_transition_id, candidate_outcome,
                  allow_branch_jumping, true, policy_outcome_clustering_fn);
          const double expected_cost
              = speculative_query_response.ExpectedCostToGoal();
          if (expected_cost < best_outcome_cost)
          {
            best_outcome_cost = expected_cost;
            best_outcome_idx = static_cast<int64_t>(idx);
          }
        }
        if (best_outcome_idx >= 0)
        {
          Log("Out of " + std::to_string(action_results.size())
              + " results selected best outcome "
              + std::to_string(best_outcome_idx) + " with expected cost "
              + std::to_string(best_outcome_cost), 2);
          const State& best_outcome = action_results[best_outcome_idx];
          execution_trace.push_back(best_outcome);
          post_outcome_callback_fn(action_results, best_outcome_idx);
        }
        else
        {
          throw std::runtime_error(
                "Could not identify a best outcome out of "
                + std::to_string(action_results.size()) + " results");
        }
        const State& outcome = execution_trace.back();
        Log("Outcome state is: "
            + common_robotics_utilities::print::Print(outcome), 1);
        if (IsTaskCompleted(outcome))
        {
          Log("Outcome state for execution " + std::to_string(num_executions)
              + " at policy step " + std::to_string(policy_exec_steps)
              + " meets task completion conditions", 2);
          task_execution_successful = true;
          policy_execution_successful = true;
        }
        else if (IsSingleExecutionCompleted(outcome))
        {
          Log("Outcome state for execution " + std::to_string(num_executions)
              + " at policy step " + std::to_string(policy_exec_steps)
              + " meets single execution completion conditions", 2);
          policy_execution_successful = true;
        }
      }
      // Update statistics
      if (policy_execution_successful)
      {
        successful_executions++;
        Log("Finished policy execution " + std::to_string(num_executions)
            + " successfully, " + std::to_string(successful_executions)
            + " successful so far", 2);
      }
      else
      {
        Log("Finished policy execution " + std::to_string(num_executions)
            + " unsuccessfully, " + std::to_string(successful_executions)
            + " successful so far", 3);
      }
      // Check the final state to see  if we're done the task
      if (IsTaskCompleted(execution_trace.back()))
      {
        Log("Finished task execution in " + std::to_string(num_executions)
            + " policy executions, of which "
            + std::to_string(successful_executions) + " were successful", 2);
        task_execution_successful = true;
      }
      // Update the policy (if enabled)
      if (enable_cumulative_learning)
      {
        policy = working_policy;
      }
    }
    if (task_execution_successful == false)
    {
      Log("Failed to complete task execution in "
          + std::to_string(num_executions) + " policy executions, of which "
          + std::to_string(successful_executions) + " were successful", 4);
    }
    const double policy_success
        = (double)successful_executions / (double)num_executions;
    std::map<std::string, double> policy_statistics;
    policy_statistics["Execution policy success"] = policy_success;
    policy_statistics["Task execution successful"]
        = (task_execution_successful) ? 1.0 : 0.0;
    policy_statistics["successful policy executions"]
        = static_cast<double>(successful_executions);
    policy_statistics["number of policy executions"]
        = static_cast<double>(num_executions);
    return std::make_pair(policy, policy_statistics);
  }

  /// Add a new primitive
  /// Primitives must have unique names, but they can have the same ranking
  /// If multiple primitives share the same ranking and are candidates for a
  /// given state, the last one encountered will be selected
  void
  RegisterPrimitive(const ActionPrimitivePtr<State, StateAlloc>& new_primitive)
  {
    for (size_t idx = 0; idx < primitives_.size(); idx++)
    {
      const ActionPrimitivePtr<State, StateAlloc>& primitive = primitives_[idx];
      if (primitive->Name() == new_primitive->Name())
      {
        throw std::invalid_argument("New planning primitive with name ["
                                    + new_primitive->Name()
                                    + "] cannot share name with existing ["
                                    + primitive->Name() + "]");
      }
      if (primitive->Ranking() == new_primitive->Ranking())
      {
        Log("New planning primitive [" + new_primitive->Name()
            + "] has the same ranking ["
            + std::to_string(new_primitive->Ranking())
            + "] as existing primitive ["
            + primitive->Name() + "] with ranking ["
            + std::to_string(primitive->Ranking())
            + "] - This may be OK, but it can cause unexpected behavior", 3);
      }
    }
    primitives_.push_back(new_primitive);
  }

  void ClearPrimitives()
  {
    primitives_.clear();
  }

  std::string GetBestPrimitiveName(const State& state)
  {
    const int64_t best_primitive_idx = GetBestPrimitiveIndex(state);
    if (best_primitive_idx >= 0)
    {
      const ActionPrimitivePtr<State, StateAlloc>& best_primitive
          = primitives_[static_cast<size_t>(best_primitive_idx)];
      return best_primitive->Name();
    }
    else
    {
      throw std::runtime_error(
          "No available primitive to handle state "
          + common_robotics_utilities::print::Print(state));
    }
  }

  void SetStateReadinessFn(const std::function<uint32_t(const State&)>& fn)
  {
    state_readiness_fn_ = fn;
  }

  void
  SetSingleExecutionCompletedFn(const std::function<bool(const State&)>& fn)
  {
    single_execution_completed_fn_ = fn;
  }

  void SetTaskCompletedFn(const std::function<bool(const State&)>& fn)
  {
    task_completed_fn_ = fn;
  }

  virtual int32_t GetDebugLevel() const
  {
    return debug_level_;
  }

  virtual int32_t SetDebugLevel(const int32_t debug_level)
  {
    debug_level_ = debug_level;
    return debug_level_;
  }

  virtual uncertainty_planning_core::PRNG& GetRandomGenerator()
  {
  #if defined(_OPENMP)
    const size_t thread_id = (size_t)omp_get_thread_num();
  #else
    const size_t thread_id = 0;
  #endif
    return rngs_[thread_id];
  }

  virtual std::map<std::string, double> GetStatistics() const
  {
    std::map<std::string, double> statistics;
    statistics["state_counter"] = (double)state_counter_;
    statistics["transition_id"] = (double)transition_id_;
    statistics["split_id"] = (double)split_id_;
    return statistics;
  }

  virtual void ResetStatistics()
  {
    state_counter_ = 0;
    transition_id_ = 0;
    split_id_ = 0;
  }

  virtual std::vector<std::vector<size_t>> ClusterParticles(
    const TaskStateRobotBasePtr& robot,
    const std::vector<SimulationResult<State>>& particles,
    const DisplayFn& display_fn)
  {
    UNUSED(robot);
    UNUSED(display_fn);
    return ClusterParticlesImpl(particles);
  }

  virtual std::vector<uint8_t> IdentifyClusterMembers(
    const TaskStateRobotBasePtr& robot,
    const std::vector<State, StateAlloc>& cluster,
    const std::vector<SimulationResult<State>>& particles,
    const DisplayFn& display_fn)
  {
    UNUSED(robot);
    UNUSED(display_fn);
    return IdentifyClusterMembersImpl(cluster, particles);
  }

  virtual State Sample(uncertainty_planning_core::PRNG& prng)
  {
    UNUSED(prng);
    return State();
  }

  /// Dummy implementations to satisfy interfaces
  /// None of these functions are meaningfully used, so all of them return
  /// empty values or throw std::runtime_error

  virtual State SampleGoal(uncertainty_planning_core::PRNG& prng)
  {
    UNUSED(prng);
    return State();
  }

  virtual std::string GetFrame() const
  {
    return "world";
  }

  virtual visualization_msgs::MarkerArray MakeEnvironmentDisplayRep() const
  {
    return visualization_msgs::MarkerArray();
  }

  virtual visualization_msgs::MarkerArray MakeConfigurationDisplayRep(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& configuration,
      const std_msgs::ColorRGBA& color,
      const int32_t starting_index,
      const std::string& config_marker_ns) const
  {
    UNUSED(immutable_robot);
    UNUSED(configuration);
    UNUSED(color);
    UNUSED(starting_index);
    UNUSED(config_marker_ns);
    return visualization_msgs::MarkerArray();
  }

  virtual visualization_msgs::MarkerArray MakeControlInputDisplayRep(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& configuration,
      const Eigen::VectorXd& control_input,
      const std_msgs::ColorRGBA& color,
      const int32_t starting_index,
      const std::string& control_input_marker_ns) const
  {
    UNUSED(immutable_robot);
    UNUSED(configuration);
    UNUSED(control_input);
    UNUSED(color);
    UNUSED(starting_index);
    UNUSED(control_input_marker_ns);
    return visualization_msgs::MarkerArray();
  }

  virtual Eigen::Vector4d Get3dPointForConfig(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& config) const
  {
    UNUSED(immutable_robot);
    UNUSED(config);
    return Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  }

  virtual bool CheckConfigCollision(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& config,
      const double inflation_ratio=0.0) const
  {
    UNUSED(immutable_robot);
    UNUSED(config);
    UNUSED(inflation_ratio);
    return false;
  }

  virtual SimulationResult<State> ForwardSimulateMutableRobot(
      const TaskStateRobotBasePtr& mutable_robot,
      const State& target_position,
      const bool allow_contacts,
      StateStepTrace& trace,
      const bool enable_tracing,
      const DisplayFn& display_fn)
  {
    UNUSED(mutable_robot);
    UNUSED(target_position);
    UNUSED(allow_contacts);
    UNUSED(trace);
    UNUSED(enable_tracing);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }

  virtual SimulationResult<State> ReverseSimulateMutableRobot(
      const TaskStateRobotBasePtr& mutable_robot,
      const State& target_position,
      const bool allow_contacts,
      StateStepTrace& trace,
      const bool enable_tracing,
      const DisplayFn& display_fn)
  {
    UNUSED(mutable_robot);
    UNUSED(target_position);
    UNUSED(allow_contacts);
    UNUSED(trace);
    UNUSED(enable_tracing);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }

  virtual SimulationResult<State> ForwardSimulateRobot(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& start_position,
      const State& target_position,
      const bool allow_contacts,
      StateStepTrace& trace,
      const bool enable_tracing,
      const DisplayFn& display_fn)
  {
    UNUSED(immutable_robot);
    UNUSED(start_position);
    UNUSED(target_position);
    UNUSED(allow_contacts);
    UNUSED(trace);
    UNUSED(enable_tracing);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }

  virtual SimulationResult<State> ReverseSimulateRobot(
      const TaskStateRobotBasePtr& immutable_robot,
      const State& start_position,
      const State& target_position,
      const bool allow_contacts,
      StateStepTrace& trace,
      const bool enable_tracing,
      const DisplayFn& display_fn)
  {
    UNUSED(immutable_robot);
    UNUSED(start_position);
    UNUSED(target_position);
    UNUSED(allow_contacts);
    UNUSED(trace);
    UNUSED(enable_tracing);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }

  virtual std::vector<SimulationResult<State>> ForwardSimulateRobots(
      const TaskStateRobotBasePtr& immutable_robot,
      const std::vector<State, StateAlloc>& start_positions,
      const std::vector<State, StateAlloc>& target_positions,
      const bool allow_contacts,
      const DisplayFn& display_fn)
  {
    UNUSED(immutable_robot);
    UNUSED(start_positions);
    UNUSED(target_positions);
    UNUSED(allow_contacts);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }

  virtual std::vector<SimulationResult<State>> ReverseSimulateRobots(
      const TaskStateRobotBasePtr& immutable_robot,
      const std::vector<State, StateAlloc>& start_positions,
      const std::vector<State, StateAlloc>& target_positions,
      const bool allow_contacts,
      const DisplayFn& display_fn)
  {
    UNUSED(immutable_robot);
    UNUSED(start_positions);
    UNUSED(target_positions);
    UNUSED(allow_contacts);
    UNUSED(display_fn);
    throw std::runtime_error("Not a valid operation on TaskPlannerAdapter");
  }
};
}  // namespace task_planner_adapter
}  // namespace uncertainty_planning_core
