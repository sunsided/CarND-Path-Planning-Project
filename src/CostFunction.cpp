#include <memory>
#include "CostFunction.h"
#include "lanes.h"

static inline double square(double value) {
    return value * value;
}

static inline double lerp(const double t, const double t_min = 0.0, const double t_max = 1.0, const double min = 0.0, const double max = 1.0) {
    const auto n = (t - t_min) / t_max;
    return n * min + (1 - n) * max;
}

struct FrontClearanceTooLowCost final : public CostFunction {
    explicit FrontClearanceTooLowCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        const auto front_clearance = lane.front.at(0).clearance();
        if (std::isnan(front_clearance)) return context.no_cost;

        const auto horizon = 2.0 * context.safety_margin;
        if (front_clearance >= horizon) return context.no_cost;
        if (front_clearance <= context.safety_margin) return context.highest_cost;

        const auto cost = lerp(front_clearance, 0, horizon);
        // const auto cost = square(context.safety_margin/front_clearance);
        return scale(cost, context);
    }
};

struct RearClearanceTooLowCost final : public CostFunction {
    explicit RearClearanceTooLowCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        // There is never a cost to cars directly behind us.
        const int current_lane_idx = get_lane_number(context.car_d, context.lane_width);
        if (lane.lane_index == current_lane_idx) return context.no_cost;

        // If no cars are behind us, we're good.
        const auto rear_clearance = lane.rear.at(0).clearance();
        if (std::isnan(rear_clearance)) return context.no_cost;

        // If cars are away far enough, we're good.
        const auto horizon = context.rear_safety_margin;
        if (rear_clearance >= horizon) return context.no_cost;

        // The closer the car, the worse.
        // TODO: Speed matters as well, as faster cars are more trouble!
        const auto interpolated_cost = lerp(rear_clearance, 0, horizon, 0, 1);
        const auto cost = std::max(context.no_cost, interpolated_cost);

        return scale(cost, context);
    }
};

struct TrackedVehiclesInFrontCost final : public CostFunction {
    explicit TrackedVehiclesInFrontCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        const auto max_count = 4;
        const auto front_count = std::min(static_cast<size_t>(4), lane.front.size());
        if (front_count == 1) return context.no_cost;

        const auto cost = lerp(front_count, 0, max_count, 0, 0.99);
        return scale(cost, context);
    }
};

struct LaneVelocityCost final : public CostFunction {
    explicit LaneVelocityCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        const auto &front = lane.front.at(0);
        if (!front.valid()) return context.no_cost;

        const auto speed_delta = std::min(context.speed_limit_mph, context.speed_limit_mph - front.speed());
        if (speed_delta <= 0) return context.no_cost;

        return scale(lerp(speed_delta, 0, context.speed_limit_mph), context);
    }
};

struct BaseLaneCost final : public CostFunction {
    explicit BaseLaneCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        if (context.base_lane < 0) return context.no_cost;
        const auto lane_delta = std::abs(context.base_lane - lane.lane_index);

        static const auto one_third = 1.0 / 3.0;
        const auto delta_normalized = lane_delta * one_third;
        return scale(delta_normalized, context);
    }
};

struct SideClearanceTooLowCost final : public CostFunction {
    explicit SideClearanceTooLowCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        const auto front_clearance = lane.front.at(0).clearance();
        const auto rear_clearance = lane.rear.at(0).clearance();
        if (front_clearance <= context.min_clearance) return context.highest_cost;
        if (rear_clearance <= context.min_clearance) return context.highest_cost;
        return context.no_cost;
    }
};

struct SkipOneLaneCost final : public CostFunction {
    explicit SkipOneLaneCost(double weight) noexcept : CostFunction{weight} {}

    double operator()(const LaneInfo &lane, const EnvContext &context) const final {
        const int current_lane_idx = get_lane_number(context.car_d, context.lane_width);
        if (std::abs(current_lane_idx - lane.lane_index) > 1) return context.highest_cost;
        return context.no_cost;
    }
};

LaneCostEvaluation::LaneCostEvaluation() noexcept
    : CostFunction{1.0}
{
    // Cost functions that encourage or prevent the selection of a lane
    track_cost<FrontClearanceTooLowCost>();
    track_cost<RearClearanceTooLowCost>();
    track_cost<BaseLaneCost>(0.1);
    track_cost<LaneVelocityCost>(0.1);
    track_cost<TrackedVehiclesInFrontCost>(0.2);

    // Cost functions that prevent the change of a lane
    track_cost<SideClearanceTooLowCost>();
    track_cost<SkipOneLaneCost>();
    // TODO: Add cost function that takes into account the estimated position of a tracked vehicle in some seconds from now
}

double LaneCostEvaluation::operator()(const LaneInfo &lane, const EnvContext &context) const {
    double cost = context.no_cost;
    for (const auto& fun: _functions) {
        const auto local_cost = (*fun)(lane, context);
        cost = std::min(context.highest_cost, std::max(cost, local_cost));
    }

    if (cost <= context.score_threshold_lo) {
        cost = context.no_cost;
    }

    return cost;
}