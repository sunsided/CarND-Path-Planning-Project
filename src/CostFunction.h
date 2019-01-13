#ifndef PATH_PLANNING_COSTFUNCTION_H
#define PATH_PLANNING_COSTFUNCTION_H

#include "EnvContext.h"
#include "LaneInfo.h"

/**
 * A cost function to evaluate a lane.
 */
struct CostFunction {
public:
    explicit CostFunction(double weight) noexcept
        : weight{weight}
    {}

    /**
     * Evaluates the specified lane in the given context.
     * @param lane The lane to evaluate.
     * @param context The environment.
     * @return A cost ranging between 0 (no cost) and 1 (highest cost).
     */
    virtual double operator()(const LaneInfo& lane, const EnvContext& context) const = 0;

protected:
    /**
     * Adjusts the calculated score according to the weight.
     * @param cost The cost
     * @return The weight.
     */
    inline double scale(double cost, const EnvContext& context) const {
        if (cost >= context.highest_cost) return context.highest_cost;
        if (cost <= context.no_cost) return context.no_cost;
        return cost * weight;
    }

private:
    const double weight;
};

struct LaneCostEvaluation final : public CostFunction {
public:
    explicit LaneCostEvaluation() noexcept;

    /**
     * Evaluates the specified lane in the given context.
     * @param lane The lane to evaluate.
     * @param context The environment.
     * @return A cost ranging between 0 (no cost) and 1 (highest cost).
     */
    double operator()(const LaneInfo& lane, const EnvContext& context) const final;

private:
    /**
     * Registers the specified cost function.
     * @tparam T The cost function to register.
     */
    template<class T>
    void track_cost(double weight = 1.0) {
        static_assert(std::is_base_of<CostFunction, T>::value, "T must derive from CostFunction");
        _functions.push_back(std::unique_ptr<T>(new T(weight)));
    }

private:
    std::vector<std::unique_ptr<CostFunction>> _functions;
};

#endif //PATH_PLANNING_COSTFUNCTION_H
