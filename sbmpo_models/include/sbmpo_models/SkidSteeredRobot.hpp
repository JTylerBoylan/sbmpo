#ifndef SBMPO_SKID_STEERED_ROBOT_MODEL_HPP
#define SBMPO_SKID_STEERED_ROBOT_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class SkidSteeredRobotModel : public Model {

    public:

    enum States {X, Y, Q};
    enum Controls {dQdt};

    SkidSteeredRobotModel() {
        goal_threshold_ = 0.25f;
        integration_steps_ = 5;
        forward_velocity_ = 0.2f;
        motor_efficiency_ = 0.76f;
        gear_ratio_ = 49.8f;
        torque_constant_ = 0.023f;
        electrical_resistance_ = 0.74f;
        vehicle_width_ = 0.39f;
        wheel_radius_ = 0.1075f;
        expansion_factor_ = 1.39f;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;
        float time_increment = time_span / integration_steps_;
        for (int i = 0; i < integration_steps_; i++) {
            next_state[X] += cosf(next_state[Q]) * forward_velocity_ * time_increment;
            next_state[Y] += sinf(next_state[Q]) * forward_velocity_ * time_increment;
            next_state[Q] += control[dQdt] * time_increment;
        }

        // Angle wrap
        while (next_state[Q] >= M_2PI)  next_state[Q] -= M_2PI;
        while (next_state[Q] < 0)       next_state[Q] += M_2PI;

        return next_state;
    }

    // Get the cost of a control
    virtual float cost(const State &state, const Control& control, const float time_span) {
        
        // Calculate omegas
        float K_exp = 0.5f * expansion_factor_ * vehicle_width_;
        float omega_l = (forward_velocity_ - K_exp * control[dQdt]) / wheel_radius_;
        float omega_r = (forward_velocity_ + K_exp * control[dQdt]) / wheel_radius_;

        // Calculate torques
        float turn_radius = forward_velocity_ / control[dQdt];
        float tau_l = torque_lookup_left_(turn_radius);
        float tau_r = torque_lookup_right_(turn_radius);

        // Calculate power
        float Km_d = 1.0f / (torque_constant_ * gear_ratio_ * motor_efficiency_);
        float P_l = tau_l*omega_l/motor_efficiency_ + (tau_l*Km_d)*(tau_l*Km_d)*electrical_resistance_;
        float P_r = tau_r*omega_r/motor_efficiency_ + (tau_r*Km_d)*(tau_r*Km_d)*electrical_resistance_;

        // Return energy cost
        float P_total = (P_l >= 0 ? P_l : 0) + (P_r >= 0 ? P_r : 0);
        return P_total * time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {
        float dx = goal[X] - state[X];
        float dy = goal[Y] - state[Y];
        return P_infinity_() * sqrtf(dx*dx + dy*dy) / forward_velocity_;
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return heuristic(state, goal) <= goal_threshold_;
    }

    virtual ~SkidSteeredRobotModel() {}

    /// @brief Set the goal threshold value
    /// @param goal_threshold Value to set as goal threshold
    void set_goal_threshold(float goal_threshold) {
        goal_threshold_ = goal_threshold;
    }

    /// @brief Set the number of integration steps (Euler)
    /// @param integration_steps Number of integration steps
    void set_integration_steps(int integration_steps) {
        integration_steps_ = integration_steps;
    }

    /// @brief Set the forward velocity of the robot
    /// @param forward_velocity Forward velocity value
    void set_forward_velocity(int forward_velocity) {
        forward_velocity_ = forward_velocity;
    }

    /// @brief Set the motor efficiency of the robot
    /// @param motor_efficiency Motor efficiency value
    void set_motor_efficiency(int motor_efficiency) {
        motor_efficiency_ = motor_efficiency;
    }

    /// @brief Set the gear ratio of the robot
    /// @param gear_ratio Gear ratio value
    void set_gear_ratio(int gear_ratio) {
        gear_ratio_ = gear_ratio;
    }

    /// @brief Set the torque constant of the robot
    /// @param torque_constant Torque constant value
    void set_torque_constant(int torque_constant) {
        torque_constant_ = torque_constant;
    }

    /// @brief Set the electrical resistance of the robot
    /// @param electrical_resistance Electrical resistance value
    void set_electrical_resistance(int electrical_resistance) {
        electrical_resistance_ = electrical_resistance;
    }

    /// @brief Set the width of the vehicle
    /// @param vehicle_width Vehicle width value
    void set_vehicle_width(int vehicle_width) {
        vehicle_width_ = vehicle_width;
    }

    /// @brief Set the expansion factor of the robot
    /// @param expansion_factor Expansion_factor value
    void set_expansion_factor(int expansion_factor) {
        expansion_factor_ = expansion_factor;
    }

    /// @brief Set the wheel radius of the robot
    /// @param wheel_radius Wheel radius value
    void set_wheel_radius(int wheel_radius) {
        wheel_radius_ = wheel_radius;
    }

    protected:

    float goal_threshold_;
    int integration_steps_;

    float forward_velocity_;
    float motor_efficiency_;
    float gear_ratio_;
    float torque_constant_;
    float electrical_resistance_;

    float vehicle_width_;
    float wheel_radius_;
    float expansion_factor_;

    virtual float P_infinity_() {
        /*
            LOOKUP VALUE
        */
        return 0.5;
    }

    virtual float torque_lookup_left_(float turn_radius) {
        /*
            LOOKUP TABLE
        */
       return 0.0f;
    }

    virtual float torque_lookup_right_(float turn_radius) {
        /*
            LOOKUP TABLE
        */
       return 0.0f;
    }

};

}

#endif