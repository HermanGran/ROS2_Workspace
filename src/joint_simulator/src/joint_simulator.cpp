#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

class joint_simulator {
private:
    double K_;
    double T_;
    double angle_;
    double angular_velocity_;
    double voltage_;
    double noise_;

public:
    joint_simulator(double K, double T);
    ~joint_simulator();
    void update(double dt);
    void setVoltage();
    void getAngle();
};

joint_simulator::joint_simulator(double K, double T) : K_(K), T_(T), angle_(0.0), angular_velocity_(0.0), voltage_(0.0), noise_(0)
{
}

joint_simulator::~joint_simulator()
{
}


// Oppdaterer angle
void joint_simulator::update(double dt) {
    // Oppdaterer "velocity" (omega) vha. førsteordens filter:
    // d(velocity)/dt = -(1/T_) * velocity_ + (K_/T_) * voltage_

    double dvel = dt * ( - (1.0 / T_) * angular_velocity_ + (K_ / T_) * voltage_ );
    angular_velocity_ += dvel;

    // Integrerer velocity -> angle
    angle_ += angular_velocity_ * dt;

}




class joint_simulator_node
{
private:
    /* data */
public:
    joint_simulator_node(/* args */);
    ~joint_simulator_node();
};

joint_simulator_node::joint_simulator_node(/* args */)
{
}

joint_simulator_node::~joint_simulator_node()
{
}