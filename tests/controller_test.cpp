#include <gtest/gtest.h>
#include <pcalib/controller.h>

namespace pcalib
{
namespace testing
{

TEST(Controller, Construtor)
{
  const double max_double = std::numeric_limits<double>::max();

  Controller controller;
  ASSERT_DOUBLE_EQ(0.0, controller.integral());
  ASSERT_DOUBLE_EQ(0.0, controller.setpoint());
  ASSERT_DOUBLE_EQ(0.0, controller.proportional_gain());
  ASSERT_DOUBLE_EQ(0.0, controller.integral_gain());
  ASSERT_DOUBLE_EQ(-max_double, controller.lower_bound());
  ASSERT_DOUBLE_EQ(+max_double, controller.upper_bound());
  ASSERT_EQ(0, controller.integral_delay());
  ASSERT_EQ(0, controller.iteration());
}

TEST(Controller, Update)
{
  double error;
  double update;
  double feedback;
  double integral;
  const double Kp = 0.172;
  const double Ki = 0.051;
  Controller controller;
  controller.set_proportional_gain(Kp);
  controller.set_integral_gain(Ki);

  controller.set_integral_delay(0);
  controller.set_setpoint(0.5);
  integral = 0;

  feedback = 0.25;
  error = controller.setpoint() - feedback;
  integral += error;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(1, controller.iteration());

  feedback = 0.45;
  error = controller.setpoint() - feedback;
  integral += error;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(2, controller.iteration());

  controller.Reset();
  controller.set_integral_delay(1);
  controller.set_setpoint(0.65);
  integral = 0;

  feedback = 0.25;
  error = controller.setpoint() - feedback;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(1, controller.iteration());

  feedback = 0.45;
  error = controller.setpoint() - feedback;
  integral += error;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(2, controller.iteration());

  controller.Reset();
  controller.set_integral_delay(2);
  controller.set_setpoint(0.13);
  integral = 0;

  feedback = 0.25;
  error = controller.setpoint() - feedback;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(1, controller.iteration());

  feedback = 0.20;
  error = controller.setpoint() - feedback;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(2, controller.iteration());

  feedback = 0.10;
  error = controller.setpoint() - feedback;
  integral += error;
  update = controller.Update(feedback, 100);
  ASSERT_DOUBLE_EQ(100 + error * Kp + integral * Ki, update);
  ASSERT_DOUBLE_EQ(integral, controller.integral());
  ASSERT_EQ(3, controller.iteration());
}

} // namespace testing

} // namespace pcalib