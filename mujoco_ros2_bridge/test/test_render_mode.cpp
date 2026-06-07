#include <stdexcept>
#include <fstream>

#include <gtest/gtest.h>

#include "mujoco_ros2_bridge/mujoco_simulation.hpp"

TEST(RenderModeTest, ParsesSupportedValues)
{
  EXPECT_EQ(mujoco_ros2_bridge::RenderMode::Headless, mujoco_ros2_bridge::parse_render_mode("headless"));
  EXPECT_EQ(mujoco_ros2_bridge::RenderMode::Viewer, mujoco_ros2_bridge::parse_render_mode("viewer"));
}

TEST(RenderModeTest, RejectsUnsupportedValues)
{
  EXPECT_THROW(mujoco_ros2_bridge::parse_render_mode("bad"), std::invalid_argument);
}

TEST(MuJoCoSimulationTest, LoadsStepsAndResetsMinimalMjcf)
{
  const std::string model_path = "/tmp/mujoco_ros2_bridge_minimal.xml";
  std::ofstream model_file(model_path);
  model_file
    << "<mujoco model='minimal'>"
    << "  <option timestep='0.001'/>"
    << "  <worldbody>"
    << "    <body name='body' pos='0 0 0.1'>"
    << "      <joint name='joint' type='hinge' axis='0 0 1'/>"
    << "      <geom name='geom' type='sphere' size='0.05' mass='1'/>"
    << "    </body>"
    << "  </worldbody>"
    << "</mujoco>";
  model_file.close();

  mujoco_ros2_bridge::SimulationConfig config;
  config.model_path = model_path;
  config.publish_clock = false;

  mujoco_ros2_bridge::MuJoCoSimulation engine(nullptr);
  std::string error;
  ASSERT_TRUE(engine.initialize(config, &error)) << error;
  EXPECT_TRUE(engine.step(1, &error)) << error;
  EXPECT_EQ(engine.step_count(), 1u);
  EXPECT_TRUE(engine.reset("", &error)) << error;
  EXPECT_EQ(engine.step_count(), 0u);
}
