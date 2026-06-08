#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>

#include "mujoco_simulation/mujoco_simulation.hpp"

TEST(RenderModeTest, ParsesSupportedValues) {
  EXPECT_EQ(mujoco_simulation::RenderMode::Headless,
            mujoco_simulation::parse_render_mode("headless"));
  EXPECT_EQ(mujoco_simulation::RenderMode::Viewer, mujoco_simulation::parse_render_mode("viewer"));
}

TEST(RenderModeTest, RejectsUnsupportedValues) {
  EXPECT_THROW(mujoco_simulation::parse_render_mode("bad"), std::invalid_argument);
}

TEST(MuJoCoSimulationTest, LoadsStepsAndResetsMinimalMjcf) {
  const std::string model_path = "/tmp/mujoco_simulation_minimal.xml";
  std::ofstream model_file(model_path);
  model_file << "<mujoco model='minimal'>"
             << "  <option timestep='0.001'/>"
             << "  <worldbody>"
             << "    <body name='body' pos='0 0 0.1'>"
             << "      <joint name='joint' type='hinge' axis='0 0 1'/>"
             << "      <geom name='geom' type='sphere' size='0.05' mass='1'/>"
             << "    </body>"
             << "  </worldbody>"
             << "</mujoco>";
  model_file.close();

  mujoco_simulation::SimulationConfig config;
  config.model_path = model_path;

  mujoco_simulation::MuJoCoSimulation engine;
  std::string error;
  ASSERT_TRUE(engine.initialize(config, &error)) << error;
  EXPECT_TRUE(engine.step(1, &error)) << error;
  EXPECT_EQ(engine.step_count(), 1u);
  EXPECT_TRUE(engine.reset("", &error)) << error;
  EXPECT_EQ(engine.step_count(), 0u);
}
