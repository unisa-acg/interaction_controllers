/* -------------------------------------------------------------------
 * 
 * This module has been developed as part of a collaboration between
 * the Automatic Control Group @ UNISA and ALTEC.
 *
 * Title:   test_workspace_trajectory.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Mar 18, 2021
 *
 * Test fixture for the WorkspaceTrajectory class.
 * 
 * -------------------------------------------------------------------
*/

// Tested class(es)
#include <workspace_trajectory/workspace_trajectory.h>

// Gtest
#include <gtest/gtest.h>

// * Test suite for class WorkspaceTrajectory
class WorkspaceTrajectoryTest : public ::testing::Test
{
  protected:
    // Per-test-suite set-up: called before the first test in the test suite.
    static void SetUpTestCase()
    {

    }

    // Constructor: called before each test in the test case.
    WorkspaceTrajectoryTest():
        workspace_trajectory_(nullptr)
    {
        std::string name("two_points_trajectory");
        std::vector<double> time = {0, 0.1};
        std::vector<double> x = {0.5, 0.5};
        std::vector<double> y = {0.25, 0.24};
        std::vector<double> z = {0.4, 0.4};
        std::vector<double> roll = {-M_PI, M_PI - 0.1};
        std::vector<double> pitch = {0, 0};
        std::vector<double> yaw = {0, 0.1};

        workspace_trajectory_ = std::make_shared<workspace_trajectory::WorkspaceTrajectory>(name, time, x, y, z, roll, pitch, yaw);
    }

    // Destructor: called after each test in the test case.
    ~WorkspaceTrajectoryTest()
    {
        
    }
    
    // Per-test-suite tear-down: called after the last test in the test suite.
    static void TearDownTestCase()
    {

    }

    std::shared_ptr<const workspace_trajectory::WorkspaceTrajectory> workspace_trajectory_;
};

// * Tests
// Test 1
/**
 * @brief verifies that method getTangentVectorRPYAtWaypoint returns the expected result
 */
TEST_F(WorkspaceTrajectoryTest, ComputesExpectedTangentVectorRPY)
{
    Eigen::VectorXd t = workspace_trajectory_->getTangentVectorRPYAtWaypoint(0);

    double tolerance =  1e-4;

    EXPECT_NEAR(t(0), 0, tolerance);
    EXPECT_NEAR(t(1), -0.0705, tolerance);
    EXPECT_NEAR(t(2), 0, tolerance);
    EXPECT_NEAR(t(3), -0.7053, tolerance);
    EXPECT_NEAR(t(4), 0, tolerance);
    EXPECT_NEAR(t(5), 0.7053, tolerance);
}

// Test 2
/**
 * @brief verifies that method getTangentVectorAtWaypoint returns the expected result
 */
TEST_F(WorkspaceTrajectoryTest, ComputesExpectedTangentVector)
{
    Eigen::VectorXd t = workspace_trajectory_->getTangentVectorAtWaypoint(0);

    double tolerance =  1e-4;

    EXPECT_NEAR(t(0), 0, tolerance);
    EXPECT_NEAR(t(1), -0.0705, tolerance);
    EXPECT_NEAR(t(2), 0, tolerance);
    EXPECT_NEAR(t(3), -0.7049, tolerance);
    EXPECT_NEAR(t(4), -0.0353, tolerance);
    EXPECT_NEAR(t(5), 0.7049, tolerance);
}

// * Run all the tests that were declared with TEST().
int main(int argc, char **argv)
{
    // Initialization
    testing::InitGoogleTest(&argc, argv);

    // Run all previously defined tests
    auto res = RUN_ALL_TESTS();

    return res;
}