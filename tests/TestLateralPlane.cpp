#include <gtest/gtest.h>
#include <engine/lateral_plane.h>

namespace {

class TestLateral: public ::testing::Test {

protected:

    TestLateral()
    {
        opt.Q << 1000000;
        opt.R << 1;

        opt.h = 1.0;
        opt.Ts = 0.01;
        opt.horizon_duration = 1.0;

        dt = 0.01;
        lat_plane = std::make_shared<LateralPlane>(dt, opt);

    }

    virtual ~TestLateral() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    LateralPlane::Ptr lat_plane;

    double q_sns;
    double q_min;
    double q_max;
    double zmp_val_current;
    double zmp_val_next;
    double duration_step;
    double middle_zmp; /* should be constant */
    double offset;
    LateralPlane::Options opt;
    double dt;

};

class TestLateralNotInitialized: public ::testing::Test {

protected:

    TestLateralNotInitialized()
    {
        dt = 0.01;
        lat_plane = std::make_shared<LateralPlane>(dt);

        /* default options are:
            h -> 1.0,
            horizon_length -> 5,
            Ts -> 0.01,
            Q -> 1000000;
            R -> 1;
         */
    }

    virtual ~TestLateralNotInitialized() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    double q_sns;
    double q_min;
    double q_max;
    double zmp_val_current;
    double zmp_val_next;
    double duration_step;

    LateralPlane::Ptr lat_plane;
    double dt;
};

TEST_F(TestLateral, checkUpdate)
{
    q_sns = 0;
    q_min = 0;
    q_max = 0.785;
    zmp_val_current = 1;
    zmp_val_next = -1;
    duration_step = 2;
    middle_zmp = 0;
    offset = 0;

    int size_window = static_cast<int>(round(opt.horizon_duration/dt));
    Eigen::VectorXd preview_window_expected(size_window);
    preview_window_expected.setOnes();

    lat_plane->update(q_sns, q_min, q_max, zmp_val_current, zmp_val_next, duration_step, middle_zmp, offset);

    ASSERT_TRUE(lat_plane->getDeltaCom() - 0 <= 0.001);
    ASSERT_TRUE(lat_plane->getPreviewWindow().isApprox(preview_window_expected));
}


TEST_F(TestLateralNotInitialized, checkUpdate)
{
    q_sns = 0;
    q_min = 0;
    q_max = 0.785;
    zmp_val_current = 1;
    zmp_val_next = -1;
    duration_step = 2;

    double default_horizon_duration = 5.;
    int size_window = static_cast<int>(round(default_horizon_duration/dt));
    int size_step = static_cast<int>(round(duration_step/dt));
    Eigen::VectorXd preview_window_expected(size_window);
    preview_window_expected.setOnes(); /* first 200 are 1 */

    Eigen::VectorXd segment(size_step);
    preview_window_expected.segment(size_step, size_step) << zmp_val_next * segment.setOnes(); /* second 200 are -1 */
    /* last 100 are 1 */

    lat_plane->update(q_sns, q_min, q_max, zmp_val_current, zmp_val_next, duration_step);

    ASSERT_TRUE(lat_plane->getDeltaCom() - 0 <= 0.001);
    ASSERT_TRUE(lat_plane->getPreviewWindow().isApprox(preview_window_expected));
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
