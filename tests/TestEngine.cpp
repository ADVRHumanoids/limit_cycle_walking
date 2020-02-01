//#include <gtest/gtest.h>
//#include <engine/engine.h>

//namespace {

//class TestEngine: public ::testing::Test {


//protected:

//     TestEngine()
//     {
//         opt.zmp_offset = 0.;
//         opt.horizon_duration = 5.;
//         opt.mpc_Q = 1000000.;
//         opt.mpc_R = 1.;
//         opt.double_stance_duration = 0.;

//         dt = 0.01;

//         eng = std::make_shared<Engine>(dt, opt);

//         ss.q = 0;
//         ss.q_min = 0;
//         ss.q_max = 0.785;
//         ss.height_com = 1;
//         ss.zmp_val_current = 1;
//         ss.zmp_val_next = -1;
//         ss.step_duration = 2;
//         ss.step_clearance = 0.5;



//     }

//     virtual ~TestEngine() {
//     }

//     virtual void SetUp() {

//     }

//     virtual void TearDown() {
//     }

//     Engine::Ptr eng;
//     double dt;
//     Engine::Options opt;
//     mdof::StepState ss;

//};


//class TestEngineNotInitialized: public ::testing::Test {


//protected:

//     TestEngineNotInitialized()
//     {

//         /* default options:
//          * double zmp_offset = 0.;
//          * double horizon_duration = 5.;
//          *  double mpc_Q = 1000000.;
//          * double mpc_R = 1.;
//          * double double_stance_duration = 0.;
//          */

//         dt = 0.01;

//         eng = std::make_shared<Engine>(dt);

//         ss.q = 0;
//         ss.q_min = 0;
//         ss.q_max = 0.785;
//         ss.height_com = 1;
//         ss.zmp_val_current = 1;
//         ss.zmp_val_next = -1;
//         ss.step_duration = 2;
//         ss.step_clearance = 0.5;



//     }

//     virtual ~TestEngineNotInitialized() {
//     }

//     virtual void SetUp() {

//     }

//     virtual void TearDown() {
//     }

//     Engine::Ptr eng;
//     double dt;
//     Engine::Options opt;
//     mdof::StepState ss;

//};


//TEST_F(TestEngine, checkCompute)
//{
//    double time = 0.;

//    ASSERT_TRUE(eng->initialize(ss));

//    Eigen::Vector3d delta_com_tot;
//    Eigen::Vector3d delta_com, delta_com_expected;
//    Eigen::Vector3d delta_foot_tot, delta_foot_tot_expected;

//    eng->compute(time, ss, delta_com, delta_foot_tot, delta_com_tot);

//    double Delta_com_expected = fabs(ss.height_com) * tan(ss.q_max - ss.q_min);

//    delta_com_expected << 0., 0., 0.;
//    delta_foot_tot_expected << 2 * Delta_com_expected, 0., 0.;

////    ASSERT_TRUE(delta_com.isApprox(delta_com_expected));
//    ASSERT_TRUE((delta_com - delta_com_expected).norm() < 0.01);
//    ASSERT_TRUE(delta_foot_tot.isApprox(delta_foot_tot_expected));


//}


//TEST_F(TestEngineNotInitialized, checkCompute)
//{
//    double time = 0.;

//    ASSERT_TRUE(eng->initialize(ss));

//    Eigen::Vector3d delta_com_tot;
//    Eigen::Vector3d delta_com, delta_com_expected;
//    Eigen::Vector3d delta_foot_tot, delta_foot_tot_expected;

//    eng->compute(time, ss, delta_com, delta_foot_tot, delta_com_tot);

//    double Delta_com_expected = fabs(ss.height_com) * tan(ss.q_max - ss.q_min);

//    delta_com_expected << 0., 0., 0.;
//    delta_foot_tot_expected << 2 * Delta_com_expected, 0., 0.;

////    ASSERT_TRUE(delta_com.isApprox(delta_com_expected));
//    ASSERT_TRUE((delta_com - delta_com_expected).norm() < 0.01);
//    ASSERT_TRUE(delta_foot_tot.isApprox(delta_foot_tot_expected));


//}


//}

//int main(int argc, char **argv) {
//  ::testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}
