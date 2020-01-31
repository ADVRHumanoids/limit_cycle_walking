#include <gtest/gtest.h>
#include <walker/walker.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace {

class TestWalker: public ::testing::Test {


protected:

     TestWalker()
     {

         std::string path_to_cfg(WALKER_TEST_CONFIG_PATH);
         std::cout << __func__ << " using path " << path_to_cfg << std::endl;

         YAML::Node walking_yaml = YAML::LoadFile(path_to_cfg + "test_walker.yaml");
//         YAML::Node ik_yaml = YAML::LoadFile(path_to_cfg + "comanPlus_VC_stack.yaml");

         double dt = 0.01;

         /* initialize Walker */
         auto par = std::make_shared<Walker::Param>(walking_yaml);
         wlkr = std::make_shared<Walker>(dt, par);


         _initial_lowering_default = par->getInitialLowering();

//         /* initialize XBotCore */
//         XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::DEBUG);
//         std::string xbot_yaml = path_to_cfg + "cogimon_walking.yaml";
//         auto model = XBot::ModelInterface::getModel(xbot_yaml);

//         /* initialize CartesianInterface */
//         XBot::Cartesian::ProblemDescription ik_problem(ik_yaml, model);
//         ci = std::make_shared<XBot::Cartesian::CartesianInterfaceImpl>(model, ik_problem);

         _state.com_vel << 0, 0, 0;
         _state.ankle_T_com[0].translation() << 0.1, - 0.15, 1;
         _state.ankle_T_com[0].linear().setIdentity();

         _state.ankle_T_com[1].translation() << 0.1, 0.15, 1;
         _state.ankle_T_com[1].linear().setIdentity();

         _state.world_T_com << -5, 0, 1;

         _state.world_T_foot[0].translation() << -5, - 0.15, 0;
         _state.world_T_foot[0].linear().setIdentity();

         _state.world_T_foot[1].translation() << -5, 0.15, 0;
         _state.world_T_foot[1].linear().setIdentity();

         _state.world_T_waist.translation() << -5, 0, 0.5;

         _ref = _state;

         _state = static_cast<const mdof::RobotState>(_state);

     }

     virtual ~TestWalker() {
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

     Walker::Ptr wlkr;
     mdof::RobotState _state;
     mdof::RobotState _ref;

//     XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;

     double _initial_lowering_default;

};

TEST_F(TestWalker, checkHoming)
{
    wlkr->homing(_state, _ref);

    /* translation part */
    ASSERT_EQ(_ref.ankle_T_com[0].translation(), _state.ankle_T_com[0].translation());
    ASSERT_EQ(_ref.ankle_T_com[1].translation(), _state.ankle_T_com[1].translation());
    /* linear part */
    ASSERT_EQ(_ref.ankle_T_com[0].linear(), _state.ankle_T_com[0].linear());
    ASSERT_EQ(_ref.ankle_T_com[1].linear(), _state.ankle_T_com[1].linear());

    Eigen::Vector3d new_com(_state.world_T_com[0], _state.world_T_com[1], _state.world_T_com[2] + _initial_lowering_default);
    ASSERT_EQ(_ref.world_T_com, new_com);

    /* translation part */
    ASSERT_EQ(_ref.world_T_foot[0].translation(), _state.world_T_foot[0].translation());
    ASSERT_EQ(_ref.world_T_foot[1].translation(), _state.world_T_foot[1].translation());
    /* linear part */
    ASSERT_EQ(_ref.world_T_foot[0].linear(), _state.world_T_foot[0].linear());
    ASSERT_EQ(_ref.world_T_foot[1].linear(), _state.world_T_foot[1].linear());

    /* translation part */
    ASSERT_EQ(_ref.world_T_waist.translation(), _state.world_T_waist.translation());
    /* linear part */
    ASSERT_EQ(_ref.world_T_waist.linear(), _state.world_T_waist.linear());
}

TEST_F(TestWalker, checkInit)
{
        wlkr->init(_state);
}

TEST_F(TestWalker, checkUpdate)
{
    wlkr->homing(_state, _ref);
    wlkr->init(_state);

    double t = 0;
    wlkr->update(t, _state, _ref);


    /* translation part */
    ASSERT_EQ(_ref.ankle_T_com[0].translation(), _state.ankle_T_com[0].translation());
    ASSERT_EQ(_ref.ankle_T_com[1].translation(), _state.ankle_T_com[1].translation());
    /* linear part */
    ASSERT_EQ(_ref.ankle_T_com[0].linear(), _state.ankle_T_com[0].linear());
    ASSERT_EQ(_ref.ankle_T_com[1].linear(), _state.ankle_T_com[1].linear());

    Eigen::Vector3d new_com(_state.world_T_com[0], _state.world_T_com[1], _state.world_T_com[2] + _initial_lowering_default);
    ASSERT_EQ(_ref.world_T_com, new_com);

    /* translation part */
    ASSERT_EQ(_ref.world_T_foot[0].translation(), _state.world_T_foot[0].translation());
    ASSERT_EQ(_ref.world_T_foot[1].translation(), _state.world_T_foot[1].translation());
    /* linear part */
    ASSERT_EQ(_ref.world_T_foot[0].linear(), _state.world_T_foot[0].linear());
    ASSERT_EQ(_ref.world_T_foot[1].linear(), _state.world_T_foot[1].linear());

    /* translation part */
    ASSERT_EQ(_ref.world_T_waist.translation(), _state.world_T_waist.translation());
    /* linear part */
    ASSERT_EQ(_ref.world_T_waist.linear(), _state.world_T_waist.linear());

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
