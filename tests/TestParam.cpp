#include <gtest/gtest.h>
#include <param/param.h>

namespace {

class TestParam: public ::testing::Test {


protected:

     TestParam()
     {

         //#ifndef WALKER_TEST_CONFIG_PATH
         //         throw std::runtime_error("WALKER_TEST_CONFIG_PATH is not defined");
         //#endif

//         std::string path_to_cfg(WALKER_TEST_CONFIG_PATH);
         std::string path_to_cfg("/home/francesco/advr-superbuild/external/limit_cycle_walking/param/");
         YAML::Node walking_yaml = YAML::LoadFile(path_to_cfg + "walking_param.yaml");

         Walker::Param param(walking_yaml);


     }

     virtual ~TestParam() {
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

};

TEST_F(TestParam, checkGetter)
{


}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
