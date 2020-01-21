#include <gtest/gtest.h>
#include <walker/walker.h>

namespace {

class TestWalker: public ::testing::Test {


protected:

     TestWalker()
     {




     }

     virtual ~TestWalker() {
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

};

TEST_F(TestWalker, checkCompute)
{


}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
