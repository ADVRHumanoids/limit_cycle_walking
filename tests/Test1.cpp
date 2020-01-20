#include <gtest/gtest.h>
#include <walker/sagittal_plane.h>

namespace {

//Eigen::Affine3d GetRandomFrame()
//{
//    Eigen::Quaterniond q;
//    q.coeffs().setRandom();
//    q.normalize();

//    Eigen::Affine3d T;
//    T.setIdentity();
//    T.linear() = q.toRotationMatrix();
//    T.translation().setRandom();

//    return T;
//}

class TestApi: public ::testing::Test {


protected:

     TestApi(){


         double dt = 0.01;
         sag_plane = std::make_shared<SagittalPlane>(dt);



     }

     virtual ~TestApi() {
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

     SagittalPlane::Ptr sag_plane;

};


TEST_F(TestApi, checkBoh)
{
    double q = 0;
    double q_min = 0;
    double q_max = 0;
    double height_com = 1;
    sag_plane->update(q, q_min, q_max, height_com);

    double delta_com = sag_plane->getDeltaCom();
    double Delta_com = sag_plane->getDeltaComTot();
    double Delta_foot = sag_plane->getDeltaFootTot();

    ASSERT_TRUE(delta_com - 0 <= 0.001);
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
