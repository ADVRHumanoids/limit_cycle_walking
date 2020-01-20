#include <gtest/gtest.h>
#include <walker/sagittal_plane.h>

namespace {

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
     double q;
     double q_min;
     double q_max;
     double height_com;

     double delta_com;
     double Delta_com;
     double Delta_foot;

     double delta_com_true;
     double Delta_com_true;
     double Delta_foot_true;
};


TEST_F(TestApi, checkUpdate)
{

    q = 0.;
    q_min = 0.;
    q_max = 0.;
    height_com = 1.;

    delta_com_true = 0.;
    Delta_com_true = 0.;
    Delta_foot_true = 0.;

    sag_plane->update(q, q_min, q_max, height_com);

    ASSERT_TRUE(sag_plane->getDeltaCom() - delta_com_true <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaComTot() - Delta_com_true <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaFootTot() - Delta_foot_true <= 0.001);

    q = 0.;
    q_min = 0.;
    q_max = 0.785398;
    height_com = 1.;

    delta_com_true = 0;
    Delta_com_true = fabs(height_com) * tan(q_max - q_min);
    Delta_foot_true = 2 * Delta_com_true;

    sag_plane->update(q, q_min, q_max, height_com);

    ASSERT_TRUE(sag_plane->getDeltaCom() - delta_com_true <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaComTot() - Delta_com_true <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaFootTot() - Delta_foot_true <= 0.001);
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
