#include <gtest/gtest.h>
#include <engine/sagittal_plane.h>

namespace {

class TestSagittal: public ::testing::Test {


protected:

     TestSagittal(){

         double dt = 0.01;
         sag_plane = std::make_shared<SagittalPlane>(dt);

     }

     virtual ~TestSagittal() {
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

     double delta_com_expected;
     double Delta_com_expected;
     double Delta_foot_expected;
};


TEST_F(TestSagittal, checkUpdate)
{

    q = 0.;
    q_min = 0.;
    q_max = 0.;
    height_com = 1.;

    delta_com_expected = 0.;
    Delta_com_expected = 0.;
    Delta_foot_expected = 0.;

    sag_plane->update(q, q_min, q_max, height_com);

    ASSERT_TRUE(sag_plane->getDeltaCom() - delta_com_expected <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaComTot() - Delta_com_expected <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaFootTot() - Delta_foot_expected <= 0.001);
}

TEST_F(TestSagittal, checkUpdateAdvanced)
{
    q = 0.;
    q_min = 0.;
    q_max = 0.785398;
    height_com = 1.;

    delta_com_expected = 0;
    Delta_com_expected = fabs(height_com) * tan(q_max - q_min);
    Delta_foot_expected = 2 * Delta_com_expected;

    sag_plane->update(q, q_min, q_max, height_com);

    ASSERT_TRUE(sag_plane->getDeltaCom() - delta_com_expected <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaComTot() - Delta_com_expected <= 0.001);
    ASSERT_TRUE(sag_plane->getDeltaFootTot() - Delta_foot_expected <= 0.001);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
