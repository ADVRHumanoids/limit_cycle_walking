#include <OpenMpC/solver/UnconstrainedMpc.h>
#include <iostream>
#include <XBotInterface/MatLogger.hpp>

Eigen::VectorXd zmp_traj(Eigen::VectorXd times)
{
    int m = 1;
    int N = times.size();
    Eigen::VectorXd zmp_traj(N);
    
    for (int i = 0; i<N; i++)
    {
        zmp_traj(i) = cos(m*3.14*times(i)) >0;
    }
    return zmp_traj;
}


int main(int argc, char **argv)
{
    XBot::MatLogger::Ptr _logger;
    _logger = XBot::MatLogger::getLogger("/tmp/kajitaTest");
    
    auto integrator =  OpenMpC::dynamics::LtiDynamics::Integrator(3,1);
    
    
    Eigen::MatrixXd C_zmp(1,3);
    double h = 1;
    
    double w = sqrt(9.8/h);
    
    C_zmp<<1 ,0, -1.0/std::pow(w, 2.0);
    integrator->addOutput("zmp", C_zmp);
    
    double Ts = 0.01;
    int N = 500;
    
    OpenMpC::UnconstrainedMpc lqr(integrator, Ts, N);
    
    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    
    Q << 1000000;
    R << 1;
    
    lqr.addInputTask(R);
    lqr.addOutputTask("zmp", Q);
    
    lqr.compute();
    
    auto K_fb = lqr.getStateFeedbackGain();
    auto K_prev = lqr.getOutputFeedforwardGainPreview("zmp");
    
    
    std::cout << "K_fb: " << K_fb << std::endl;
    std::cout << "K_prev: " << K_prev << std::endl;
   
    
    Eigen::Vector3d x; 
    x << 0.15, 0, 0;
    
    Eigen::VectorXd times;
    times.setLinSpaced(N, Ts, Ts*N);
    
    double dt = 0.01;
    
    double step_width =  0.3;
    
    for (int i =0; i<2000; i++)
    {
        Eigen::VectorXd u(1);
        
        Eigen::VectorXd zmp_ref = step_width*zmp_traj(times.array() + dt*i);
        u = K_fb * x + K_prev * zmp_ref;
        
        integrator->integrate(x, u, dt, x);
        
        _logger->add("com", x);
        _logger->add("u", u);
        _logger->add("zmp", C_zmp*x);
        _logger->add("zmp_ref", zmp_ref.coeff(0));
        
        
    }
    
    _logger->flush();
   
}
