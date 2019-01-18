#include <OpenMpC/solver/UnconstrainedMpc.h>
#include <iostream>
#include <XBotInterface/MatLogger.hpp>

// Eigen::VectorXd zmp_traj(Eigen::VectorXd times)
// {
//     int m = 1;
//     int N = times.size();
//     Eigen::VectorXd zmp_traj(N);
//     
//     for (int i = 0; i<N; i++)
//     {
//         zmp_traj(i) = cos(m*3.14*times(i)) >0;
//     }
//     return zmp_traj;
// }



void lSpline(Eigen::VectorXd times, Eigen::VectorXd y, double dt, Eigen::VectorXd &times_vec, Eigen::VectorXd &Y)
{

    dt = 1.0/100;
    std::cout << "dt: " << dt << std::endl;
    int n = times.size();
    std::cout << "n: " << n << std::endl;
    double N = 0;
    
    
    Eigen::VectorXi N_chunks(n-1);
    
    for(int i=0; i<n-1; i++)
    { 
            N = N + (times.coeff(i+1)-times.coeff(i))/dt;
            N_chunks(i) = round((times.coeff(i+1)-times.coeff(i))/dt);
            
    }
    
//     std::cout << "N: " << N << std::endl;
//     std::cout << "N_progress: " << N_chunks.transpose() << std::endl;
    
    
    Y.resize(N,1);
    
    times_vec.setLinSpaced(N, dt, dt*N);

    int idx = 0;
    for(int i=0; i<n-1; i++)
    {
        Eigen::VectorXd temp_vec(N_chunks(i));
        temp_vec.setLinSpaced(N_chunks(i), y.coeff(i), y.coeff(i+1));
        //TODO fix make it start from 0
        Y.segment(idx, temp_vec.size()) = temp_vec;
        idx += temp_vec.size();    
    }
}

void generate_zmp(double y_start, double t_start, double dt, Eigen::VectorXd &zmp_t, Eigen::VectorXd &zmp_y)
    {

        double step_duration = 0.3;
        int max_steps = 10;
    
    
        int num_points = 2 * max_steps;
        
        double t_end = t_start + step_duration*num_points;
        //TODO qui potrei mettere anche un t_windows_end
        
        std::cout << "t_end: " << t_end << std::endl;
        Eigen::VectorXd y, times;
        

        y.resize(num_points*2,1);
        times.resize(num_points*2,1);
            

        int myswitch = 1;
        int j = 0;
        //without double stance generator
        for (int i = 0; i<num_points; i++)
        { 
            times(j) = t_start + step_duration* i;    
            times(j+1) = t_start + step_duration* (i+1);
            
            y(j) = myswitch * y_start;
            y(j+1) = myswitch * y_start;
            myswitch = -1 * myswitch;
            j = j+2;
        }
        
    
        Eigen::VectorXd y_tot(y.size() + 3);
        Eigen::VectorXd times_tot(times.size() + 3);
        
        y_tot << 0, 0, y, 0;
        times_tot << 0, t_start, times, t_end;

        
        lSpline(times_tot, y_tot, dt, zmp_t, zmp_y);
    }

void zmp_traj(double window_start, double window_end, Eigen::VectorXd &zmp_window_t, Eigen::VectorXd &zmp_window_y)
    {
        double dt = 1.0/100;
        
        if (window_start == 0)
        {
            window_start = dt;
        } //TODO bad add also negative
        
        Eigen::VectorXd zmp_t;
        Eigen::VectorXd zmp_y;
        
        double first_step = -0.3;
        double start_walk = 2;
        
      
        generate_zmp(first_step, start_walk, dt, zmp_t, zmp_y);

        
        int window_size = round((window_end - window_start)/dt) + 1;
        
        std::cout << "(window_end - window_start)/dt: " << (window_end - window_start)/dt << std::endl;
        std::cout << "window_start: " << window_start << std::endl;
        std::cout << "window_end: " << window_end << std::endl;

        zmp_window_t.setLinSpaced(window_size, window_start, window_end);
        
        int i = 0;
        while (i < zmp_t.size() && zmp_t[i] < window_start) //
        {
            i++;
        }
//         
        int j = 0;
        while (j < zmp_t.size() && zmp_t[j] < window_end) // 
        {
            j++;
        }
        
//        
        zmp_window_y.resize(window_size,1);
   
// 
        zmp_window_y.setZero();
        zmp_window_y.segment(0, j-i) = zmp_y.segment(i, j-i);
        
        std::cout << "window_size: " << window_size << std::endl;
        std::cout << "zmp_window_y_size: " << zmp_window_y.size() << std::endl;
//         
        
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
    
    double T = 2;
    
    int N = 2/0.01 +1;
    
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
    x << 0, 0, 0;
    
//     Eigen::VectorXd times;
//     times.setLinSpaced(N, Ts, Ts*N);
//     
    double dt = 0.01;
//     
//     double step_width =  0.3;
    
    
    Eigen::VectorXd zmp_window_t;
    Eigen::VectorXd zmp_window_y;
    

    
    
    for (int i =1; i<2000; i++)
    {
        Eigen::VectorXd u(1);
        
    zmp_traj(dt*i, T+dt*i, zmp_window_t, zmp_window_y);
//         Eigen::VectorXd zmp_ref = step_width*zmp_traj(times.array() + dt*i);
         
        u = K_fb * x + K_prev * zmp_window_y;
        
        integrator->integrate(x, u, dt, x);
        
        _logger->add("com", x);
        _logger->add("u", u);
        _logger->add("zmp", C_zmp*x);
        _logger->add("zmp_ref", zmp_window_y.coeff(0));
        
        
    }
    
    _logger->flush();
   
}
