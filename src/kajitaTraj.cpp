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
//     std::cout << "dt: " << dt << std::endl;
    int n = times.size();
//     std::cout << "n: " << n << std::endl;
    double N = 0;
    
    
    Eigen::VectorXi N_chunks(n-1);
    
    for(int i=0; i<n-1; i++)
    { 
        N = N + (times.coeff(i+1)-times.coeff(i))/dt;
        N_chunks(i) = round((times.coeff(i+1)-times.coeff(i))/dt);      
    }
    
//     std::cout << "N: " << N << std::endl;
//     std::cout << "N_progress: " << N_chunks.transpose() << std::endl;

    Y.resize(N+1,1);
    Y.setZero();
    
    times_vec.setLinSpaced(N+1, 0, dt*N);
    
//      std::cout << "times_vec: " << times_vec.transpose() << std::endl;

    int idx = 0;
    for(int i=0; i<n-1; i++)
    {
        Eigen::VectorXd temp_vec(N_chunks(i));
        temp_vec.setLinSpaced(N_chunks(i), y.coeff(i), y.coeff(i+1));
//         std::cout << "temp_vec: " << temp_vec.transpose() << std::endl;
        //TODO fix make it start from 0
        Y.segment(idx, temp_vec.size()) = temp_vec;
        idx += temp_vec.size();
//         std::cout << "idx: " << idx << std::endl;  
    }

    
//     std::cout << "Y: " << Y.transpose() << std::endl;
//     for (int i = 0; i < times_vec.size(); i++)
//     {
//         _logger->add("times", times_vec(i));
//         _logger->add("Y", Y(i));
//     }
}

void generate_zmp(double y_start, double t_start, double dt, Eigen::VectorXd &zmp_t, Eigen::VectorXd &zmp_y)
    {


        double step_duration = 0.3;
        int max_steps = 8;
        
        
        
        int num_points = 2 * max_steps;
        
        double t_end = t_start + step_duration*num_points;
        //TODO qui potrei mettere anche un t_windows_end
        
//         std::cout << "t_end: " << t_end << std::endl;
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
        
//         std::cout << "zmp_t_size: " << zmp_t.size() << std::endl;
//         std::cout << "zmp_y_size: " << zmp_y.size() << std::endl;
//         
//         std::cout << "zmp_t: " << zmp_t.transpose() << std::endl;
//         std::cout << "zmp_y: " << zmp_y.transpose() << std::endl;
        
    }

void zmp_traj(double window_start, double window_end, Eigen::VectorXd &zmp_window_t, Eigen::VectorXd &zmp_window_y)
    {
        double dt = 1.0/100;
        
        XBot::MatLogger::Ptr _logger;
        _logger = XBot::MatLogger::getLogger("/tmp/kajitaTest1");
        
        double first_step = -0.1;
        double start_walk = 1;
        
      
        Eigen::VectorXd zmp_t;
        Eigen::VectorXd zmp_y;
        
        generate_zmp(first_step, start_walk, dt, zmp_t, zmp_y);
        
        int window_size = round((window_end - window_start))/dt + 1;
        
        std::cout << "(window_end - window_start)/dt: " << (window_end - window_start)/dt << std::endl;
        std::cout << "window_start: " << window_start << std::endl;
        std::cout << "window_end: " << window_end << std::endl;
        std::cout << "zmp_y_size: " << zmp_y.size() << std::endl;
        
        int zmp_y_size = zmp_t.size();
        zmp_window_t.setLinSpaced(window_size, window_start, window_end);
        
        int i = 0;
        while (i < zmp_t.size() && zmp_t[i] < window_start)
        {
            i++;
        }
//         
        int j = 0;
        while (j < zmp_t.size() && zmp_t[j] < window_end)
        {
            j++;
        }
//        
        zmp_window_y.resize(window_size,1);
       
//         std::cout << "window_size: "<< window_size << std::endl;
        std::cout << "i: " << i << std::endl;
        std::cout << "j: " << j << std::endl;      
// 
        zmp_window_y.setZero();
        zmp_window_y.segment(0, j-i) = zmp_y.segment(i, j-i);
//         
//         if (window_start >= 3)
//         {
// //             std::cout << zmp_y.segment(i, j-i) << std::endl;
//                 zmp_y.conservativeResize(zmp_y_size+9,1);
//                 zmp_window_y.segment(0, j-i) = zmp_y.segment(i+9, j-i);
//         }
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
    
    C_zmp << 1 ,0, -1.0/std::pow(w, 2.0);
    integrator->addOutput("zmp", C_zmp);
    
    double Ts = 0.01; // dt window
    
    double T = 2; //length window in sec
    
    int N = T/0.01; //same as the integration time
    
    OpenMpC::UnconstrainedMpc lqr(integrator, Ts, N+1);
    
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
    
    std::cout << "K_fb_size: " << K_fb.size() << std::endl;
    std::cout << "K_prev_size: " << K_prev.size() << std::endl;
   
    
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
    double window_start;
    double window_end;
//     zmp_traj(0, T, zmp_window_t, zmp_window_y);
    
/*           for (int i = 0; i < zmp_window_t.size(); i++)
        {
            _logger->add("zmp_window_t", zmp_window_t(i));
            _logger->add("zmp_window_y", zmp_window_y(i));
        }   */ 
        
        
    for (int i =1; i<1000; i++)
    {
        Eigen::VectorXd u(1);
        
        window_start = dt*i;
        
        if (dt*i > 3)
        {
            window_start = dt*i + 0.1;
        }

        zmp_traj(window_start, T+window_start, zmp_window_t, zmp_window_y);
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
