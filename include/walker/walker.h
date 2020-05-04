#ifndef WALKER_H
#define WALKER_H


#include <robot/robot_state.h>
#include <engine/engine.h>
#include <walker/foot_trajectory.h>

class Walker {
public:

    typedef std::shared_ptr<Walker> Ptr;

    class Param;

    Walker(double dt, std::shared_ptr<Param> par = getDefaultParam()); /* TODO */

    enum class Event { SagReached = 0, Start = 2, Stop = 3, Empty = 4 };
    enum class State { Idle = 0, Walking = 1, Starting = 2, Stopping = 4, LastStep = 5 };
    enum class Stance { Double = 0, Single = 1 };

    bool init(const mdof::RobotState &state);

    bool start();
    bool stop();

    /* parametrization a la Bennewitz */
    bool setStep(double delta_x, double delta_y, double delta_theta);

    /* stride of the step */
    bool setQMax(std::vector<double> q_max);

    bool setDistFeet(std::vector<double> dist_feet);
    /* rotation (in rad) of foot and waist */
    bool setPhi(std::vector<double> phi);

    bool update(double time,
                const mdof::RobotState &state,
                mdof::RobotState &ref);

    bool homing(const mdof::RobotState &state,
                mdof::RobotState &ref);

    State getState() {return _current_state;}

    void log(std::string name, XBot::MatLogger2::Ptr logger);

    friend std::ostream& operator<<(std::ostream& os, Event s);
    friend std::ostream& operator<<(std::ostream& os, State s);

private:


    static std::shared_ptr<Walker::Param> getDefaultParam();

    bool step_machine(double time);

    bool qDetector(double q,
                   double q_min,
                   double q_max);

    bool impactDetector(double swing_leg_heigth,
                        double terrain_heigth);

    bool sagHandler(double time,
                    const mdof::RobotState &state);

    bool latHandler(double time,
                    const mdof::RobotState &state);

    bool updateQ(double time,
                  double q_sag,
                  double q_sag_min,
                  double q_sag_max,
                 double steep_q,
                 double& q);

    double computeQSag(bool current_swing_leg,
                       double theta,
                       Eigen::Vector3d world_T_com,
                       Eigen::Vector3d world_T_com_start,
                       std::array<Eigen::Affine3d, 2> ankle_T_com);

    bool updateQMax(double time);
    bool updateTheta(double time);
    bool updatePhi(double time);

    bool updateStep();
    bool updateZmp(mdof::RobotState state);
    //    bool resetter();

    State _current_state, _previous_state;
    Event _current_event, _previous_event;
    Stance _current_stance;

    /* parameters of the stepping motion */
    int _step_counter, _cycle_counter;
    double _steep_q, _steep_q_sag, _steep_q_lat;

    double _t_impact;

    double _theta, _phi, _dist_feet;
    double _theta_previous, _phi_prev;

    double _alpha_sag;

    double _time;
    double _new_event_time;

    /* received command to start walking */
    bool _started;

    bool _turning_step = false;
    bool _turning_step_prev = false;
    /* starting time */
    double _t_start_walk;

    /* phase variable */
    double _q, _q_min, _q_max;
    double _q_sag, _q_sag_min, _q_sag_max;
    double _q_lat, _q_lat_min, _q_lat_max;
    double _q_sag_max_previous, _q_max_previous;

    Eigen::Vector2d _com_disp;
    /* dt of the control */
    double _dt;

    double _zmp_right, _zmp_left;

    /* swinging leg (0 -> left, 1 -> right) */
    bool _current_swing_leg;

    /* buffer of commands for step */
    std::deque<double> _q_buffer, _theta_buffer, _phi_buffer, _dist_feet_buffer;

    double _terrain_height;

    Eigen::Vector3d _com_pos_start;
    /* TODO com_pos_goal is not used */
    Eigen::Vector3d _com_pos_goal;

    Eigen::Vector3d _delta_com;

    bool _flag_theta;
    std::array<Eigen::Affine3d, 2> _foot_pos_start;
    std::array<Eigen::Affine3d, 2> _foot_pos_goal;

    Eigen::Affine3d _waist_pos_start;
    Eigen::Affine3d _waist_pos_goal;

    Eigen::Vector3d _delta_com_tot;
    Eigen::Vector3d _delta_foot_tot;

    bool _zero_cross;
//    std::array<bool, 2> _foot_contact;

    /* initial and final time of foot movement */
    double _step_t_start;
    double _step_t_end;

    double _step_clearance;

    double _zmp_middle;
    std::vector<Eigen::MatrixXd> _zmp_vals;
    Eigen::VectorXd _durations;

    double _distance_ankle_com;
    double _height_com;

    bool _update_step;
    bool _execute_step;

    /* duration single stance and double stance */
    double _ss_duration, _ds_duration;
    /* parameters for the robot */
    mdof::StepState _step;
    std::shared_ptr<Param> _param;
    Engine::Ptr _engine;


    friend std::ostream& operator<<(std::ostream& os, Event s)
    {
        switch (s)
        {
            case Event::Empty : return os << "empty";
            case Event::SagReached :  return os << "sagittal completed";
            case Event::Start :  return os << "start";
            case Event::Stop :  return os << "stop";
            default : return os << "wrong event";
        }
    }


    friend std::ostream& operator<<(std::ostream& os, State s)
    {
        switch (s)
        {
            case State::Idle :  return os << "idle";
            case State::Walking : return os << "walking";
            case State::Starting : return os << "starting";
            case State::Stopping : return os << "stopping";
            case State::LastStep : return os << "last step";
            default : return os << "wrong state";
        }
    }
};


#include <param/param.h>

#endif // WALKER_H
