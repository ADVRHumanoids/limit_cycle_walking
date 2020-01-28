#ifndef STEP_MACHINE_H
#define STEP_MACHINE_H

#include <robot/robot_state.h>
#include <engine/engine.h>
#include <walker/foot_trajectory.h>

class Walker {
public:

    typedef std::shared_ptr<Walker> Ptr;

    class Param;

    Walker(double dt, std::shared_ptr<Param> par = getDefaultParam()); /* TODO */

    enum class Event { Impact = 0, Start = 1, Stop = 2, Empty = 3 }; /* some private (Impact), some public ? */
    enum class State { Idle = 0, Walking = 1, Starting = 2, Stopping = 4, LastStep = 5 };

    bool init(const mdof::RobotState &state);

    bool start();
    bool stop();

    /* parametrization a la Bennewitz */
    bool setStep(double delta_x, double delta_y, double delta_theta);

    bool setQMax(std::vector<double> q_max);
    bool setTheta(std::vector<double> theta);


    bool update(double time,
                const mdof::RobotState &state,
                mdof::RobotState &ref);

    bool homing(const mdof::RobotState &state,
                mdof::RobotState &ref);

    State getState() {return _current_state;}

    friend std::ostream& operator<<(std::ostream& os, Event s);
    friend std::ostream& operator<<(std::ostream& os, State s);

private:

    static std::shared_ptr<Walker::Param> getDefaultParam();

    bool step_machine(double time);

    bool impactDetector(double time,
                        double q,
                        double q_min,
                        double q_max,
                        double swing_leg_heigth,
                        double terrain_heigth);

    bool landingHandler(double time,
                        const mdof::RobotState &state);


    bool computeQFake(double time,
               double q,
               double q_min,
               double q_max,
               double steep_q,
               bool started,
               double start_walk,
               double& q_fake);

    double computeQ(bool current_swing_leg,
                  double theta,
                  Eigen::Vector3d world_T_com,
                  Eigen::Vector3d world_T_com_start,
                  std::array<Eigen::Affine3d, 2> ankle_T_com);

    bool updateQMax();

//    bool resetter();

    State _current_state, _previous_state;
    Event _current_event, _previous_event;

    /* parameters of the stepping motion */

    int _step_counter, _cycle_counter;
    double _steep_q;

    double _t_impact;

    double _theta;


    double _time;
    double _new_event_time;

    /* received command to start walking */
    bool _started;

    /* starting time */
    double _t_start_walk;

    /* phase variable */
    double _q;

    /* current minimum q */
    double _q_min;

    /* current maximum q */
    double _q_max;

    /* fake q */
    double _q_fake;

    /* dt of the control */
    double _dt;

    /* swinging leg (0 -> left, 1 -> right) */
    bool _current_swing_leg;

    /* time between instant that received START command is received and instant where robot starts walking. Required for initial lateral shift of the CoM */
    double _delay_start;

    /* buffer of commands for step */
    std::deque<double> _q_buffer;
    std::deque<double> _theta_buffer;

    double _terrain_height;

    Eigen::Vector3d _com_pos_start;
    Eigen::Vector3d _com_pos_goal;

    std::array<Eigen::Affine3d, 2> _foot_pos_start;
    std::array<Eigen::Affine3d, 2> _foot_pos_goal;

    Eigen::Affine3d _waist_pos_start;
    Eigen::Affine3d _waist_pos_goal;

//    std::array<bool, 2> _foot_contact;

    double _step_t_start;
    double _step_t_end;

    double _step_duration;
    double _step_clearance;

    double _middle_zmp;

    /* parameters for the robot */
//    std::shared_ptr<Param> _param;
    std::shared_ptr<Param> _param;

    mdof::StepState * _step;
    Engine::Ptr _engine;


    friend std::ostream& operator<<(std::ostream& os, Event s)
    {
        switch (s)
        {
            case Event::Empty : return os << "empty";
            case Event::Impact :  return os << "impact";
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

#endif // STEP_MACHINE_H
