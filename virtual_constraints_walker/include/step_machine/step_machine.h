#ifndef STEP_MACHINE_H
#define STEP_MACHINE_H

#include <robot/step_state.h>
#include <walker/walker.h>


class StepMachine {
public:

    class Param;

    StepMachine(double dt, Param * par = getDefaultParam()); /* TODO */

    enum class Event { Impact = 0, Start = 1, Stop = 2, Empty = 3 }; /* some private (Impact), some public ? */
    enum class State { Idle = 0, Walking = 1, Starting = 2, Stopping = 4, LastStep = 5 };

    bool initialize(const mdof::RobotState * state);

    bool start();
    bool stop();

    /* parametrization a la Bennewitz */
    bool setStep(double delta_x, double delta_y, double delta_theta);

    bool setQMax(std::vector<double> q_max);
    bool setTheta(std::vector<double> theta);


    bool update(double time,
                const mdof::RobotState * state,
                mdof::RobotState * ref);

    bool homing(double time,
                const mdof::RobotState * state,
                mdof::RobotState * ref);

    friend std::ostream& operator<<(std::ostream& os, Event s);
    friend std::ostream& operator<<(std::ostream& os, State s);

private:

    static Param * getDefaultParam();

    bool core(double time,
              double q_fake,
              mdof::RobotState * ref);

    bool impactDetector(double time,
                        double q,
                        double q_min,
                        double q_max,
                        double swing_leg_heigth,
                        double terrain_heigth);

    bool landingHandler(double time,
                        double terrain_heigth,
                        const mdof::RobotState * state,
                        mdof::RobotState * ref);


    bool StepMachine::qFake(double time,
                            double q,
                            double q_min,
                            double q_max,
                            double steep_q,
                            bool started,
                            double start_walk,
                            double& q_fake);

    bool updateStep(mdof::RobotState * ref);

    bool resetter();

    State _current_state, _previous_state;
    Event _current_event, _previous_event;

    /* parameters of the stepping motion */
    mdof::StepState * _step;

    int _step_counter, _cycle_counter;
    double _steep_q;

    double _time_impact;

    double _theta;

    Walker::Ptr _walker;

    /* parameters for the robot */
    Param * _param;

    double _time;
    double _new_event_time;

    /* received command to start walking */
    bool _started;

    /* starting time */
    double _start_walk;

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

    std::deque<double> _q_buffer;
    std::deque<double> _theta_buffer;


    double _terrain_height;


};


#include <param/param.h>

#endif // STEP_MACHINE_H
