#ifndef STEP_MACHINE_H
#define STEP_MACHINE_H

#include <robot/step_state.h>
#include <walker/walker.h>


class StepMachine {
public:

    class Param;

    StepMachine(mdof::RobotState * state); /* TODO */

    enum class Event { Impact = 0, Start = 1, Stop = 2, Empty = 3 }; /* some private (Impact), some public ? */
    enum class State { Idle = 0, Walking = 1, Starting = 2, Stopping = 4, LastStep = 5 };


    bool setEvent(Event event);
    bool setParameters(double q_max, double theta) {_q_max = q_max; _theta = theta;};

    friend std::ostream& operator<<(std::ostream& os, Event s);
    friend std::ostream& operator<<(std::ostream& os, State s);

    bool update(const mdof::RobotState * state,
                mdof::RobotState * ref);

private:


    bool core();
    bool impactDetector();
    bool landingHandler();
    double qFake();



    State _current_state, _previous_state;
    Event _current_event, _previous_event;

    /* parameters of the stepping motion */
    mdof::StepState * _step;

    mdof::RobotState * _state;

    int _step_counter;
    int _steep_q;

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

    /* time between instant that received START command is received and instant where robot starts walking. Required for initial lateral shift of the CoM */
    double _delay_start = 1.5;

    /* phase variable */
    double _q;

    /* current minimum q */
    double _q_min;

    /* current maximum q */
    double _q_max;

    /* temporary q */
    double _q_temp;

    /* dt of the control */
    double _dt;



};


#include <param/param.h>

#endif // STEP_MACHINE_H
