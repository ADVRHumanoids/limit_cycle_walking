#include <walker/walker.h>
#include <state_machine/state_machine.h>

std::ostream& operator<<(std::ostream& os, Walker::Event s)
{
    switch (s)
    {
    case Walker::Event::Empty : return os << "empty";
    case Walker::Event::Impact :  return os << "impact";
    case Walker::Event::Start :  return os << "start";
    case Walker::Event::Stop :  return os << "stop";
    default : return os << "wrong event";
    }
};


std::ostream& operator<<(std::ostream& os, Walker::State s)
{
    switch (s)
    {
    case Walker::State::Idle :  return os << "idle";
    case Walker::State::Walking : return os << "walking";
    case Walker::State::Starting : return os << "start walking";
    case Walker::State::Stopping : return os << "stop walking";
    case Walker::State::LastStep : return os << "last step";
    default : return os << "wrong state";
    }
};

std::ostream& operator<<(std::ostream& os, Walker::Phase s)
{
    switch (s)
    {
    case Walker::Phase::Land :  return os << "land";
    case Walker::Phase::Flight :  return os << "flight";
    default : return os << "wrong phase";
    }
};

std::ostream& operator<<(std::ostream& os, Walker::Side s)
{
    switch (s)
    {
    case Walker::Side::Left :  return os << "Left";
    case Walker::Side::Right :  return os << "Right";
    case Walker::Side::Double :  return os << "Double";
    default : return os << "wrong side";
    }
};
};
