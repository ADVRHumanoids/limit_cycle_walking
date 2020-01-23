#include <gtest/gtest.h>
#include <param/param.h>

#include <XBotInterface/Utils.h>

class Process
{

public:

    Process(std::vector<const char *> args);

    int wait();

    void kill(int signal = SIGTERM);

    ~Process();

private:

    std::string _name;
    pid_t _pid;

};

Process::Process(std::vector< const char* > args):
    _name(args[0])
{
    /* args is a vector of pointers */

    /* tzhe second parameter passed to execvp() needs to be a pointer to character strings, with a final NULL pointer */
    args.push_back(nullptr);

    /*
     * args.data() returns a pointer to the first element of the vector, which is a pointer
     * converting this to a pointer to a pointer to a char
     */

    char ** argv = (char**)args.data();


    /* this clone the current process and return the pid of the child process, which I store */
    _pid = ::fork();

    if(_pid == -1)
    {
        perror("fork");
        throw std::runtime_error("Unable to fork()");
    }

    if(_pid == 0)
    {
        /* here the program enters if I forked correctly.
         * I substitute the current clone with the process specified in argv, using execvp.
         * If this substitution is correct, I will never reach perror("execvp"), because it is a new different process.
         */
        ::execvp(argv[0], argv);
        perror("execvp");
        throw std::runtime_error("Unknown command");
    }

}

int Process::wait()
{
    int status;
    while(::waitpid(_pid, &status, 0) != _pid);
    printf("Child process '%s' exited with status %d\n", _name.c_str(), status);
    return status;

}

void Process::kill(int signal)
{
    /* kill the process with the pid stored in _pid */
    ::kill(_pid, signal);
    printf("Killed process '%s' with signal %d\n", _name.c_str(), signal);
}

Process::~Process()
{
    kill(SIGINT);
    wait();
}


class TestParam: public ::testing::Test {


protected:

    TestParam()
    {
        /* these are the default parameters */

        /* init parameters */
        _initial_lowering = -0.12;
        _first_side_step = 0;
        _lean_forward = 0;

        /* walking parameters */
        _step_clearance = 0.1;
        _step_duration = 2;
        _double_stance_duration = 0;
        _max_inclination = 0.1;

        /* mpc parameters */
        _mpc_Q = 1000000;
        _mpc_R = 1;
        _horizon_duration = 5;
        _zmp_offset = 0;

    }

    virtual ~TestParam() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    double _initial_lowering;
    bool _first_side_step;
    double _lean_forward;

    /* walking parameters */
    double _step_clearance;
    double _step_duration;
    double _double_stance_duration;
    double _max_inclination;

    /* mpc parameters */
    double _mpc_Q;
    double _mpc_R;
    double _horizon_duration;
    double _zmp_offset;

};

TEST_F(TestParam, checkGetterYAML)
{
    std::string path_to_cfg(WALKER_TEST_CONFIG_PATH);
    YAML::Node walking_yaml = YAML::LoadFile(path_to_cfg + "test_param.yaml");

    Walker::Param param(walking_yaml);

    /* init parameters */
    _initial_lowering = 1;
    _first_side_step = false;
    _lean_forward = 2;

    /* walking parameters */
    _step_clearance = 3;
    _step_duration = 4;
    _double_stance_duration = 5;
    _max_inclination = 6;

    /* mpc parameters */
    _mpc_Q = 7;
    _mpc_R = 8;
    _horizon_duration = 9;
    _zmp_offset = 10;

    ASSERT_EQ(param.getInitialLowering(), _initial_lowering);
    ASSERT_EQ(param.getFirstSideStep(), _first_side_step);
    ASSERT_EQ(param.getLeanForward(), _lean_forward);
    ASSERT_EQ(param.getStepClearance(), _step_clearance);
    ASSERT_EQ(param.getStepDuration(), _step_duration);
    ASSERT_EQ(param.getDoubleStanceDuration(), _double_stance_duration);
    ASSERT_EQ(param.getMaxInclination(), _max_inclination);
    ASSERT_EQ(param.getMpcQ(), _mpc_Q);
    ASSERT_EQ(param.getMpcR(), _mpc_R);
    ASSERT_EQ(param.getHorizonDuration(), _horizon_duration);
    ASSERT_EQ(param.getZmpOffset(), _zmp_offset);

}

TEST_F(TestParam, checkGetterROS)
{

    ros::NodeHandle nh;
    Walker::Param param(nh);

    /* init parameters */
    _initial_lowering = 1;
    _first_side_step = false;
    _lean_forward = 2;

    /* walking parameters */
    _step_clearance = 3;
    _step_duration = 4;
    _double_stance_duration = 5;
    _max_inclination = 6;

    /* mpc parameters */
    _mpc_Q = 7;
    _mpc_R = 8;
    _horizon_duration = 9;
    _zmp_offset = 10;

    ASSERT_EQ(param.getInitialLowering(), _initial_lowering);
    ASSERT_EQ(param.getFirstSideStep(), _first_side_step);
    ASSERT_EQ(param.getLeanForward(), _lean_forward);
    ASSERT_EQ(param.getStepClearance(), _step_clearance);
    ASSERT_EQ(param.getStepDuration(), _step_duration);
    ASSERT_EQ(param.getDoubleStanceDuration(), _double_stance_duration);
    ASSERT_EQ(param.getMaxInclination(), _max_inclination);
    ASSERT_EQ(param.getMpcQ(), _mpc_Q);
    ASSERT_EQ(param.getMpcR(), _mpc_R);
    ASSERT_EQ(param.getHorizonDuration(), _horizon_duration);
    ASSERT_EQ(param.getZmpOffset(), _zmp_offset);

}


TEST_F(TestParam, checkGetterUninitialized)
{

    ros::NodeHandle nh;
    Walker::Param param;

    ASSERT_EQ(param.getInitialLowering(), _initial_lowering);
    ASSERT_EQ(param.getFirstSideStep(), _first_side_step);
    ASSERT_EQ(param.getLeanForward(), _lean_forward);
    ASSERT_EQ(param.getStepClearance(), _step_clearance);
    ASSERT_EQ(param.getStepDuration(), _step_duration);
    ASSERT_EQ(param.getDoubleStanceDuration(), _double_stance_duration);
    ASSERT_EQ(param.getMaxInclination(), _max_inclination);
    ASSERT_EQ(param.getMpcQ(), _mpc_Q);
    ASSERT_EQ(param.getMpcR(), _mpc_R);
    ASSERT_EQ(param.getHorizonDuration(), _horizon_duration);
    ASSERT_EQ(param.getZmpOffset(), _zmp_offset);

}
int main(int argc, char **argv) {

    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }

    Process roscore({"roscore", "-p", "11322"});


    /* Launch ros server node */
    ros::init(argc, argv, "param_ros_test");
    ros::NodeHandle nh("param");

    std::string param;
    std::string path_to_cfg(WALKER_TEST_CONFIG_PATH);
    XBot::Utils::ReadFile(path_to_cfg + "/test_param.yaml", param);

    nh.setParam("/walking_parameters", param);


    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
