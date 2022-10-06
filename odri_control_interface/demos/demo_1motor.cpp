#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <map>


typedef Eigen::Matrix<double, 1, 1> Vector1d;

int main(int argc, char** argv)
{
    if (argc != 2){
        std::cout << "invalid number of arguments" << std::endl;
        return 1;
    }
    std::map <int, std::string> filenames { {8, CONFIG_SOLO_MOTOR8_YAML}, {4, CONFIG_SOLO_MOTOR4_YAML}, {2, CONFIG_SOLO_MOTOR2_YAML}, {10, CONFIG_SOLO_MOTOR10_YAML}, {9, CONFIG_SOLO_MOTOR9_YAML},{3, CONFIG_SOLO_MOTOR3_YAML},
     {5, CONFIG_SOLO_MOTOR5_YAML}, {11, CONFIG_SOLO_MOTOR11_YAML}, {6, CONFIG_SOLO_MOTOR6_YAML}, {1, CONFIG_SOLO_MOTOR1_YAML}, {7, CONFIG_SOLO_MOTOR7_YAML}, {0, CONFIG_SOLO_MOTOR0_YAML}  };
    nice(-20);  // Give the process a high priority.

    

    // Define the robot from a yaml file.
    int motor_number = atoi(argv[1]);

    auto robot = RobotFromYamlFile(filenames[motor_number]);
    
    std::cout << "demo_1motor.cpp: Define the robot from a yaml file done!" << std::endl;
    robot->Start();
    std::cout << "demo_1motor.cpp: robot->Start(); done!" << std::endl;
    robot->WaitUntilReady();
    std::cout << "demo_1motor.cpp: robot->WaitUntilReady(); done!" << std::endl;

    // Store initial position data.
    

    Vector1d des_pos;
    des_pos << 0.0;

    Vector1d torques;

    Vector1d positions;
    Vector1d velocities;
    Vector1d gain_KP;
    Vector1d gain_KD;

    double kp = 5.;
    //double kp = 20.;
    double kd = 0.05;
    //double kd = 0;
    int c = 0;
    double dt = 0.001;
	double t = 0;
    double freq = 0.5;
	double amplitude = 0.35;
    bool comand_success = true;

    std::ofstream outfile;
    std::string outfile_name = std::string("motor") + std::to_string(motor_number) + std::string(".csv");
    outfile.open(outfile_name, std::fstream::out);
    while (!robot->IsTimeout() && c < 10000)
    {
        robot->ParseSensorData();

        // Run the main controller.
        auto pos = robot->joints->GetPositions();
        auto vel = robot->joints->GetVelocities();

        // Compute PD control on the zero position.
        for (int i = 0; i < 1; i++)
        //for (int i = 0; i < 3; i++)
        {
            double ref = des_pos[i] + amplitude * sin(2 * M_PI * freq * t);
            //double ref = des_pos[i] + copysign(amplitude, sin(2 * M_PI * freq * t));
            double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);
            positions[i] = ref;
            velocities[i] = v_ref;
            torques[i] = 0.0;
            gain_KP[i] = kp;
            gain_KD[i] = kd;

        }
        robot->joints->SetTorques(torques);
        robot->joints->SetDesiredPositions(positions);
        robot->joints->SetDesiredVelocities(velocities);
        robot->joints->SetPositionGains(gain_KP);
        robot->joints->SetVelocityGains(gain_KD);

        // Checks if the robot is in error state (that is, if any component
        // returns an error). If there is an error, the commands to send
        // are changed to send the safety cont1rol.
        // robot->SendCommandAndWaitEndOfCycle(0.001);
        comand_success = robot->SendCommandAndWaitEndOfCycle(0.001);

        c++;
        t += dt;
        if (c % 50 == 0)
        {
            std::cout << "Joints: ";
            auto robot_positions = robot->joints->GetPositions();
            robot->joints->PrintVector(robot_positions);
            std::cout << std::endl;
            std::cout << "References: ";
            robot->joints->PrintVector(positions);
            std::cout << std::endl;
            std::cout << "Command success: " << comand_success << std::endl;
            outfile << t << "," << positions[0] << "," << robot_positions[0] << std::endl;
        }
    }
    outfile.close();

    printf("Timeout detected\n");


    return 0;
}
