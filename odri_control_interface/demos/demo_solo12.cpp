#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>

// typedef Eigen::Matrix<double, 12, 1> Vector12d;
//typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 1, 1> Vector1d;
//typedef Eigen::Matrix<double, 3, 1> Vector3d;

int main()
{
    nice(-20);  // Give the process a high priority.

    // Define the robot from a yaml file.
    //auto robot = RobotFromYamlFile(CONFIG_SOLO12_YAML);
    
    std::cout << "here 1: " << std::endl;
    auto robot = RobotFromYamlFile(CONFIG_SOLO12_ONE_JOINT_YAML);
    //auto robot = RobotFromYamlFile(CONFIG_SOLO12_NO_J1_YAML);
    // auto robot = RobotFromYamlFile(CONFIG_SOLO12_YAML);
    std::cout << "here 2: " << std::endl;
    robot->Start();
    std::cout << "here 3: " << std::endl;
    robot->WaitUntilReady();
    std::cout << "here 4: " << std::endl;

    // Store initial position data.
    
    
    // des_pos << 0.0, 0.7, -1.4, -0.0, 0.7, -1.4, 0.0, -0.7, +1.4, -0.0, -0.7,
    //     +1.4;
    // 


    // Initialize the communication, session, joints, wait for motors to be ready
    // and run the joint calibration.
    // robot->Initialize(des_pos);

    // Initialize simple pd controller.
    // Vector12d des_pos;
    // des_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Vector12d torques;

    // Vector12d positions;
    // Vector12d velocities;
    // Vector12d gain_KP;
    // Vector12d gain_KD;

    // Vector8d des_pos;
    // des_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Vector8d torques;
    // Vector8d positions;
    // Vector8d velocities;
    // Vector8d gain_KP;
    // Vector8d gain_KD;

    
    Vector1d des_pos;
    des_pos << 0.0;
    Vector1d torques;

    Vector1d positions;
    Vector1d velocities;
    Vector1d gain_KP;
    Vector1d gain_KD;

    // Vector3d des_pos;
    // des_pos << 0.0, 0.0, 0.0;
    // Vector3d torques;

    // Vector3d positions;
    // Vector3d velocities;
    // Vector3d gain_KP;
    // Vector3d gain_KD;


    //double kp = 5.;
    double kp = 20.;
    double kd = 0.05;
    int c = 0;
    double dt = 0.001;
	double t = 0;
    double freq = 0.5;
	double amplitude = 0.35;
    bool comand_success = true;
    while (!robot->IsTimeout())
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
            //double v_ref = 2. * M_PI * freq * amplitude * cos(2 * M_PI * freq * t);
            positions[i] = ref;
            velocities[i] = 0.0;
            torques[i] = 0.0;
            // if (i == 0 || i == 5){
            //     gain_KP[i] = 0;
            //     gain_KD[i] = kd;
            // }
            // else{
            //     gain_KP[i] = kp;
            //     gain_KD[i] = kd;
            // }
            gain_KP[i] = kp;
            gain_KD[i] = kd;
        }
        // robot->joints->SetTorques(torques);
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
        if (c % 250 == 0)
        {
            std::cout << "Joints: ";
            robot->joints->PrintVector(robot->joints->GetPositions());
            std::cout << std::endl;
            std::cout << "References: ";
            robot->joints->PrintVector(positions);
            std::cout << std::endl;
            std::cout << "Command success: " << comand_success << std::endl;
        }
    }

    printf("Timeout detected\n");


    return 0;
}
