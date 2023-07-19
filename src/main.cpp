/*  main_controller
    Author: Grant Gibson (Biped Robotics Lab)
*/

// Agility includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <ctime>

#include "lowlevelapi.h"

#include "Digit_Controller.hpp"


int main(int argc, char *argv[])
{
  // The publisher address should be changed to the ip address of the robot
  const char *publisher_address = "127.0.0.1";
  llapi_init(publisher_address);

  // Define inputs and outputs (updated each iteration)
  llapi_command_t command = {0};
  llapi_observation_t observation;

  // Connect to robot (need to send commands until the subscriber connects)
  command.apply_command = false;
  while (!llapi_get_observation(&observation)) llapi_send_command(&command);
  
  std::cout << "=========== Connected ===========\n\n";

  // Get local copy of command limits (torque and damping)
  const llapi_limits_t *limits = llapi_get_limits();

  // Initialize controller
  Digit_Controller controller;
  int init_ctrl_mode = std::atoi(argv[1]);
  int flag_torque_only = std::atoi(argv[2]);
  controller.Initialize_(init_ctrl_mode,flag_torque_only);
  controller.Set_Initial_Standing_Gains_();
  controller.Set_Initial_Walking_Gains_();
  std::cout << "Controller initialized" << "\n";

  // Extra variables
  int iter_loop = 1;
  double time_local_start = clock();
  double time_local = time_local_start;
  double time_local_prev = time_local;

  while (true)
  {
    /* Update observation */
    int return_val = llapi_get_observation(&observation);
    if (return_val < 1)
    {
    }
    else if (return_val)
    {

      /* Local time */
      time_local = clock();
      std::cout << "time_local: " << (time_local - time_local_start) / (double)CLOCKS_PER_SEC << "(" << 1.0 / ((time_local - time_local_prev) / (double)CLOCKS_PER_SEC) << ")\n\n";
      time_local_prev = time_local;

      /* Update Controller */
      std::cout << "Updating..." << "\n";
      controller.Update_(command, observation, limits);
      std::cout << "========================================\n\n";

      /* Fallback Operation Mode */
      command.fallback_opmode = Locomotion;
      command.apply_command = true;

      /* Send Command */
      llapi_send_command(&command);

      /* Increase loop iteration */
      iter_loop++;
    }
    else
    {
      // No new data
      std::cout << "No new data received\n\n";
    }

    /* Check if llapi has become disconnected */
    if (!llapi_connected())
    {
      std::cout << "\n------------------------- Disconnected! ---------------------------\n";
      // Handle error case. You don't need to re-initialize subscriber
      // Calling llapi_send_command will keep low level api open
    }

    usleep(1000);
  }

  return 0;
}
