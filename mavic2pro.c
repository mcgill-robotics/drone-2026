/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/lidar.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high)                                                \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");

  // ADD THIS - enable lidar and point cloud (visual in 3D view)
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);
  wb_lidar_enable_point_cloud(lidar);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor,
                           rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  printf("Start the drone...\n");

  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }
  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");

  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;

  double target_altitude = 1.0;

  // ADD THIS - print lidar data only every N steps.
  int lidar_print_counter = 0;
  FILE *lidar_file = fopen("lidar_log.csv", "w");
  if (lidar_file == NULL) {
    printf("ERROR: could not open lidar_log.csv for writing\n");
  }
  if (lidar_file != NULL)
    setvbuf(lidar_file, NULL, _IONBF, 0);
  // Main loop
  double vx = 0.0, vy = 0.0, vz = 0.0, yaw_rate = 0.0;
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];

    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

    // ADD THIS - print lidar point cloud every 100 steps
    lidar_print_counter++;
    if (lidar_print_counter >= 100) {
      lidar_print_counter = 0;

      int num_points = wb_lidar_get_number_of_points(lidar);
      const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar);

      printf("--- LiDAR: %d points ---\n", num_points);

      // Print first 10 points so the console doesn't get flooded
      int print_count = num_points < 10 ? num_points : 10;
      for (int i = 0; i < print_count; i++) {
        float x = point_cloud[i].x;
        float y = point_cloud[i].y;
        float z = point_cloud[i].z;
        // distance from drone to that point
        float dist = sqrtf(x * x + y * y + z * z);
        // printf("  point[%d]: x=%.2f y=%.2f z=%.2f\n", i, x, y, z);
        // printf("  dist=%.2f m\n", dist);
        printf("%.4f,%.4f,%.4f,%.4f,%.4f\n", wb_robot_get_time(), x, y, z,
               dist);
        if (lidar_file != NULL) {
          fprintf(lidar_file, "%.4f,%.4f,%.4f,%.4f,%.4f\n", wb_robot_get_time(),
                  x, y, z, dist);
        }
      }
    }

    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
      case WB_KEYBOARD_UP:
        pitch_disturbance = -2.0;
        break;
      case WB_KEYBOARD_DOWN:
        pitch_disturbance = 2.0;
        break;
      case WB_KEYBOARD_RIGHT:
        yaw_disturbance = -1.3;
        break;
      case WB_KEYBOARD_LEFT:
        yaw_disturbance = 1.3;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
        roll_disturbance = -1.0;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
        roll_disturbance = 1.0;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
        target_altitude += 0.05;
        // printf("target altitude: %f [m]\n", target_altitude);
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
        target_altitude -= 0.05;
        // printf("target altitude: %f [m]\n", target_altitude);
        break;
      }
      key = wb_keyboard_get_key();
    }

    const double roll_input =
        k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) +
                               pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude =
        CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input =
        k_vertical_p * pow(clamped_difference_altitude, 3.0);

    const double front_left_motor_input = k_vertical_thrust + vertical_input -
                                          roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input +
                                           roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input -
                                         roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input +
                                          roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  };
  fclose(lidar_file);
  wb_robot_cleanup();

  return EXIT_SUCCESS;
}