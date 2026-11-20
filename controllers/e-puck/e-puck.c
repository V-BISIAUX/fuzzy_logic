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

#include <stdio.h>
#include <string.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

#define SEUIL_BAS 100     // ps2 en dessous : trop loin du mur
#define SEUIL_HAUT 350    // ps2 au-dessus : trop près du mur
#define VITESSE_BASE 3.0  // vitesse commune des deux roues (rad/s)
#define DELTA 0.5         // écart de vitesse appliqué pour tourner

#define SEUIL_AVANT 300   // ps1

#define V_DROITE 0.5        // virage à droite
#define V_DROIT 0.0         // tout droit
#define V_GAUCHE -0.5       // virage à gauche
#define V_GAUCHE_FORT -2.0  // virage fort à gauche (coin)

#define V_RAPIDE 3.0
#define V_MOYENNE 2.0
#define V_LENTE 0.5

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}

static double trapeze(double x, double a, double b, double c, double d) {
    if (x < a || x > d) {
        return 0.0;  // cas hors du trapèze
    }
    if (x < b) {
        return (x - a) / (b - a);  // cas côté qui monte
    }
    if (x <= c) {
        return 1.0;  // cas plateau
    }

    return (d - x) / (d - c);  // cas côté qui descend
}

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i, j;
  double speed[2];
  double sensors_value[8];
  double braitenberg_coefficients[8][2] = {{0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06},
                                           {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942}};
  int time_step;
  int camera_time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) { 
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);

    static int compteur = 0;   // mémorise sa valeur d'un pas à l'autre
    if (++compteur % 8 == 0) { // on n'affiche qu'un pas sur 8
      printf("ps1=%4.0f ps2=%4.0f ps3=%4.0f | ps4=%4.0f ps5=%4.0f ps6=%4.0f\n",
             sensors_value[1], sensors_value[2], sensors_value[3],
             sensors_value[4], sensors_value[5], sensors_value[6]);
    }

    const double *a = wb_accelerometer_get_values(accelerometer);
    /* printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]); */

    /* compute odometry and speed values*/
    /* compute_odometry(left_position_sensor, right_position_sensor); */

  
    double D = sensors_value[2];   // ps2
    double F = sensors_value[1];   // ps1

    // fuzzification  
    double loin   = trapeze(D, 0, 0, 80, 150);
    double bon    = trapeze(D, 80, 150, 300, 400);
    double proche = trapeze(D, 300, 400, 10000, 10000);
    double devant = trapeze(F, 110, 200, 10000, 10000);

    // règles + défuzzification
    double somme  = loin + bon + proche + devant;
    double virage = (loin * V_DROITE + bon * V_DROIT + proche * V_GAUCHE + devant * V_GAUCHE_FORT) / somme;

    /* règles de vitesse : ET = min, OU = max, NON = 1 - valeur */
    double w_lente   = devant;                                  // devant proche -> lente
    double w_rapide  = fmin(bon, 1.0 - devant);                 // bon ET NON devant -> rapide
    double w_moyenne = fmin(fmax(loin, proche), 1.0 - devant);  // (loin OU proche) ET NON devant -> moyenne
    double vitesse = (w_lente * V_LENTE + w_rapide * V_RAPIDE + w_moyenne * V_MOYENNE)/ (w_lente + w_rapide + w_moyenne);

    speed[0] = vitesse + virage;
    speed[1] = vitesse - virage;

    if (compteur % 8 == 0)
      printf("D=%4.0f F=%4.0f | loin=%.2f bon=%.2f proche=%.2f devant=%.2f -> virage=%.2f\n", D, F, loin, bon, proche, devant, virage);
    if (compteur % 8 == 0)
      printf("D=%4.0f F=%4.0f | virage=%.2f vitesse=%.2f\n", D, F, virage, vitesse);

    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
