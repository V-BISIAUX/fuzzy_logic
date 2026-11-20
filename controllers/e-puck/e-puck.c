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
#include <math.h>

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define COTE_MUR 1  // 1 = mur à droite, -1 = mur à gauche

#define V_DROITE 0.5  // vers le mur
#define V_DROIT 0.0  // tout droit
#define V_GAUCHE -0.5  // loin du mur
#define V_GAUCHE_FORT -3.0  // loin du mur, fort (coin)

#define V_RAPIDE 3.0
#define V_MOYENNE 2.0
#define V_LENTE 0.5

/* Fonction permettant la création des trapèzes */
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
  WbDeviceTag distance_sensor[8], left_motor, right_motor;
  int i;
  int time_step;
  double speed[2];
  double sensors_value[8];

  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    time_step = 64;
  } else {  // e-puck d'origine
    time_step = 256;
  }

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);

    /* entrées : la distance au mur et obstacle devant, selon le côté suivi */
    double D = (COTE_MUR == 1) ? sensors_value[2] : sensors_value[5];  // ps2 ou ps5
    double F = (COTE_MUR == 1) ? sensors_value[1] : sensors_value[6];  // ps1 ou ps6

    /* fuzzification */
    double loin = trapeze(D, 0, 0, 80, 150);
    double bon = trapeze(D, 80, 150, 300, 400);
    double proche = trapeze(D, 300, 400, 10000, 10000);
    double devant = trapeze(F, 150, 250, 10000, 10000);

    /* règles de virage + défuzzification */
    double somme = loin + bon + proche + devant;
    double virage = (loin * V_DROITE + bon * V_DROIT + proche * V_GAUCHE + devant * V_GAUCHE_FORT) / somme;

    /* règles de vitesse (ET = min, OU = max, NON = 1 - valeur) + défuzzification */
    double w_lente = devant;  // devant proche -> lente
    double w_rapide = fmin(bon, 1.0 - devant);  // bon ET NON devant -> rapide
    double w_moyenne = fmin(fmax(loin, proche), 1.0 - devant);  // (loin OU proche) ET NON devant -> moyenne
    double vitesse = (w_lente * V_LENTE + w_rapide * V_RAPIDE + w_moyenne * V_MOYENNE) / (w_lente + w_rapide + w_moyenne);

    /* vitesses des roues : le signe du virage s'inverse selon le côté suivi */
    speed[0] = vitesse + COTE_MUR * virage;
    speed[1] = vitesse - COTE_MUR * virage;

    /* affichage : capteurs ps1 à ps6, virage et vitesse (un pas sur 8) */
    static int compteur = 0;
    if (++compteur >= 8) {
      compteur = 0;
      printf("ps1=%4.0f ps2=%4.0f ps3=%4.0f ps4=%4.0f ps5=%4.0f ps6=%4.0f | virage=%5.2f vitesse=%4.2f\n", sensors_value[1], sensors_value[2], sensors_value[3], sensors_value[4], sensors_value[5], sensors_value[6], virage, vitesse);
    }

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
