/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

//#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "estimator_kalman.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "radiolink.h"
#include "configblock.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "usec_time.h"
#include "peer_localization.h"
#include "math3d.h"



#define DEBUG_MODULE "HELLOWORLD"
#define dt_msec 50
#define neig_num_d 4
#define obs_num_d 5
#define total_search_num 7

// Flocking algorithm params
uint8_t neig_id;
uint8_t quad_num = 5;
uint8_t neig_num = 4;
uint8_t obs_num = 5;
uint8_t neig_kk = 3;
uint8_t obs_kk = 2;
uint8_t acc_search_num = 3;

float migration_speed = 0.5;
float dt = dt_msec / 1e3;
float d_safety_obs = 0.5;
float d_safety_rob = 0.3;
float d_ref = 0.8;
float kColl = 1000.0;
float kObs = 10.0;
float d_0 = 2.0;
float kRob = 250.0;
float kDir = 2.0;
float kSpd = 2.0;
float xiPow = 3.0;
float zp5 = 0.5;
float spd_min = 0.05;
float spd_max = 5.0;
float acc_max = 2.0;
float kHeig = 100.0;
float kAlt = 1.0;
float hMargin = 0.5;
float vZeroThresh = 0.001;

float neig_X[] = {0,0,0,0,0};
float neig_Y[] = {0,0,0,0,0};
float neig_Z[] = {0,0,0,0,0};
bool neig_alive[] = {false, false, false, false, false};
bool neig_alive_arr[] = {false, false, false, false};

float obs_X[] = {-1.5, -0.35, -1.49, 0.75, 0.75};
float obs_Y[] = {-0.34, 0.9, 2.19, -0.35, 2.17};

float take_off_height = 0.75;
float des_height = 0.75;
float rng_pos_list[5][3] = {{-0.2915, 0.0962, 0.7096} , {0.3602, 0.5931, 0.8426} , {-0.9999, 0.6728, 0.6022} , {-0.8488, -0.5016, 0.9391} , {0.0734, -0.5306, 0.5137}};
float acc_list[7][3] = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {0, 0, 0}};

float calc_spd;
float start_time,end_time;
float altitude_z;

struct vec acc_vec_list[total_search_num];
struct vec neig_pos_arr[neig_num_d];
struct vec obs_pos_arr[obs_num_d];
struct vec search_vel_arr[total_search_num];

struct vec migration_vel;
struct vec pos_vec;
struct vec vel_vec;
struct vec search_pos_vec;
struct vec search_vel_vec;
struct vec search_head_vec;
struct vec neig_rob_sub_vec;
struct vec obs_rob_sub_vec;
struct vec obs_uni_vec;
struct vec search_acc_vec;

float search_heur_arr[total_search_num];
float neig_dist_arr[neig_num_d];
float obs_dist_arr[obs_num_d];
float neig_dist;
float obs_dist;
float heur;
float xi;
float prevHeig;

//static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdFlightState;


typedef struct {
  uint8_t id;
  float x;
  float y;
  float z;
  } _coords;

void insertion_sort_multi_two(float arr[], uint8_t n, struct vec arr2[]) {
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    struct vec key2 = arr2[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
        arr[j + 1] = arr[j];
        arr2[j + 1] = arr2[j];
        j--;
    }
    arr[j + 1] = key;
    arr2[j + 1] = key2;
  }
}

void insertion_sort_multi_neig(float arr[], uint8_t n, struct vec arr2[], bool arr3[]) {
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    struct vec key2 = arr2[i];
    bool key3 = arr3[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
        arr[j + 1] = arr[j];
        arr2[j + 1] = arr2[j];
        arr3[j + 1] = arr3[j];
        j--;
    }
    arr[j + 1] = key;
    arr2[j + 1] = key2;
    arr3[j + 1] = key3;
  }
}

void appMain() {

  static uint8_t flightstate;
  flightstate = 3; //0;
  
  PARAM_GROUP_START(fmodes)
  PARAM_ADD_CORE(PARAM_UINT8, ufs, &flightstate)
  PARAM_GROUP_STOP(fmodes)
  
  paramIdFlightState = paramGetVarId("fmodes", "ufs");
  
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");

  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id = (uint8_t)((address) & 0x00000000ff);

  _coords self_coords;
  self_coords.id = my_id;

  migration_vel.x = 0.0;
  migration_vel.y = migration_speed;
  migration_vel.z = 0.0;

  for (int i = 0; i < total_search_num; ++i) {
    acc_vec_list[i].x = acc_max*acc_list[i][0];
    acc_vec_list[i].y = acc_max*acc_list[i][1];
    acc_vec_list[i].z = acc_max*acc_list[i][2];
  }

  vel_vec.x = 0.0;
  vel_vec.y = 0.5;
  vel_vec.z = 0.0;

  paramSetInt(paramIdResetKalman, 1);
  //paramSetInt(paramIdCommanderEnHighLevel, 1);

  while(1) {

    if (flightstate == 3){

      start_time = usecTimestamp() / 1e3;
      self_coords.x = logGetFloat(idX);
      self_coords.y = logGetFloat(idY);
      self_coords.z = logGetFloat(idZ);

      pos_vec.x = rng_pos_list[my_id-1][0]; //self_coords.x;
      pos_vec.y = rng_pos_list[my_id-1][1]; //self_coords.y;
      pos_vec.z = rng_pos_list[my_id-1][2]; //self_coords.z;

      prevHeig = pos_vec.z;

      //neighbor position vector array
      int j = 0;
      for (int i = 0; i < quad_num; ++i) {
        if ((i+1) != my_id) {
          neig_pos_arr[j].x = rng_pos_list[i][0]; //neig_X[i];
          neig_pos_arr[j].y = rng_pos_list[i][1]; //neig_Y[i];
          neig_pos_arr[j].z = rng_pos_list[i][2]; //neig_Z[i];
          neig_alive_arr[j] = true; //neig_alive[i];
          j = j + 1;
        }
      }

      for (int i = 0; i < obs_num; ++i) {
        obs_pos_arr[i].x = obs_X[i];
        obs_pos_arr[i].y = obs_Y[i];
        obs_pos_arr[i].z = pos_vec.z;
      }

      for (int i = 0; i < neig_num; ++i) {
        neig_rob_sub_vec = vsub(neig_pos_arr[i],pos_vec);
        neig_dist_arr[i] = vmag(neig_rob_sub_vec); 
      }

      insertion_sort_multi_neig(neig_dist_arr, neig_num, neig_pos_arr, neig_alive_arr);

      for (int i = 0; i < neig_num; ++i) {
        neig_pos_arr[i].y = neig_pos_arr[i].y + dt*migration_speed; //calculate the predicted position of the neighbor for v^m in the y-axis direction
      }


      for (int i = 0; i < obs_num; ++i) {
          obs_rob_sub_vec = vsub(obs_pos_arr[i],pos_vec);
          obs_dist_arr[i] = vmag(obs_rob_sub_vec);
      }

      insertion_sort_multi_two(obs_dist_arr, obs_num, obs_pos_arr);

      for (int k = 0; k < total_search_num; ++k) {

        heur = 0;

        search_acc_vec = acc_vec_list[k];
        search_vel_vec = vadd(vel_vec, vscl(dt,search_acc_vec));

        calc_spd = clamp(vmag(search_vel_vec),spd_min,spd_max);


        if ((search_vel_vec.x == (float)0) && (search_vel_vec.y == (float)0) && (search_vel_vec.z == (float)0)) {
          search_head_vec.x = 0.0;
          search_head_vec.y = 1.0;
          search_head_vec.z = 0.0;
        }
        else {
          search_head_vec = vdiv(search_vel_vec,vmag(search_vel_vec));
        }

        search_vel_vec = vscl(calc_spd,search_head_vec);

        search_pos_vec = vadd(pos_vec,vscl(dt,search_vel_vec));

        float heur_obs = 0;
        for (int i = 0; i < obs_kk; ++i) {
          obs_rob_sub_vec = vsub(obs_pos_arr[i],search_pos_vec);
          obs_dist = vmag(obs_rob_sub_vec);
          if (obs_dist < d_0) {
            obs_uni_vec = vdiv(obs_rob_sub_vec,obs_dist);
            xi = vdot(search_head_vec,obs_uni_vec);
            if (xi < (float)0){xi = 0.0;}
            if (obs_dist <= d_safety_obs){
                heur_obs = heur_obs + kColl*kObs*zp5*powf(1/obs_dist - 1/d_0,2);
            }
            else{
                heur_obs = heur_obs + powf(xi,xiPow)*kObs*zp5*powf(1/obs_dist - 1/d_0,2);
            }
          }
        }
        heur_obs = heur_obs/(float)obs_kk;

        uint8_t alive_neig_num = 0;
        float heur_neig = 0;
        for (int i = 0; i < neig_num; ++i) {
          bool is_neig_alive = neig_alive_arr[i];
          if (is_neig_alive){
            alive_neig_num = alive_neig_num + 1;
            neig_rob_sub_vec = vsub(neig_pos_arr[i],search_pos_vec);
            neig_dist = vmag(neig_rob_sub_vec);
            if (neig_dist <= d_safety_rob){
              heur_neig = heur_neig + kColl*kRob*zp5*powf(neig_dist-d_ref,2);
            }
            else{
              heur_neig = heur_neig + kRob*zp5*powf(neig_dist-d_ref,2);
            }
          }
          if (alive_neig_num == neig_kk){break;}
        }
        if (alive_neig_num > 0){heur_neig = heur_neig/(float)alive_neig_num;}

        float heur_dir = kDir*(1 - vdot(search_vel_vec,migration_vel)/(calc_spd*migration_speed));
        float heur_spd = kSpd*fabsf(migration_speed - calc_spd)/migration_speed;
        
        float heur_height = 0;
        float heigthErr = fabsf(search_pos_vec.z - des_height);
        if (heigthErr > hMargin){
          heur_height = kHeig*zp5*powf(1/heigthErr - 1/hMargin,2);
        }           

        float heur = heur_obs + heur_neig + heur_dir + heur_spd + heur_height;

        search_vel_arr[k] = search_vel_vec;
        search_heur_arr[k] = heur;
      }

      insertion_sort_multi_two(search_heur_arr, total_search_num, search_vel_arr);

      float vx = search_vel_arr[0].x;
      float vy = search_vel_arr[0].y;
      float vz = search_vel_arr[0].z;
      
      if (fabsf(vz) < vZeroThresh){
        vz = kAlt*(prevHeig - pos_vec.z);
      }
      else{
        prevHeig = pos_vec.z;
      }
      
      vel_vec = mkvec(vx,vy,vz);

      end_time = usecTimestamp() / 1e3;
      float computation_time = (end_time - start_time);
      DEBUG_PRINT("%f\n",(double)computation_time);

      vTaskDelay(M2T(dt_msec)); 
      //DEBUG_PRINT("Velocity input is: %f %f %f \n", (double)vx, (double)vy, (double)vz);
      //DEBUG_PRINT("Heading input is: %f \n", (double)self_head);
    }
  }
}

