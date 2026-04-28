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

uint8_t neig_id; //id of the quadcopter

//flocking method parameters:
uint8_t quad_num = 5; //number of quadcopters
uint8_t neig_num = 4; //number of neighbors
uint8_t obs_num = 5; //number of obstacles
uint8_t neig_kk = 3; //max. number of neihgbors for topological selection
uint8_t obs_kk = 2; //max. number of obstacles for topological selection
uint8_t acc_search_num = 3;

/*
float neig_X_print;
float neig_Y_print;
float neig_Z_print;
*/

float migration_speed = 0.5; //migration speed
float dt = dt_msec / 1e3; //dt in seconds
float d_safety_obs = 0.5; //safety distance for obstacles
float d_safety_rob = 0.3; //safety distance for neighbors
float d_ref = 0.8; //reference inter-robot distance
float kColl = 1000.0; //collision gain
float kObs = 10.0; //obstacle avoidance gain
float d_0 = 2.0; //obstacle influence threshold
float kRob = 250.0; //inter-robot distance gain
float kDir = 2.0; //direction gain
float kSpd = 2.0; //speed gain
float xiPow = 3.0; //power of approach rate 
float spd_min = 0.05; //min. speed
float spd_max = 5.0; //max. speed
float acc_max = 2.0; //max. acceleration
float kHeig = 100.0; //altitude gain
float hMargin = 0.5; //altitude margin

float kAlt = 1.0; //gain for altitude control
float vZeroThresh = 0.001; //threshold for checking values around zero
float zp5 = 0.5;

//arrays for tracking positions of neighbors:
float neig_X[] = {0,0,0,0,0}; //x-positions
float neig_Y[] = {0,0,0,0,0}; //y-positions
float neig_Z[] = {0,0,0,0,0}; //z-positions

//arrays for tracking whether quadcopters are alive or not
bool neig_alive[] = {false, false, false, false, false};
bool neig_alive_arr[] = {false, false, false, false};

//x-y positions of the cylindrical obstacles in the flight arena:
float obs_X[] = {-1.5, -0.35, -1.49, 0.75, 0.75}; //x-positions
float obs_Y[] = {-0.34, 0.9, 2.19, -0.35, 2.17}; //y-positions

float take_off_height = 0.75; //take-off height
float des_height = 0.75; //reference altitude

//x-y-z positions of 5 quadcopters generated randomly:
float rng_pos_list[5][3] = {{-0.0415, -2.6538, 0.7096}, {0.6102, -2.1569, 0.8426}, {-0.7499, -2.0772, 0.6022}, {-0.5988, -3.2516, 0.9391}, {0.3234, -3.2806, 0.5137}};
//acceleration set for generating predicted states:
float acc_list[7][3] = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {0, 0, 0}};

float calc_spd;
float start_time,end_time;
float altitude_z;

struct vec acc_vec_list[total_search_num];
struct vec neig_pos_arr[neig_num_d];
struct vec obs_pos_arr[obs_num_d];
struct vec search_vel_arr[total_search_num];

//vectors for calculations:
struct vec migration_vel; //migration velocity
struct vec pos_vec; //position
struct vec vel_vec; //velocity
struct vec search_pos_vec; //predicted position
struct vec search_vel_vec; //predicted velocity
struct vec search_head_vec; //predicted heading
struct vec neig_rob_sub_vec; //relative neighbor position
struct vec obs_rob_sub_vec; //relative obstacle position
struct vec obs_uni_vec; //relative unit direction vector of an obstacle
struct vec search_acc_vec; //predicted acceleration

float search_heur_arr[total_search_num]; //array of heuristic costs of each node
float neig_dist_arr[neig_num_d]; //array of distances to neighbors
float obs_dist_arr[obs_num_d]; //array of distances to obstacles
float neig_dist; //relative distance to a neighbor
float obs_dist; //relative distance to an obstacle
float heur; //heuristic cost
float xi; //approach term
float prevHeig; //previous altitude for altitude control

//static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdFlightState;


//struct for tracking ids and positions of quadcopters
typedef struct {
  uint8_t id;
  float x;
  float y;
  float z;
  } _coords;

//function for setting calculated velocity as a setpoint
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float vz)
{
  
  float yawRate = 0.0; //Since the robot model is holonomic, yaw rate is always zero

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawRate;
  
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;
  
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity.z = vz;
  setpoint->velocity_body = false;
}

//function that sorts two dimensional array based on the first dimension
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

//function that sorts three dimensional array based on the first dimension
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

  static setpoint_t setpoint;
  static uint8_t flightstate; //parameter for changing the flight state of the quadcopter
  flightstate = 0; //0 -> wait on the ground
  
  //a parameter is created for changing the value of the flight state to control experiment via Crazyswarm platform
  PARAM_GROUP_START(fmodes)
  PARAM_ADD_CORE(PARAM_UINT8, ufs, &flightstate)
  PARAM_GROUP_STOP(fmodes)
  paramIdFlightState = paramGetVarId("fmodes", "ufs");
  
  //collects positions of the quadcopter
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff); //id of the quadcopter

  _coords self_coords;
  self_coords.id = my_id;
  
  //migraiton velocity:
  migration_vel.x = 0.0;
  migration_vel.y = migration_speed;
  migration_vel.z = 0.0;
  
  //acceleration set where a_alpha is taken as max. acceleration:
  for (int i = 0; i < total_search_num; ++i) {
    acc_vec_list[i].x = acc_max*acc_list[i][0];
    acc_vec_list[i].y = acc_max*acc_list[i][1];     
    acc_vec_list[i].z = acc_max*acc_list[i][2];
  }
  
  //initial velocity of the quadcopter
  vel_vec.x = 0.0;
  vel_vec.y = 0.0;
  vel_vec.z = 0.0;

  paramSetInt(paramIdResetKalman, 1);
  //paramSetInt(paramIdCommanderEnHighLevel, 1);

  while(1) {

    //if quadcopter is on the ground, it waits until flight state changes
    if (flightstate == 0){
      flightstate = paramGetInt(paramIdFlightState); //checks value of the flight state
      vTaskDelay(M2T(100));
    }

    //if flight state is 1, take-off
    if (flightstate == 1){
      vTaskDelay(M2T(1000));
      crtpCommanderHighLevelTakeoff(take_off_height, 2.5);
      vTaskDelay(M2T(3000));
      flightstate = 2;
    }

    //if take of is completed, change the flight state to flocking
    if ((flightstate == 2) && crtpCommanderHighLevelIsTrajectoryFinished()){
        crtpCommanderHighLevelGoTo(rng_pos_list[my_id-1][0], rng_pos_list[my_id-1][1], rng_pos_list[my_id-1][2], 0.0, 2.0, false);
        vTaskDelay(M2T(3000));
        flightstate = 3;
    }

    //if flight state is 3, start flocking
    if (flightstate == 3){

      //gets position of the quadcopter
      self_coords.x = logGetFloat(idX);
      self_coords.y = logGetFloat(idY);
      self_coords.z = logGetFloat(idZ);

      //updates position of the quadcopter
      pos_vec.x = self_coords.x;
      pos_vec.y = self_coords.y;
      pos_vec.z = self_coords.z;

      prevHeig = pos_vec.z;
       
      //collects positions of other quadcopters
      for (int i = 0; i < neig_num; ++i) {
        peerLocalizationOtherPosition_t const *otherPos = peerLocalizationGetPositionByIdx(i);
        if (otherPos == NULL){continue;}
        uint8_t peer_id = otherPos->id;
        if (peer_id == 0){continue;}
        neig_alive[peer_id - 1] = true;
        // Store received coordinates
        neig_X[peer_id - 1] = otherPos->pos.x;
        neig_Y[peer_id - 1] = otherPos->pos.y;
        neig_Z[peer_id - 1] = otherPos->pos.z;
      }

      //updates positions of other quadcopters
      int j = 0;
      for (int i = 0; i < quad_num; ++i) {
        if ((i+1) != my_id) {
          neig_pos_arr[j].x = neig_X[i];
          neig_pos_arr[j].y = neig_Y[i];
          neig_pos_arr[j].z = neig_Z[i];
          neig_alive_arr[j] = neig_alive[i];
          j = j + 1;
        }
      }

      //updates obstacle positions
      for (int i = 0; i < obs_num; ++i) {
        obs_pos_arr[i].x = obs_X[i];
        obs_pos_arr[i].y = obs_Y[i];
        obs_pos_arr[i].z = pos_vec.z;
      }
      
      //calculates relative positions and distances to the other quadcopters
      for (int i = 0; i < neig_num; ++i) {
        neig_rob_sub_vec = vsub(neig_pos_arr[i],pos_vec);
        neig_dist_arr[i] = vmag(neig_rob_sub_vec); 
      }

      insertion_sort_multi_neig(neig_dist_arr, neig_num, neig_pos_arr, neig_alive_arr); //sorts positions based on relative distances
      
      //calculates predicted positions of others considering they will move at migration velocity at the next time step
      for (int i = 0; i < neig_num; ++i) {
        neig_pos_arr[i].y = neig_pos_arr[i].y + dt*migration_speed; //calculate the predicted position of the neighbor for v^m in the y-axis direction
      }

      //calculates relative positions and distances to the obstacles
      for (int i = 0; i < obs_num; ++i) {
          obs_rob_sub_vec = vsub(obs_pos_arr[i],pos_vec);
          obs_dist_arr[i] = vmag(obs_rob_sub_vec);
      }

      insertion_sort_multi_two(obs_dist_arr, obs_num, obs_pos_arr); //sorts positions based on relative distances
      
      //Predictive Search Method using single step predictions:
      for (int k = 0; k < total_search_num; ++k) {

        heur = 0;

        search_acc_vec = acc_vec_list[k]; //predicted acceletion
        search_vel_vec = vadd(vel_vec, vscl(dt,search_acc_vec)); //calculates predicted velocity, without speed constraints

        calc_spd = clamp(vmag(search_vel_vec),spd_min,spd_max); //calculates predicted speed, clamping between min. and max. speed

        // if speed is 0, then take predicted heading as it directs toward migraiton direction
        if ((search_vel_vec.x == (float)0) && (search_vel_vec.y == (float)0) && (search_vel_vec.z == (float)0)) {
          search_head_vec.x = 0.0;
          search_head_vec.y = 1.0;
          search_head_vec.z = 0.0;
        }
        else {
          search_head_vec = vdiv(search_vel_vec,vmag(search_vel_vec)); //calculates predicted heading
        }

        search_vel_vec = vscl(calc_spd,search_head_vec); //calculates predicted velocity

        search_pos_vec = vadd(pos_vec,vscl(dt,search_vel_vec)); //calculates predicted position
        
        //calculates obstacle avoidance heuristic:
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
        

        //calculates inter-robot heuristic:
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
        
        //calculates migration heuristic:
        float heur_dir = kDir*(1 - vdot(search_vel_vec,migration_vel)/(calc_spd*migration_speed));
        float heur_spd = kSpd*fabsf(migration_speed - calc_spd)/migration_speed;
        
        //calculates altitude heuristic:
        float heur_height = 0;
        float heigthErr = fabsf(search_pos_vec.z - des_height);
        if (heigthErr > hMargin){
          heur_height = kHeig*zp5*powf(1/heigthErr - 1/hMargin,2);
        }           
        
        float heur = heur_obs + heur_neig + heur_dir + heur_spd + heur_height; //total heuristic of a node
        
        //appends predicted velocity and heuristic cost to arrays
        search_vel_arr[k] = search_vel_vec;
        search_heur_arr[k] = heur;
      }

      insertion_sort_multi_two(search_heur_arr, total_search_num, search_vel_arr); //sort predicted velocities based on heuristic costs
      
      //collect the velocity with smallest cost as the next velocity
      float vx = search_vel_arr[0].x;
      float vy = search_vel_arr[0].y;
      float vz = search_vel_arr[0].z;
      
      //if velocity in z-axis is 0, then altitude controller maintains altitude of the quadcopter at the same:
      if (fabsf(vz) < vZeroThresh){
        vz = kAlt*(prevHeig - pos_vec.z); //controls altitude with a proportional controller since the velocity controller of Crazyflie cannot maintain zero speed in z-direction
      }
      else{
        prevHeig = pos_vec.z;
      }
      
      vel_vec = mkvec(vx,vy,vz); //update the velocity of the quadcopter
      
      //sets velocity input:
      setHoverSetpoint(&setpoint, vx, vy, vz);
      commanderSetSetpoint(&setpoint, 3);

      //end_time = usecTimestamp() / 1e3;
      //float computation_time = (end_time - start_time);
      //DEBUG_PRINT("Execution time: %d \n",computation_time);

      //uint8_t remaining_time = (dt*1000 - computation_time);
      //vTaskDelay(M2T(remaining_time));

      vTaskDelay(M2T(dt_msec)); //sleeps for dt, assuming computation time is negligible

      //DEBUG_PRINT("Velocity input is: %f %f %f \n", (double)vx, (double)vy, (double)vz);
      //DEBUG_PRINT("Heading input is: %f \n", (double)self_head);

      flightstate = paramGetInt(paramIdFlightState); //checks value of the flight state
    }
 
    //if flight state is 4, the quadcopter lands
    if (flightstate == 4){
      commanderRelaxPriority();
      crtpCommanderHighLevelLand(0.03, 3.0); 
      vTaskDelay(M2T(4000));
      return;
    }
  }
}

