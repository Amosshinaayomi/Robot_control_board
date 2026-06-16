#pragma once
#include "pose.h"
typedef struct {
  float x, y;
} waypoint;


volatile float target_linear_vel = 0.0f;
volatile float target_angular_vel = 0.0f;


// Tuning parameters
#define LOOKAHEAD_DIST      0.3f    // meters
#define WP_TOLERANCE        0.1f    // meters
#define MAX_LINEAR_VEL      0.4f    // m/s
#define MAX_ANGULAR_VEL     4.0f    // rad/s
#define VEL_REDUCTION_OMEGA 1.0f    


static waypoint path[] = {
    {0.4f, 0.0f},
    {0.4f, 0.4f},
    {0.0f, 1.0f},
    {0.0f, 0.0f}  
};

static uint16_t num_path_wps = sizeof(path) / sizeof(path[0]);
static int current_wp_index = 0;

void purePursuitUpdate(pose_packet_t &pose) {
  float rx = pose.x;
  float ry = pose.y;
  float theta = pose.theta;
  

  if(current_wp_index >= num_path_wps) {
    Serial.printf("waypoint concluded");
    target_linear_vel = 0.0f;
    target_angular_vel = 0.0f;
    return;
  }

    // Current target waypoint
    float wx = path[current_wp_index].x;
    float wy = path[current_wp_index].y;

    // 1. Check if reached waypoint (Euclidean distance)
    float dx_w = wx - rx;
    float dy_w = wy - ry;
    float dist_to_wp = sqrtf(dx_w * dx_w + dy_w * dy_w);

    if(dist_to_wp < WP_TOLERANCE) {
      current_wp_index++;
      if(current_wp_index >= num_path_wps) {
        target_linear_vel = 0.0f;
        target_angular_vel = 0.0f;
        return;
      }
    }

    // Recalculate for new waypoint 
    wx = path[current_wp_index].x;
    wy = path[current_wp_index].y;

    dx_w = wx - rx;
    dy_w = wy - ry;


    float cos_t = cosf(theta);
    float sin_t = sinf(theta);
    float local_x = cos_t * dx_w + sin_t * dy_w;
    float local_y = -sin_t * dx_w + cos_t * dy_w;

    // 3. Pure pursuit: use fixed lookahead distance L
    //    If the waypoint is closer than L, just chase it directly
    float L = LOOKAHEAD_DIST;
    float y_L = local_y;   // lateral offset in robot frame
    float omega_cmd;


      // Avoid division by zero (if waypoint exactly ahead, omega = 0)
    if (fabsf(y_L) < 0.001f) {
        omega_cmd = 0.0f;
    } else {
        omega_cmd = 2.0f * MAX_LINEAR_VEL * y_L / (L * L);
    }

    // 4. Compute linear velocity – slow down for sharp turns and final approach
    float v_cmd = MAX_LINEAR_VEL;


    // Reduce speed if angular command is high
    if (fabsf(omega_cmd) > VEL_REDUCTION_OMEGA) {
        v_cmd = MAX_LINEAR_VEL * VEL_REDUCTION_OMEGA / fabsf(omega_cmd);
    }
    // Slow down when near the waypoint (soft landing)
    if (dist_to_wp < 0.5f) {
        float approach_scale = dist_to_wp / 0.5f;  // 0..1
        if (approach_scale < 0.2f) approach_scale = 0.2f; // don't go below 20%
        v_cmd *= approach_scale;
    }

    // 5. Clamp outputs
    if (v_cmd > MAX_LINEAR_VEL) v_cmd = MAX_LINEAR_VEL;
    if (v_cmd < -MAX_LINEAR_VEL) v_cmd = -MAX_LINEAR_VEL;

    if (omega_cmd > MAX_ANGULAR_VEL) omega_cmd = MAX_ANGULAR_VEL;
    if (omega_cmd < -MAX_ANGULAR_VEL) omega_cmd = -MAX_ANGULAR_VEL;

    // 6. Update target velocities for motionControllerTask
    target_linear_vel = v_cmd;
    target_angular_vel = omega_cmd;
} 