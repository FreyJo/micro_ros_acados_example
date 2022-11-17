// Copyright (c) 2020 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <std_msgs/msg/float64.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>


// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_pendulum_ode.h"

#define NX     PENDULUM_ODE_NX
#define NZ     PENDULUM_ODE_NZ
#define NU     PENDULUM_ODE_NU
#define NP     PENDULUM_ODE_NP



// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
std_msgs__msg__Float64 pub_msg;
std_msgs__msg__Float64 sub_msg;

double x_current[NX];

/***************************** CALLBACKS ***********************************/

void my_subscriber_callback(const void * msgin, void * capsule_in)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;

  sim_solver_capsule *capsule = (sim_solver_capsule *) capsule_in;
  sim_config *acados_sim_config = pendulum_ode_acados_get_sim_config(capsule);
  sim_in *acados_sim_in = pendulum_ode_acados_get_sim_in(capsule);
  sim_out *acados_sim_out = pendulum_ode_acados_get_sim_out(capsule);
  void *acados_sim_dims = pendulum_ode_acados_get_sim_dims(capsule);
  // if (msg == NULL) {
  //   printf("Callback: msg NULL\n");
  // } else {
  //   printf("Callback: I heard: %s\n", msg->data.data);
  // }
  // TODO:

  sim_in_set(acados_sim_config, acados_sim_dims,
      acados_sim_in, "x", x_current);
  status = pendulum_ode_acados_sim_solve(capsule);

  if (status != ACADOS_SUCCESS)
  {
      printf("acados_solve() failed with status %d.\n", status);
  }

  sim_out_get(acados_sim_config, acados_sim_dims,
          acados_sim_out, "x", x_current);

  printf("\nx_current, %d\n", ii);
  for (int jj = 0; jj < NX; jj++)
  {
      printf("%e\n", x_current[jj]);
  }

  pub_msg.data = x_current[1];
  rc = rcl_publish(&my_pub, &pub_msg, NULL);
}

/******************** MAIN PROGRAM ****************************************/
int main(int argc, const char * argv[])
{

  int status = 0;

  sim_solver_capsule *capsule = pendulum_ode_acados_sim_solver_create_capsule();
  status = pendulum_ode_acados_sim_create(capsule);

  if (status)
  {
      printf("acados_create() returned status %d. Exiting.\n", status);
      exit(1);
  }

  sim_config *acados_sim_config = pendulum_ode_acados_get_sim_config(capsule);
  sim_in *acados_sim_in = pendulum_ode_acados_get_sim_in(capsule);
  sim_out *acados_sim_out = pendulum_ode_acados_get_sim_out(capsule);
  void *acados_sim_dims = pendulum_ode_acados_get_sim_dims(capsule);

  // initial condition
  x_current[0] = 0;
  x_current[1] = 3.141592653589793;
  x_current[2] = 0;
  x_current[3] = 0;

  sim_in_set(acados_sim_config, acados_sim_dims,
      acados_sim_in, "x", x_current);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "pendulum_ode_sim", "", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

  // create a publisher to publish topic 'next_state' with type std_msg::msg::Float64
  // my_pub is global, so that the timer callback can access this publisher.
  const char * topic_name = "next_state";
  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64);

  rc = rclc_publisher_init_default(
    &my_pub,
    &my_node,
    my_type_support,
    topic_name);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name);
    return -1;
  }

  // assign message to publisher
  std_msgs__msg__Float64__init(&pub_msg);

  // create subscription
  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(
    &my_sub,
    &my_node,
    my_type_support,
    topic_name);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }

  // one Float64 message for subscriber
  std_msgs__msg__Float64__init(&sub_msg);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // total number of handles = #subscriptions
  unsigned int num_handles = 1 + 1;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  // add subscription to executor
  rc = rclc_executor_add_subscription_with_context(
    &executor, &my_sub, &sub_msg, &my_subscriber_callback, (void *) capsule,
    ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }

  // Optional prepare for avoiding allocations during spin
  rclc_executor_prepare(&executor);

  rclc_executor_spin(&executor);

  // clean up
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);

  std_msgs__msg__Float64__fini(&pub_msg);
  std_msgs__msg__Float64__fini(&sub_msg);

  // free solver
  status = pendulum_ode_acados_sim_free(capsule);
  if (status) {
      printf("pendulum_ode_acados_sim_free() returned status %d. \n", status);
  }
  pendulum_ode_acados_sim_solver_free_capsule(capsule);


  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}