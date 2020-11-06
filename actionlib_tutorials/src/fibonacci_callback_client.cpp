#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib_tutorials/FibonacciAction.h"

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

// Chamada uma vez, quando o goal é concluído
void doneCB(const actionlib::SimpleClientGoalState& state, 
            const FibonacciResultConstPtr& result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

// Chamada uma vez, quando o goal torna-se ativo
void activeCB() {
  ROS_INFO("Goal just went active");
}

// Chamada a cada vez que o feedback é recebido
void feedbackCB(const FibonacciFeedbackConstPtr& feedback) {
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fibonacci_callback");

  // Cria o Client da Action
  Client ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal");

  // Envia o goal
  FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);

  ros::spin();

  return 0;
}
