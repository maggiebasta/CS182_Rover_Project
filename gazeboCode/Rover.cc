#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ncurses.h>

#include <iostream>
#include <fstream>

#include <tuple>
#include <vector>

typedef std::vector <std::pair<int,int>> pair_list;

// choose trajectory based off of algorithm under test

//A*
pair_list trajectory = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, -1}, {5, -2}, {6, -3}, {7, -3}, {8, -3}, {9, -2}, {10, -1}, {10, 0}, {10, 1}, {10, 2}, {11, 3}, {11, 4}, {12, 5}, {13, 5}, {14, 5}, {15, 6}, {16, 7}, {17, 7}, {18, 7}, {19, 7}, {20, 7}, {21, 7}, {22, 8}, {23, 9}, {24, 10}, {25, 11}, {26, 11}, {27, 11}, {28, 11}, {29, 12}, {30, 13}, {31, 13}, {32, 14}, {33, 15}, {34, 16}, {35, 16}, {36, 16}, {37, 16}, {38, 16}, {39, 16}, {40, 16}, {41, 17}, {42, 17}, {43, 18}, {44, 19}, {44, 20}, {44, 21}, {44, 22}, {45, 23}, {46, 24}, {47, 24}, {48, 24}, {49, 24}, {50, 25}, {51, 26}, {52, 27}, {53, 28}, {54, 29}, {55, 30}, {56, 31}, {57, 32}, {58, 33}, {57, 34}, {57, 35}, {57, 36}, {57, 37}, {58, 38}, {59, 39}, {60, 40}, {60, 41}, {59, 42}, {58, 43}, {58, 44}, {59, 45}, {59, 46}, {59, 47}, {59, 48}, {60, 49}, {61, 50}, {62, 51}, {62, 52}, {62, 53}, {62, 54}, {62, 55}, {63, 56}, {63, 57}, {63, 58}, {64, 59}, {64, 60}, {63, 61}, {62, 62}};

// BFS
// pair_list trajectory = {{0,0},{1,1},{2,2},{3,3},{4,4},{5,5},{6,6},{7,7},{8,8},{9,9},{10,10},{11,11},{12,12},{13,13},{14,14},{15,15},{16,16},{17,17},{18,18},{19,19},{20,20},{21,21},{22,22},{23,23},{24,24},{25,25},{26,26},{27,27},{28,28},{29,29},{30,30},{31,31},{32,32},{33,33},{34,34},{35,35},{36,36},{37,37},{38,38},{39,39},{40,40},{41,41},{42,42},{43,43},{44,44},{45,45},{46,46},{47,47},{48,48},{49,49},{50,50},{51,51},{52,52},{53,53},{54,54},{55,55},{56,56},{57,57},{58,58},{59,59},{60,60},{61,61},{62,62}};

const double trajSize = trajectory.size();

// how close to trajectory point rover moves to before moving on to next point in trajectory
const double distanceThreshold = .2;


// CONSTANTS
const double pi = 3.14159;
const double legAirSpeed = 7.0;
// 20 degrees before home pos
const double landingAngle = 0.349;
const double legGroundSpeed = legAirSpeed * (2*landingAngle/(2*pi - 2*landingAngle));

const double k_p = 5.0;


// GLOBAL VARIABLES
int state = 1;
int legStates[6] = {0,0,0,0,0,0};
double timeNow;

const double maxTurningThreshold = 0.9;
double turningThreshold = maxTurningThreshold;

int trajIndex = 0;

// variable for reading from gamepad, set usingGamepad = true if you want to control with gamecontroller
// USING LOGITECH F310 Gamepad
std::ifstream myfile;
int gameData;
const bool MANUAL_CONTROL = false;


void moveForward(gazebo::physics::JointPtr legs[6], double legAngles[6], int legStates[6]){
    // MOVE FORWARD
    
    // right side in the air
    if (state == 1){

        if (legAngles[0] > pi){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legAirSpeed);

        if (legAngles[1] > pi){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legAirSpeed);

        if (legAngles[2] > pi){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legAirSpeed);

        // make left legs go to home position
        if ((legAngles[3] > landingAngle) and (legAngles[3] < 2*pi - 2*landingAngle)){
            legs[3]->SetVelocity(0, 0.0);
            legStates[3] = 1;
        }
        else 
            legs[3]->SetVelocity(0, legAirSpeed);

        if ((legAngles[4] > landingAngle) and (legAngles[4] < 2*pi - 2*landingAngle)){
            legs[4]->SetVelocity(0, 0.0);
            legStates[4] = 1;
        }
        else 
            legs[4]->SetVelocity(0, legAirSpeed);

        if ((legAngles[5] > landingAngle) and (legAngles[5] < 2*pi - 2*landingAngle)){
            legs[5]->SetVelocity(0, 0.0);
            legStates[5] = 1;
        }
        else 
            legs[5]->SetVelocity(0, legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    }
    // move right feet to ground and move left feet to the air
    else if (state == 2){
      
        // right legs finish air
        if (legAngles[0] > 2*pi - landingAngle){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legAirSpeed);

        if (legAngles[1] > 2*pi - landingAngle){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legAirSpeed);

        if (legAngles[2] > 2*pi - landingAngle){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legAirSpeed);

        // left legs finish ground step
        if ((legAngles[3] > landingAngle) and (legAngles[3] < landingAngle * 2)){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[4] > landingAngle) and (legAngles[4] < landingAngle * 2)){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[5] > landingAngle) and (legAngles[5] < landingAngle * 2)){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, legGroundSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 3;
        }
      
    } 
    // move left feet to ground and move right feet to the air
    else if (state == 3){
        // right legs finish ground step
        if ((legAngles[0] > landingAngle) and (legAngles[0] < landingAngle * 2)){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[1] > landingAngle) and (legAngles[1] < landingAngle * 2)){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[2] > landingAngle) and (legAngles[2] < landingAngle * 2)){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legGroundSpeed);

        // left legs finish air
        if (legAngles[3] > 2*pi - landingAngle){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, legAirSpeed);

        if (legAngles[4] > 2*pi - landingAngle){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, legAirSpeed);

        if (legAngles[5] > 2*pi - landingAngle){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    } 
}


void rotateLeft(gazebo::physics::JointPtr legs[6], double legAngles[6], int legStates[6]){
    // COUNTERCLOCKWISE

    // right side in the air
    if (state == 1){

        if (legAngles[0] > pi){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legAirSpeed);

        if (legAngles[1] > pi){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, -legAirSpeed);

        if (legAngles[2] > pi){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legAirSpeed);

        
        // make left legs go to home position
        if ((legAngles[3] > landingAngle) and (legAngles[3] < 2*pi - 2*landingAngle)){
            legs[3]->SetVelocity(0, 0.0);
            legStates[3] = 1;
        }
        else 
            legs[3]->SetVelocity(0, -legAirSpeed);

        if ((legAngles[4] > landingAngle) and (legAngles[4] < 2*pi - 2*landingAngle)){
            legs[4]->SetVelocity(0, 0.0);
            legStates[4] = 1;
        }
        else 
            legs[4]->SetVelocity(0, legAirSpeed);

        if ((legAngles[5] > landingAngle) and (legAngles[5] < 2*pi - 2*landingAngle)){
            legs[5]->SetVelocity(0, 0.0);
            legStates[5] = 1;
        }
        else 
            legs[5]->SetVelocity(0, -legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    }
    // move right feet to ground and move left feet to the air
    else if (state == 2){
      
        // right legs finish air
        if (legAngles[0] > 2*pi - landingAngle){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legAirSpeed);

        if (legAngles[1] < landingAngle){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, -legAirSpeed);

        if (legAngles[2] > 2*pi - landingAngle){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legAirSpeed);

        // left legs finish ground step
        if ((legAngles[3] > 2 * landingAngle) and (legAngles[3] < 2*pi - landingAngle)){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, -legGroundSpeed);

        if ((legAngles[4] > landingAngle) and (legAngles[4] < landingAngle * 2)){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[5] > 2 * landingAngle) and (legAngles[5] < 2*pi - landingAngle)){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, -legGroundSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 3;
        }
      
    } 
    // move left feet to ground and move right feet to the air
    else if (state == 3){
        // right legs finish ground step. Used AND to ensure that when its greater than landing angle, it 
        //had just finished off the ground and not at the start (since the angle is around 6 radians at beginning of 
       // ground movement)
    
        if ((legAngles[0] > landingAngle) and (legAngles[0] < landingAngle * 2)){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[1] > landingAngle * 2) and (legAngles[1] < 2*pi - landingAngle)){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, -legGroundSpeed);

        if ((legAngles[2] > landingAngle) and (legAngles[2] < landingAngle * 2)){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, legGroundSpeed);

        // left legs finish air
        if (legAngles[3] < landingAngle){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, -legAirSpeed);

        if (legAngles[4] > 2*pi - landingAngle){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, legAirSpeed);

        if (legAngles[5] < landingAngle){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, -legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    }
}

void rotateRight(gazebo::physics::JointPtr legs[6], double legAngles[6], int legStates[6]){
    // CLOCKWISE

    // right side in the air
    if (state == 1){

        if (legAngles[0] > pi){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, -legAirSpeed);

        if (legAngles[1] > pi){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legAirSpeed);

        if (legAngles[2] > pi){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, -legAirSpeed);

        
        // make left legs go to home position
        if (legAngles[3] > 2*pi - 2*landingAngle){
            legs[3]->SetVelocity(0, 0.0);
            legStates[3] = 1;
        }
        else 
            legs[3]->SetVelocity(0, legAirSpeed);

        if (legAngles[4] < landingAngle){
            legs[4]->SetVelocity(0, 0.0);
            legStates[4] = 1;
        }
        else 
            legs[4]->SetVelocity(0, -legAirSpeed);

        if (legAngles[5] > 2*pi - 2*landingAngle){
            legs[5]->SetVelocity(0, 0.0);
            legStates[5] = 1;
        }
        else 
            legs[5]->SetVelocity(0, legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    }
    // move right feet to ground and move left feet to the air
    else if (state == 2){
      
        // right legs finish air
        if (legAngles[0] < landingAngle){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, -legAirSpeed);

        if (legAngles[1] > 2*pi - landingAngle){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legAirSpeed);

        if (legAngles[2] < landingAngle){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, -legAirSpeed);

        // left legs finish ground step
        if ((legAngles[3] > landingAngle) and (legAngles[3] < landingAngle * 2)){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[4] > 2 * landingAngle) and (legAngles[4] < 2*pi - landingAngle)){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, -legGroundSpeed);

        if ((legAngles[5] > landingAngle) and (legAngles[5] < landingAngle * 2)){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, legGroundSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 3;
        }
      
    } 
    // move left feet to ground and move right feet to the air
    else if (state == 3){
        // right legs finish ground step. Used AND to ensure that when its greater than landing angle, it 
        //had just finished off the ground and not at the start (since the angle is around 6 radians at beginning of 
       // ground movement)
    
        if ((legAngles[0] > landingAngle * 2) and (legAngles[0] < 2*pi - landingAngle)){
          legs[0]->SetVelocity(0, 0);
          legStates[0] = 1;
        }
        else 
          legs[0]->SetVelocity(0, -legGroundSpeed);

        if ((legAngles[1] > landingAngle) and (legAngles[1] < landingAngle * 2)){
          legs[1]->SetVelocity(0, 0);
          legStates[1] = 1;
        }
        else 
          legs[1]->SetVelocity(0, legGroundSpeed);

        if ((legAngles[2] > landingAngle * 2) and (legAngles[2] < 2*pi - landingAngle)){
          legs[2]->SetVelocity(0, 0);
          legStates[2] = 1;
        }
        else 
          legs[2]->SetVelocity(0, -legGroundSpeed);

        // left legs finish air
        if (legAngles[3] > 2*pi - landingAngle){
          legs[3]->SetVelocity(0, 0);
          legStates[3] = 1;
        }
        else 
          legs[3]->SetVelocity(0, legAirSpeed);

        if (legAngles[4] < landingAngle){
          legs[4]->SetVelocity(0, 0);
          legStates[4] = 1;
        }
        else 
          legs[4]->SetVelocity(0, -legAirSpeed);

        if (legAngles[5] > 2*pi - landingAngle){
          legs[5]->SetVelocity(0, 0);
          legStates[5] = 1;
        }
        else 
          legs[5]->SetVelocity(0, legAirSpeed);
        

        if (legStates[0] and legStates[1] and legStates[2] and legStates[3] and legStates[4] and legStates[5]){
          legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
          state = 2;
        }
    }
}



namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->leftFront = this->model->GetJoint("left_front_wheel_hinge");
      this->leftMiddle = this->model->GetJoint("left_middle_wheel_hinge");
      this->leftBack = this->model->GetJoint("left_back_wheel_hinge");
      this->rightFront = this->model->GetJoint("right_front_wheel_hinge");
      this->rightMiddle = this->model->GetJoint("right_middle_wheel_hinge");
      this->rightBack = this->model->GetJoint("right_back_wheel_hinge");

      // initialize variables
      legs[0] = this->rightFront;
      legs[1] = this->leftMiddle;
      legs[2] = this->rightBack;
      legs[3] = this->leftFront;
      legs[4] = this->rightMiddle;
      legs[5] = this->leftBack;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
    }


    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(1, 0, 0));
      common::Time current_time = this->model->GetWorld()->GetSimTime();
      timeNow = current_time.Double();

      legAngles[0] = fmod(legs[0]->GetAngle(0).Radian(), 2*pi);
      legAngles[1] = fmod(legs[1]->GetAngle(0).Radian(), 2*pi);
      legAngles[2] = fmod(legs[2]->GetAngle(0).Radian(), 2*pi);
      legAngles[3] = fmod(legs[3]->GetAngle(0).Radian(), 2*pi);
      legAngles[4] = fmod(legs[4]->GetAngle(0).Radian(), 2*pi);
      legAngles[5] = fmod(legs[5]->GetAngle(0).Radian(), 2*pi);

      // in case angle is negative convert to positive
      legAngles[0] = legAngles[0] + 2*pi*(legAngles[0] < 0);
      legAngles[1] = legAngles[1] + 2*pi*(legAngles[1] < 0);
      legAngles[2] = legAngles[2] + 2*pi*(legAngles[2] < 0);
      legAngles[3] = legAngles[3] + 2*pi*(legAngles[3] < 0);
      legAngles[4] = legAngles[4] + 2*pi*(legAngles[4] < 0);
      legAngles[5] = legAngles[5] + 2*pi*(legAngles[5] < 0);

      // wait two seconds before allowing rover to move 
      if (timeNow < 2){
          return;
      }
      
      if (MANUAL_CONTROL){
          if (timeNow - saveTimeShorter > .15){
               saveTimeShorter = timeNow;

              // read from controlValues.txt which is being updated by the gameController
              myfile.open ("controlValues.txt");
              myfile >> gameData;
              myfile.close();
          }
      }

      // controller for following a point
      this->model_coor = this->model->GetWorldPose();
      math::Vector3 modelPos(0,0,0);
      modelPos = this->model_coor.pos;

      // get current goal from the trajectory using trajIndex
      double xgoal = trajectory[trajIndex].first;
      double ygoal = trajectory[trajIndex].second;
      double xgoal2 = trajectory[trajIndex + 3].first;
      double ygoal2 = trajectory[trajIndex + 3].second;

      double xdisp, ydisp, xdisp2, ydisp2, xdisp3, ydisp3;
      xdisp = xgoal - modelPos.x;
      ydisp = ygoal - modelPos.y;
      xdisp2 = xgoal2 - modelPos.x;
      ydisp2 = ygoal2 - modelPos.y;

      double dist2Goal = pow(xdisp*xdisp + ydisp*ydisp, .5);
      double dist2Goal2 = pow(xdisp2*xdisp2 + ydisp2*ydisp2, .5);
      double dist2Goal3 = pow(xdisp3*xdisp3 + ydisp3*ydisp3, .5);

      if (dist2Goal < distanceThreshold){
          if ((trajIndex + 6 < trajSize) && (dist2Goal > dist2Goal2)){
            trajIndex += 7;
          }
          else{
            trajIndex += 4;
          }
      }

      if (trajIndex >= trajSize){
          return;
      }

      double actualYaw = this->model_coor.rot.GetYaw();

      double desYaw = atan2(ydisp, xdisp);

      int turningState = 0;
      double yawError;

      

      double turningStateSave = turningState;

      // if usingGamepad control through gameData variable, else do controller to follow trajectory
      if (MANUAL_CONTROL){
          // button X
          if (gameData == 0){
              rotateLeft(legs, legAngles, legStates);
              turningState = 1;
          } 
          // button A
          else if (gameData == 1){
              for (int i = 0; i < 6; i++){
                 legs[i]->SetVelocity(0, 0);
              }
          }
          // button B
          else if (gameData == 2){
              rotateRight(legs, legAngles, legStates);
              turningState = 2;
          }
          else if (gameData == 3){
              moveForward(legs, legAngles, legStates);
              turningState = 3;
          }
      } else{

          // if the desired yaw is to the left of the permitted yaw interval, rotateLeft
          if ((desYaw > actualYaw + turningThreshold) or (actualYaw - desYaw > pi))
          {
            rotateLeft(legs, legAngles, legStates);
            turningState = 1;

            // shrink turning threshold so rover turns to very close to correct angle
            turningThreshold = maxTurningThreshold/4.0;
          } 
          // else if the desired yaw is to the right of the permitted yaw interval, rotateRight
          else if ((desYaw < actualYaw - turningThreshold) or (desYaw - actualYaw > pi))
          {
            rotateRight(legs, legAngles, legStates);
            turningState = 2;

            // shrink turning threshold so rover turns to very close to correct angle
            turningThreshold = maxTurningThreshold/4.0;
          } 
          else {
            moveForward(legs, legAngles, legStates);
            turningState = 3;

            // expand turning threshold so rover doesn't have to keep rotating
            turningThreshold = maxTurningThreshold;
          }

          if (timeNow - saveTime > .5){
               saveTime = timeNow;
               //std::cout << "actualYaw: " << actualYaw << " desYaw: " << desYaw << " TurningState: " << turningState << " xgoal: " << xgoal << " ygoal: " << ygoal << " xdisp: " << xdisp << " ydisp: " << ydisp << " dis2Goal: " << dist2Goal << std::endl;
               //std::cout << "BLAH: " << std::endl;
          }
      }
      
    
      // reset state to 1 if type of turningState has changed
      if (turningStateSave != turningState){
        state = 1;
        legStates[0] = legStates[1] = legStates[2] = legStates[3] = legStates[4] = legStates[5] = 0;
      }      


    }

    // Pointer to the model
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    // Will- Point to joint
    private: physics::JointPtr leftFront;
    private: physics::JointPtr leftMiddle;
    private: physics::JointPtr leftBack;
    private: physics::JointPtr rightFront;
    private: physics::JointPtr rightMiddle;
    private: physics::JointPtr rightBack;


    private: physics::JointPtr legs[6]; 

    // variables for walking
    private: int state;
    private: double legAngles [6];


    private: math::Pose model_coor;
    private: math::Pose goal_coor;
    private: math::Vector3 goalVector;
    private: double saveTime;
    private: double saveTimeShorter;


    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
