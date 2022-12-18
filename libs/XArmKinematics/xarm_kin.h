/*********************************************************************
 *
 * Provides forward and inverse kinematics for XArm robot
 *
 *********************************************************************/
#pragma once
#include "ofMain.h"
#include "ikfast.h"
#include "URUtils.h"

// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

class XArmKinematics
{
public:
  XArmKinematics(RobotType type);
  XArmKinematics();
  ~XArmKinematics();
  // @param q       The 7 joint values
  // @param T       The 4x4 end effector pose in row-major ordering
  void forward(const double *q, double *T);

  // @param q       The 7 joint values
  // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
  void forward_all(const double *q, double *T1, double *T2, double *T3,
                   double *T4, double *T5, double *T6, double *T7 = nullptr);

  // @param T       The 4x4 end effector pose in row-major ordering
  // @param q_sols  An 8x7 array of doubles returned, all angles should be in [0,2*PI)
  // @param q6_des  An optional parameter which designates what the q6 value should take
  //                in case of an infinite solution on that joint.
  // @return        Number of solutions found (maximum of 8)
  int inverse(const double *T, double *q_sols, double q7_des = 0.0);

private:
  double d1;
  double a2;
  double a3;
  double d4;
  double d5;
  double d6;
};