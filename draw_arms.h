#ifndef DRAW_ARMS_H_
#define DRAW_ARMS_H_

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "arm_struct.h"

#ifdef PLOT
#include <deque>
#include "pipestream.h"
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

void makeBox1();
void drawBox1();
void makeBox2();
void drawBox2();

void makeArm();
void makeBase();
void makeSensor();
void printSensorPosition();
void drawBase();
void drawP();
void drawSensor();
void drawGripper_edge();
void drawGripper();
void drawGripper_start();
void drawArmCenter();
void drawArmSide();
void Pcontrol();
void inverseKinematics();
void yugan_a();

#endif
