/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotMap.h"

class Indexer {
 public:
  Indexer();
  void Spin(double, double);
  void Spin(double);
  void switchPushBall();
  void setPushBall(positionStatus);
  void feedBall(double);
  void Divet(double, double, double);
  void moveIndexFixedPos(double);
  bool checkLimitSwitch();
  int getIndexerPosition();
  double getFeederRPM();
  double getCurrent();
  void Printer();
  void dashboardPrinter();

  double divetTime = 0;
  double fixedPosTime = 0;
  int indexNum = 0;

 private:
  WPI_TalonSRX index{indexerMotorID};
  WPI_TalonSRX feeder{feederMotorID};

  frc::DoubleSolenoid pneumaticPusher{PCMID, pusherKForward, pusherKReverse};

  double realTime = 0;

  int indexPosList[3] = {100, 1665, 3000}; //will eventually be populated with the encoder values needed to rotate indexer to proper positions

  //frc::DigitalInput indexSwitch{indexSwitchID};
};
