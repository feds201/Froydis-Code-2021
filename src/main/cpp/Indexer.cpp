/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Indexer.h"

Indexer::Indexer() {
	index.SetInverted(false);
	index.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10); //Relative or Absolute? 
    index.SetSensorPhase(false);
	index.SetSelectedSensorPosition(0);
	index.ConfigFeedbackNotContinuous(true);
	index.SetNeutralMode(Brake);

	index.ConfigPeakOutputForward(INDEXER_SPEED_FINAL_BOT);
	index.ConfigPeakOutputReverse(-INDEXER_SPEED_FINAL_BOT);

	index.ConfigAllowableClosedloopError(0, 10, 10);
	index.ConfigForwardSoftLimitEnable(false);
    index.ConfigReverseSoftLimitEnable(false);

	index.Config_kP(0, index_P, 10);
	index.Config_kI(0, index_I, 10);
	index.Config_kD(0, index_D, 10);
	index.ConfigMaxIntegralAccumulator(0, 100, 10);

	feeder.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	feeder.SetSelectedSensorPosition(0);
}

//Overloaded method - spins indexer using left and right trigger (originally for demo bot)
void Indexer::Spin(double triggerForward, double triggerReverse) { 
	if (triggerForward > 0 && triggerReverse > 0) {
    	index.Set(ControlMode::PercentOutput, 0);
	}
	else {
		index.Set(ControlMode::PercentOutput, triggerForward - triggerReverse);
	}	
}

//Overloaded method - spins indexer just using one speed input
void Indexer::Spin(double speed) {
	index.Set(ControlMode::PercentOutput, speed);
}

//If pneumatic ball pusher is up, make it down, vice versa (not used in sequencing code)
void Indexer::switchPushBall() {
    if (pneumaticPusher.Get() == frc::DoubleSolenoid::Value::kForward) {
		pneumaticPusher.Set(frc::DoubleSolenoid::Value::kReverse);
	}
    else {
		pneumaticPusher.Set(frc::DoubleSolenoid::Value::kForward);
	}
}

//Set pneumatic ball pusher to a specific state (used in sequencing code)
void Indexer::setPushBall(positionStatus position) { //FIX LATER
	if (position == EXTENDED) {
		pneumaticPusher.Set(frc::DoubleSolenoid::Value::kReverse); //UP
	}
	else {
		pneumaticPusher.Set(frc::DoubleSolenoid::Value::kForward); //DOWN
	}
}

void Indexer::feedBall(double speed) {   // Set indexer to home position in between functions
	feeder.Set(ControlMode::PercentOutput, speed);
} 

//Actual name should be Dither - makes indexer unjam by rotating backwards for 0.5 seconds every 2.5 seconds
void Indexer::Divet(double time, double timeTwo, double speed) {

	divetTime += 1; //Number of times loop repeats 
	realTime = (divetTime * riolooptime) / 1000; //Since loop repeats every riolooptime (80ms) this converts it into seconds

	if (realTime < time) {
		Spin(-1 * speed);
	}

	if ((realTime > time) && (realTime < timeTwo)) {
		Spin(speed); //Spin the bot in the opposite direction for these 0.5 seconds
	}

	else if (realTime > timeTwo) {
		Spin(-1 * speed); //Spin bot normally and reset time
		divetTime = 0;
	}
}
void Indexer::startPos(int indexStartPos){
	index.Set(ControlMode::Position, indexStartPos);
	fixedPosTime = 0;
}

void Indexer::moveIndexFixedPos(double indexTime) {
	fixedPosTime += 1;
	realTime = (fixedPosTime * riolooptime) / 1000; //Since loop time repeats every riolooptime (40ms) this converts to seconds
     
	if (realTime < indexTime) {
		index.Set(ControlMode::Position, indexPosList[indexNum]);
    }
	else {
		indexNum++;

        if (indexNum == 4) {
            indexNum = 0;
			fixedPosTime = 0;
        }
		//fixedPosTime = 0;
	}
    index.Set(ControlMode::Position, indexPosList[indexNum]); // is this really needed?
	}

int Indexer::getIndexerPosition() {
	return index.GetSelectedSensorPosition(0);
}

double Indexer::getFeederRPM() {
	return feeder.GetSelectedSensorVelocity(0) * velToRPM_SRX;
}

double Indexer::getCurrent() {
	return index.GetOutputCurrent();
}

void Indexer::Printer() {
	std::cout << "Indexer Position " << getIndexerPosition() << " counts" << std::endl; //Not logged yet
	std::cout << "Feeder RPM " << getFeederRPM() << std::endl; //Not logged yet	
	std::cout << "Ball Pusher State: " << ((pneumaticPusher.Get() == frc::DoubleSolenoid::Value::kForward) ? "kForward (Up)" : "kReverse (Down)") << std::endl; //Not logged yet
	std::cout << "Current: " << getCurrent() << " Amps" << std::endl;
}

void Indexer::dashboardPrinter() {
	frc::SmartDashboard::PutNumber("Indexer Position (counts)", getIndexerPosition());
	frc::SmartDashboard::PutNumber("Indexer Timer", realTime);
	frc::SmartDashboard::PutNumber("Feeder RPM", getFeederRPM());
	//frc::SmartDashboard::PutNumber("Indexer Current (amps)", getCurrent());
	frc::SmartDashboard::PutString("Ball Pusher State",
		 (pneumaticPusher.Get() == frc::DoubleSolenoid::Value::kForward) ? "kForward (Up)" : "kReverse (Down)");
}