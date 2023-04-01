// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  // Setup auto chooser. Modified stock code.
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameMobility, kAutoNameMobility);
  m_chooser.AddOption(kAutoNameBalance, kAutoNameBalance);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Motor setup
  m_right2.Follow(m_right1);
  m_left2.Follow(m_left1);

  m_left1.SetInverted(true);
  //m_left2.SetInverted(false);
  m_right1.SetInverted(false);
  //m_right2.SetInverted(false);

  // Initially set all pneumatics to false.
  solonoid_arms.Set(false);
  solonoid_gripper.Set(false);
  solonoid_brakes.Set(false);

  // Zero Encoders (maybe adding all of them is a little hopeful)
  e_extender.SetPosition(0);
  e_left1.SetPosition(0);
  e_left2.SetPosition(0);
  e_right1.SetPosition(0);
  e_right2.SetPosition(0);

  // This configures the REVPH for using the Digital sensor.
  // Not needed every run, but if we have to switch to a new/backup it is needed.
  compressor_main.EnableDigital();
  s_IMU.Calibrate();
#ifdef TEST_PID
  // Setup PID arguments for extender
  c_extender.SetFeedbackDevice(e_extender);
  c_extender.SetP(EXTENDER_P);
  c_extender.SetI(EXTENDER_I);
  c_extender.SetD(EXTENDER_D);
  c_extender.SetIZone(EXTENDER_IZ);
  c_extender.SetFF(EXTENDER_FF);
  c_extender.SetOutputRange(EXTENDER_MIN_OUT, EXTENDER_MAX_OUT);
  e_extender.SetPositionConversionFactor(EXTENDER_CONVERSION);
#endif
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  // Mostly using this to update logging and UI.
	frc::SmartDashboard::PutBoolean("Extender Upper Limit", di_extender_upper.Get());
	frc::SmartDashboard::PutBoolean("Extender Lower Limit", di_extender_lower.Get());
	frc::SmartDashboard::PutBoolean("Extender at drop distance", di_extender_drop.Get());
	frc::SmartDashboard::PutNumber("Extender Rotation Count", e_extender.GetPosition());
  frc::SmartDashboard::PutBoolean("Extender PID Controller", b_vel_mode);
	frc::SmartDashboard::PutNumber("IMU Pitch", s_IMU.GetGyroAngleX().value());
	frc::SmartDashboard::PutNumber("Left Y", controller_driver->GetLeftY());
	frc::SmartDashboard::PutNumber("Right X", controller_driver->GetRightX());
  
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  
  #ifdef TEST_BALANCE
  d_pitch = s_IMU.GetAngle();
  d_initial_pitch = s_IMU.GetAngle();
  #endif
  
  // Start timer for auto
  game_timer.Reset();
  game_timer.Start();
  fmt::print("Auto selected: {}\n", m_autoSelected);

}

void Robot::AutonomousPeriodic() {
  /////////////////////////////////////////////////////////////////////////////
  // Mobility Only Auto
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameMobility) {
    // Back up for 5 seconds at 1/4 speed. Should get us just outside of the community.
    // TODO: Add code to drop the cube on the highest node.
    if (game_timer.Get() > 1_s && game_timer.Get() < 6_s){
      if (!(m_right1.Get() > 0.130 && m_right1.Get() < 0.120 )) {
        m_left1.Set(-0.25);
        m_right1.Set(0.25);
      }
    } 
    // Set speed to zero and hold position.
    if (game_timer.Get() > 6_s && game_timer.Get() < 6.5_s){
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Balancing Auto
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameBalance) {
    // Go over(?) the charge station. -1/4 speed for 5 seconds.
    if ( game_timer.Get() < 2.2_s){
      m_left1.Set(-0.30);
      m_right1.Set(0.30);
    }
    // Go slow over and down the charge station.
    if (game_timer.Get() > 2.2_s && game_timer.Get() < 7_s){
      m_left1.Set(-.10);
      m_right1.Set(0.10);
    } 
    // Start back up the charge station at 1/4 speed for 1.5 seconds
    if (game_timer.Get() > 7_s && game_timer.Get() < 8.5_s){
      m_left1.Set(0.25);
      m_right1.Set(-0.25);
    }
    // Stop moving
    if (game_timer.Get() > 8.5_s && game_timer.Get() < 9_s){
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }
    #ifdef TEST_BALANCE
    // Start autobalance process
    // Assumes we are not on a level surface.
    if (game_timer.Get() > 9_s && game_timer.Get() < 15_s) {
      d_hpitch = d_pitch;
      d_pitch = s_IMU.GetGyroAngleX();
        
      if ((d_pitch > DEADBAND_BALANCE or d_pitch < -DEADBAND_BALANCE) and game_timer.Get() >= t_pause_time) {
        // if pitch direction changes, shorten drive time. => go slower instead of pause
        if ((d_pitch < -DEADBAND_BALANCE and d_hpitch > DEADBAND_BALANCE) or (d_pitch > DEADBAND_BALANCE and d_hpitch < -DEADBAND_BALANCE)) {
          d_drive_speed *= NERF_AUTO;
          t_pause_time = game_timer.Get() + 1_s;
        }
        // Do we need to take into account angle delta in our calculations?
        // Maybe use the derivitive of this to slow our speed?
            // If there is a high angular velocity, maybe we should slow down and let it settle?
        if (game_timer.Get() >= t_pause_time) {
          if (d_pitch > DEADBAND_BALANCE){
            // Move Forward
            m_left1.Set(d_drive_speed);
            m_right1.Set(-d_drive_speed);
          } else if (d_pitch < -DEADBAND_BALANCE){
            //Move Backward
            m_left1.Set(-d_drive_speed);
            m_right1.Set(d_drive_speed);
          }
        }
      }


      // Do this when balanced
      /*
      im_mode = rev::CANSparkMax::IdleMode::kBrake;
      m_left1.SetIdleMode(im_mode);
      m_left2.SetIdleMode(im_mode);
      m_right1.SetIdleMode(im_mode);
      m_right2.SetIdleMode(im_mode);
      solonoid_brakes.Set(true);
      
      
      */
   
    }
    #endif
  }
}

void Robot::TeleopInit() {
    im_mode = rev::CANSparkMax::IdleMode::kCoast;
    m_left1.SetIdleMode(im_mode);
    m_left2.SetIdleMode(im_mode);
    m_right1.SetIdleMode(im_mode);
    m_right2.SetIdleMode(im_mode);
    solonoid_brakes.Set(false);
}

void Robot::TeleopPeriodic() {
  // At some point we should try moving to PID controllers for movement.
  // This would let us set a target velocity based off of the controller
  // position instead of setting a percent output. Would also allow switching
  // to distance tracking so that we can move exact distances to tie into
  // camera inputs or other sensors. 
  // Get analog controls 
  double d_driver_speed = controller_driver->GetLeftY();
  double d_driver_turn = controller_driver->GetRightX();
  double d_arms_extend = -controller_arms->GetLeftX();
  
  #ifdef TEST_MINOR_TURN
  double d_turn_minorl = controller_driver->GetLeftTriggerAxis();
  double d_turn_minorr = controller_driver->GetRightTriggerAxis();
  #endif

  // Apply deadband to analog controls
  if (abs(d_arms_extend) <= DEADBAND_CONTROL) {
    d_arms_extend = 0.0;
  }

  // Apply movement macros
  d_driver_speed = d_driver_speed * NERF_SPEED;
  d_driver_turn = d_driver_turn * NERF_TURN;
  d_arms_extend *= NERF_EXTEND;
  #ifdef TEST_MINOR_TURN
  d_turn_minorl *= 0.5;
  d_turn_minorr *= 0.5;
  #endif
  

  // Set motor and pneumatic outputs from control inputs
  
  // Command motors
  d_drive.ArcadeDrive(d_driver_speed, d_driver_turn - d_turn_minorl + d_turn_minorr);


// Command pneumatic solonoids
  if (controller_arms->GetLeftBumperPressed()) {
    #ifdef TEST_RETRACT_SAFE
    // Retract arms. Remember, threading on the rod is such that
    // more negative is farther out. 0 is initial position.
    if (e_extender.GetPosition() < EXTENDER_PICK) {
      c_extender.SetReference(EXTENDER_HOME, rev::CANSparkMax::ControlType::kPosition);
    }
    #endif
    solonoid_arms.Toggle();
  }
  if (controller_arms->GetRightBumperPressed()) {
    solonoid_gripper.Toggle();
  }
  if (controller_driver->GetLeftBumperPressed()) {
    solonoid_brakes.Toggle();
    #ifdef TEST_BRAKING
    if (im_mode == rev::CANSparkMax::IdleMode::kCoast) {
      im_mode = rev::CANSparkMax::IdleMode::kBrake;
    }
    if (im_mode == rev::CANSparkMax::IdleMode::kBrake) {
      im_mode = rev::CANSparkMax::IdleMode::kCoast;
    }
    m_left1.SetIdleMode(im_mode);
    m_left2.SetIdleMode(im_mode);
    m_right1.SetIdleMode(im_mode);
    m_right2.SetIdleMode(im_mode);
    #endif
  }
  #ifdef TEST_POOFER
  if (controller_arms->GetLeftStickButtonPressed()) {
    solonoid_poofer.Toggle();
  }
  #endif

  #ifdef TEST_PID
  if (controller_arms->GetStartButtonPressed()) {
    b_vel_mode = !b_vel_mode;
  }
  if (controller_arms->GetRightStickButtonPressed()){
    e_extender.SetPosition(0);
  }
  #endif

  // Apply limit switches to extender motor
  if (d_arms_extend > 0.0 && !di_extender_upper.Get()) {
    d_arms_extend = 0.0;
  }
  if (d_arms_extend < 0.0 && !di_extender_lower.Get()) {
    d_arms_extend = 0.0;
  }

#ifdef TEST_PID
  // Command motor
  // Need to figure out a way to do both of these at the same time without toggling states.
  if (!b_vel_mode) {
    c_extender.SetReference(d_arms_extend, rev::CANSparkMax::ControlType::kDutyCycle);
    //m_extender.Set( d_controller_1_x);
  } else {
    // Home Position
    if (controller_arms->GetAButtonPressed()){
      c_extender.SetReference(EXTENDER_HOME, rev::CANSparkMax::ControlType::kPosition);
    }

    // Pickup position
    if (controller_arms->GetBButtonPressed()) {
      c_extender.SetReference(EXTENDER_PICK, rev::CANSparkMax::ControlType::kPosition);
    }

    // Mid position
    if (controller_arms->GetXButtonPressed()) {
      c_extender.SetReference(EXTENDER_MID, rev::CANSparkMax::ControlType::kPosition);
    }

    // High Position
    if (controller_arms->GetYButtonPressed()) {
      c_extender.SetReference(EXTENDER_HIGH, rev::CANSparkMax::ControlType::kPosition);
    }
  }
  #else
    m_extender.Set(d_arms_extend);
  #endif

  // Toggle for brake/coast mode for motors

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
