// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameMobility, kAutoNameMobility);
  m_chooser.AddOption(kAutoNameBalance, kAutoNameBalance);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_right2.Follow(m_right1);
  m_left2.Follow(m_left1);

  m_left1.SetInverted(false);
  m_left2.SetInverted(false);
  m_right1.SetInverted(false);
  m_right2.SetInverted(false);

  solonoid_arms.Set(false);
  solonoid_gripper.Set(false);
  solonoid_brakes.Set(false);

  // Zero Encoders (maybe adding all of them is a little hopeful)
  e_extender.SetPosition(0);
  e_left1.SetPosition(0);
  e_left2.SetPosition(0);
  e_right1.SetPosition(0);
  e_right2.SetPosition(0);

  compressor_main.EnableDigital();
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
	frc::SmartDashboard::PutBoolean("Extender Upper Limit", di_extender_upper.Get());
	frc::SmartDashboard::PutBoolean("Extender Lower Limit", di_extender_lower.Get());
	frc::SmartDashboard::PutBoolean("Extender at drop distance", di_extender_drop.Get());
	frc::SmartDashboard::PutNumber("Extender Rotation Count", e_extender.GetPosition());
  frc::SmartDashboard::PutBoolean("Extender PID Controller", b_vel_mode);
	frc::SmartDashboard::PutNumber("IMU Pitch", s_IMU.GetPitch());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  
  #ifdef TEST_BALANCE
  d_pitch = s_IMU.GetPitch();
  d_initial_pitch = s_IMU.GetPitch();
  #endif
  game_timer.Reset();
  game_timer.Start();
  fmt::print("Auto selected: {}\n", m_autoSelected);

}

void Robot::AutonomousPeriodic() {
  /////////////////////////////////////////////////////////////////////////////
  // Mobility Only Auto
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameMobility) {
    if (game_timer.Get() > 1_s && game_timer.Get() < 6_s){
      if (!(m_right1.Get() > 0.130 && m_right1.Get() < 0.120 )) {
        m_left1.Set(-0.25);
        m_right1.Set(0.25);
      }
    } 

    if (game_timer.Get() > 6_s && game_timer.Get() < 6.5_s){
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Balancing Auto
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameBalance) {
    // Go out and over(?) the charge station.
    if (game_timer.Get() > 1_s && game_timer.Get() < 6_s){
      if (!(m_right1.Get() > 0.130 && m_right1.Get() < 0.120 )) {
        m_left1.Set(-0.25);
        m_right1.Set(0.25);
      }
    } 
    // Start back up the charge station
    if (game_timer.Get() > 6_s && game_timer.Get() < 6.5_s){
      m_left1.Set(0.25);
      m_right1.Set(-0.25);
    }
    #ifdef TEST_BALANCE
    // Start autobalance process
    // Assumes we are not on a level surface.
    if (game_timer.Get() > 6.5_s && game_timer.Get() < 15_s) {
      d_hpitch = d_pitch;
      d_pitch = s_IMU.GetPitch();
        
      if ((d_pitch > DEADBAND_BALANCE or d_pitch < -DEADBAND_BALANCE) and game_timer.Get() >= t_pause_time) {
        // if pitch direction changes, shorten drive time. => go slower instead of pause
        if ((d_pitch < -DEADBAND_BALANCE and d_hpitch > DEADBAND_BALANCE) or (d_pitch > DEADBAND_BALANCE and d_hpitch < -DEADBAND_BALANCE)) {
          d_drive_speed *= NERF_AUTO;
          t_pause_time = game_timer.Get() + 1_s;
        }
        // Do we need to take into account angle delta in our calculations?
        // 
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
  // Get analog controls 
  double d_driver_speed = -controller_driver->GetRightX();
  double d_driver_turn = -controller_driver->GetLeftY();
  double d_arms_extend = controller_arms->GetRightY();
  
  #ifdef TEST_MINOR_TURN
  double d_turn_minorl = controller_driver->GetLeftTriggerAxis();
  double d_turn_minorr = controller_driver->GetRightTriggerAxis();
  #endif

  // Apply deadband to analog controls
  if (abs(d_driver_speed) <= DEADBAND_CONTROL) {
    d_driver_speed = 0.0;
  }
  if (abs(d_driver_turn) <= DEADBAND_CONTROL) {
    d_driver_turn = 0.0;
  }
  if (abs(d_arms_extend) <= DEADBAND_CONTROL) {
    d_arms_extend = 0.0;
  }
  
  #ifdef TEST_MINOR_TURN
  if (abs(d_turn_minorl) <= DEADBAND_CONTROL) {
    d_turn_minorl = 0.0;
  }
  if (abs(d_turn_minorr) <= DEADBAND_CONTROL) {
    d_turn_minorr = 0.0;
  }
  #endif

  // Scale from 0 - 100
  d_driver_speed = (d_driver_speed) / (1.0 - DEADBAND_CONTROL);
  d_driver_turn = (d_driver_turn) / (1.0 - DEADBAND_CONTROL);
  d_arms_extend = (d_arms_extend) / (1.0 - DEADBAND_CONTROL);
  #ifdef TEST_MINOR_TURN
  d_turn_minorl = (d_turn_minorl) / (1.0 - DEADBAND_CONTROL);
  d_turn_minorr = (d_turn_minorr) / (1.0 - DEADBAND_CONTROL);
  #endif

  // Apply movement macros
  d_driver_speed *= NERF_SPEED;
  d_driver_turn *= NERF_TURN;
  d_arms_extend *= NERF_EXTEND;
  #ifdef TEST_MINOR_TURN
  d_turn_minorl *= NERF_TURN * NERF_TURN;
  d_turn_minorr *= NERF_TURN * NERF_TURN;
  #endif
  
  // Calculate left and right drive controls
  #ifdef TEST_MINOR_TURN
  double d_left = -d_driver_speed + d_driver_turn  - d_turn_minorl;
  double d_right = -d_driver_speed - d_driver_turn - d_turn_minorr;
  #else
  double d_left = -d_driver_speed + d_driver_turn;
  double d_right = -d_driver_speed - d_driver_turn;
  #endif
  // Set motor and pneumatic outputs from control inputs
  
  // Command motors
  m_left1.Set(d_left);
  m_right1.Set(d_right);

// Command pneumatic solonoids
  if (controller_arms->GetLeftBumperPressed()) {
    solonoid_arms.Toggle();
  }
  if (controller_arms->GetRightBumperPressed()) {
    solonoid_gripper.Toggle();
  }
  if (controller_driver->GetLeftBumperPressed()) {
    solonoid_brakes.Toggle();
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
  if (b_vel_mode) {
    c_extender.SetReference(d_arms_extend, rev::CANSparkMax::ControlType::kDutyCycle);
    //m_extender.Set( d_controller_1_x);
  } else {
  if (controller_arms->GetXButtonPressed()){
    c_extender.SetReference(EXTENDER_HOME, rev::CANSparkMax::ControlType::kPosition);
  } 
  if (controller_arms->GetYButtonPressed()) {
    c_extender.SetReference(EXTENDER_MID, rev::CANSparkMax::ControlType::kPosition);
  }
  if (controller_arms->GetBButtonPressed()) {
    c_extender.SetReference(EXTENDER_HIGH, rev::CANSparkMax::ControlType::kPosition);
  }
  if (controller_arms->GetAButtonPressed()) {
    c_extender.SetReference(EXTENDER_PICK, rev::CANSparkMax::ControlType::kPosition);
  }
}
#else
  m_extender.Set(d_arms_extend);
#endif

  // Toggle for brake/coast mode for motors
  if (controller_driver->GetStartButtonPressed()) {
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
  }
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
