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

  // Setup PID arguments for extender
  c_extender.SetFeedbackDevice(e_extender);
  c_extender.SetP(EXTENDER_P);
  c_extender.SetI(EXTENDER_I);
  c_extender.SetD(EXTENDER_D);
  c_extender.SetIZone(EXTENDER_IZ);
  c_extender.SetFF(EXTENDER_FF);
  c_extender.SetOutputRange(EXTENDER_MIN_OUT, EXTENDER_MAX_OUT);
  e_extender.SetPositionConversionFactor(EXTENDER_CONVERSION);
  m_extender.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 100.0);
  m_extender.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -100.0);
  
  #ifdef TEST_PID_DRIVING
  c_left1.SetFeedbackDevice(e_left1);
  c_left1.SetP(DRIVE_P);
  c_left1.SetI(DRIVE_I);
  c_left1.SetD(DRIVE_D);
  c_left1.SetIZone(DRIVE_IZ);
  c_left1.SetFF(DRIVE_FF);
  c_left1.SetOutputRange(DRIVE_MIN_OUT, DRIVE_MAX_OUT);
  e_left1.SetPositionConversionFactor(DRIVE_CONVERSION);

  c_right1.SetFeedbackDevice(e_right1);
  c_right1.SetP(DRIVE_P);
  c_right1.SetI(DRIVE_I);
  c_right1.SetD(DRIVE_D);
  c_right1.SetIZone(DRIVE_IZ);
  c_right1.SetFF(DRIVE_FF);
  c_right1.SetOutputRange(DRIVE_MIN_OUT, DRIVE_MAX_OUT);
  e_right1.SetPositionConversionFactor(DRIVE_CONVERSION);
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
	frc::SmartDashboard::PutNumber("IMU Pitch", s_IMU.GetYComplementaryAngle().value());
  #ifdef TEST_BALANCE
  #endif
  if (!di_extender_upper.Get()) {
    e_extender.SetPosition(0);
  }
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  
  #ifdef TEST_BALANCE
  d_pitch = s_IMU.GetYComplementaryAngle() - 86_deg;
  d_initial_pitch = s_IMU.GetYComplementaryAngle() - 86_deg;
  #endif
  
  // Start timer for auto
  game_timer.Reset();
  game_timer.Start();
  t_pause_time = 1_s;
  fmt::print("Auto selected: {}\n", m_autoSelected);

}

void Robot::AutonomousPeriodic() {
  /////////////////////////////////////////////////////////////////////////////
  // Score a cube in the high node and then get the mobility score.
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameMobility) {

    // Backup and extend arms
    if (game_timer.Get() < 0.5_s){
      m_left1.Set(0.20);
      m_right1.Set(0.20);
      c_extender.SetReference(EXTENDER_MID / 2, rev::CANSparkMax::ControlType::kPosition);
    } 

    // Raise arms and stop
    if (game_timer.Get() > 0.5_s && game_timer.Get() < 1_s){
      solonoid_arms.Set(true);
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }

    // Extend arms and drive forward
    if (game_timer.Get() > 3_s && game_timer.Get() < 3.5_s){
      c_extender.SetReference(EXTENDER_HIGH, rev::CANSparkMax::ControlType::kPosition);
      m_left1.Set(-0.23);
      m_right1.Set(-0.23);
    }

    // Stop and give us time to settle
    if (game_timer.Get() > 3.5_s && game_timer.Get() < 4_s){
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }
    
    // Release
    if (game_timer.Get() > 6_s && game_timer.Get() < 6.5_s){
      solonoid_gripper.Set(true);
    }

    // Close the gripper and retract to a safe position to lower the arms
    if (game_timer.Get() > 6.5_s && game_timer.Get() < 7_s){
      solonoid_gripper.Set(false);
      c_extender.SetReference(EXTENDER_MID / 2, rev::CANSparkMax::ControlType::kPosition);
    }
    
    // Drop the arms and bring them all the way home.
    if (game_timer.Get() > 8_s && game_timer.Get() < 8.5_s){
      solonoid_arms.Set(false);
      c_extender.SetReference(EXTENDER_HOME, rev::CANSparkMax::ControlType::kPosition);
    }

    // Back up out of the community. This happens at the same time as we reset.
    if (game_timer.Get() > 6.5_s && game_timer.Get() < 12.5_s){
      m_left1.Set(0.25);
      m_right1.Set(0.25);
    } 
    // Set speed to zero and hold position.
    if (game_timer.Get() > 12.5_s && game_timer.Get() < 13_s){
      m_left1.Set(0.0);
      m_right1.Set(0.0);
    }
    //TODO Come back into the community after we go out.
  }

  /////////////////////////////////////////////////////////////////////////////
  // Balancing Auto
  /////////////////////////////////////////////////////////////////////////////
  if (m_autoSelected == kAutoNameBalance) {
    // Start out by scoring from the poofer.
    if ( game_timer.Get() < 1_s){
      solonoid_poofer.Set(true);
    }

    // Reset poofer and start driving backwards. Should end when we are partially over the charge station.
    if (game_timer.Get() > 1_s &&  game_timer.Get() < 3.2_s){
      solonoid_poofer.Set(false);
      m_left1.Set(0.30);
      m_right1.Set(0.30);
    }

    // Go slow the rest of the way over the charge station, and out of the community.
    if (game_timer.Get() > 3.2_s && game_timer.Get() < 9_s){
      m_left1.Set(0.10);
      m_right1.Set(0.10);
    } 

    // Start back up the charge station at 1/4 speed for 1.5 seconds
    if (game_timer.Get() > 9_s && game_timer.Get() < 11_s){
      m_left1.Set(-0.25);
      m_right1.Set(-0.25);
    }
    
    // Go a little slower for the last leg.
    if (game_timer.Get() > 11_s && game_timer.Get() < 12.25_s){
      m_left1.Set(-0.18);
      m_right1.Set(-0.18);
    } 
    /*   
    if (game_timer.Get() > 11.25_s && game_timer.Get() < 11.5_s){
      im_mode = rev::CANSparkMax::IdleMode::kBrake;
      m_left1.SetIdleMode(im_mode);
      m_left2.SetIdleMode(im_mode);
      m_right1.SetIdleMode(im_mode);
      m_right2.SetIdleMode(im_mode);
      solonoid_brakes.Set(true);
    }*/

    if (game_timer.Get() > 12.25_s && game_timer.Get() < 15_s) {
      solonoid_brakes.Set(true);
    } else {
      solonoid_brakes.Set(false);
    }  
    // Start autobalance process
    // Assumes we are not on a level surface.
    if (game_timer.Get() > 13_s && game_timer.Get() < 15_s) {
      // Set the idle mode so we slip less.
      im_mode = rev::CANSparkMax::IdleMode::kBrake;
      m_left1.SetIdleMode(im_mode);
      m_left2.SetIdleMode(im_mode);
      m_right1.SetIdleMode(im_mode);
      m_right2.SetIdleMode(im_mode);
      // Save historical pitch and get current pitch.
      d_hpitch = d_pitch;
      d_pitch = s_IMU.GetYComplementaryAngle() - 86_deg;
      // Manage brakes during pause time.
      if (game_timer.Get() < t_pause_time) {
        solonoid_brakes.Set(true);
      } else {
        solonoid_brakes.Set(false);
      }
      // Autobalance magic.
      if ((d_pitch < -DEADBAND_BALANCE and d_hpitch > DEADBAND_BALANCE) or (d_pitch > DEADBAND_BALANCE and d_hpitch < -DEADBAND_BALANCE)) {
          d_drive_speed *= NERF_AUTO;
          t_pause_time = game_timer.Get() + 1_s;
        }
      if ((d_pitch > DEADBAND_BALANCE or d_pitch < -DEADBAND_BALANCE) and 
        (game_timer.Get() >= t_pause_time or t_pause_time == 1_s)) {
        //if (game_timer.Get() >= t_pause_time) {
          if (d_pitch > DEADBAND_BALANCE){
            // Move Forward
            m_left1.Set(-d_drive_speed);
            m_right1.Set(-d_drive_speed);
          } else if (d_pitch < -DEADBAND_BALANCE){
            //Move Backward
            m_left1.Set(d_drive_speed);
            m_right1.Set(d_drive_speed);
          }
        //}
      }
    }
  }
}

void Robot::TeleopInit() {
  // Reset idle mode for driving so we don't brown-out and disengauge brakes. 
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
  double d_turn_minorl = controller_driver->GetLeftTriggerAxis();
  double d_turn_minorr = controller_driver->GetRightTriggerAxis();

  // Apply movement macros
  d_driver_speed = d_driver_speed * NERF_SPEED;
  d_driver_turn = d_driver_turn * NERF_TURN;
  d_arms_extend *= NERF_EXTEND;
  d_turn_minorl *= 0.5;
  d_turn_minorr *= 0.5;
  
  // Set motor and pneumatic outputs from control inputs
  
  // Command motors
  #ifndef TEST_PID_DRIVING
  // Drive motors and calculate how minor turning changes the controls.
  d_drive.ArcadeDrive(d_driver_speed, d_driver_turn - d_turn_minorl + d_turn_minorr);
  #else
  // Calculate individual speed targets 
  double d_left_speed = d_driver_speed + d_driver_turn- d_turn_minorl + d_turn_minorr;
  double d_right_speed = d_driver_speed - d_driver_turn + d_turn_minorl - d_turn_minorr;
  // Apply velocity set points.
  c_left1.SetReference(d_left_speed, rev::CANSparkMax::ControlType::kVelocity);
  c_right1.SetReference(d_right_speed, rev::CANSparkMax::ControlType::kVelocity);
  #endif
// Command pneumatic solonoids
  // Arms control
  if (controller_arms->GetLeftBumperPressed()) {
    // Retract arms when up. Remember, threading on the rod is such that
    // more negative is farther out. 0 is initial position.
    if (solonoid_arms.Get() && e_extender.GetPosition() < EXTENDER_PICK) {
      c_extender.SetReference((EXTENDER_PICK + EXTENDER_HOME) / 2, rev::CANSparkMax::ControlType::kPosition);
    }
    solonoid_arms.Toggle();
  }

  // Gripper control
  if (controller_arms->GetRightBumperPressed()) {
    solonoid_gripper.Toggle();
  }

  // Brake control. Sets pneumatic brakes and brake idle mode.
  if (controller_driver->GetLeftBumperPressed()) {
    solonoid_brakes.Toggle();
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

  // Poofer control
  if (controller_arms->GetLeftStickButtonPressed()) {
    solonoid_poofer.Set(true);
  }
  if (controller_arms->GetLeftStickButtonReleased()) {
    solonoid_poofer.Set(false);
  }

  if (controller_arms->GetStartButtonPressed()) {
    b_vel_mode = !b_vel_mode;
    c_extender.SetReference(e_extender.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
  }

  // Command extender motor
  // Need to figure out a way to do both of these at the same time without toggling states.
  if (!b_vel_mode) {  
    // Apply limit switches to extender motor
    if (m_extender.GetAppliedOutput() && !di_extender_upper.Get()) {
      d_arms_extend = 0.0;
    }
    if (d_arms_extend < 0.0 && !di_extender_lower.Get()) {
      d_arms_extend = 0.0;
    }
    c_extender.SetReference(d_arms_extend, rev::CANSparkMax::ControlType::kDutyCycle);
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
