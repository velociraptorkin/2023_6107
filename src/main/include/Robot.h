// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

// Build Options

#define TEST_PID 1
//#define TEST_BALANCE 1
//#define TEST_MINOR_TURN 1
//#define TEST_POOFER 1


#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>

// Pneumatic bindings
#define CHANNEL_BRAKE 6
#define CHANNEL_ARMS 3
#define CHANNEL_GRIPPER 5
#define CHANNEL_POOFER 2

// Digital IO Bindings
#define CHANNEL_EXTENDER_UPPER 1
#define CHANNEL_EXTENDER_LOWER 0
#define CHANNEL_EXTENDER_DROP 2

// PID Position Settings
#define EXTENDER_P 0.3
#define EXTENDER_I 1e-4
#define EXTENDER_D 1.0
#define EXTENDER_IZ 0.0
#define EXTENDER_FF 0.0
#define EXTENDER_MAX_OUT 1.0
#define EXTENDER_MIN_OUT -1.0
#define EXTENDER_HOME -1.0
#define EXTENDER_MID -2.0
#define EXTENDER_HIGH -4.0
#define EXTENDER_PICK -6.0
#define EXTENDER_CONVERSION 0.125

// Various other settings
#define NERF_SPEED 0.25
#define NERF_TURN 1.0
#define NERF_EXTEND 0.3
#define NERF_AUTO 0.5
#define AUTO_SPEED 0.125
#define DEADBAND_CONTROL 0.125
#define DEADBAND_BALANCE 5.0
#define CONTROL_MODE ControlMode::PercentOutput

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "No Auto";
  const std::string kAutoNameMobility = "Mobility";
  const std::string kAutoNameBalance = "Balance";
  std::string m_autoSelected;
  rev::CANSparkMax::IdleMode im_mode = rev::CANSparkMax::IdleMode::kCoast;
  frc::Timer game_timer;
  #ifdef TEST_BALANCE
  ctre::phoenix::sensors::WPI_PigeonIMU s_IMU{13}; 
  double d_pitch, d_hpitch, d_initial_pitch;
  units::time::second_t t_pause_time;
  #endif

  double d_drive_speed = AUTO_SPEED;
  #ifdef TEST_PID
  bool b_vel_mode = false;
  #endif

  // Controllers
  frc::XboxController * controller_driver = new frc::XboxController(0);
  frc::XboxController * controller_arms = new frc::XboxController(1);
  
  
  // Motor controllers
  rev::CANSparkMax m_left1{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_left2{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right1{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right2{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_extender{5, rev::CANSparkMax::MotorType::kBrushless};
  

  // Pneumatic controls
  frc::Solenoid solonoid_arms{12, frc::PneumaticsModuleType::REVPH, CHANNEL_ARMS};
  frc::Solenoid solonoid_gripper{12, frc::PneumaticsModuleType::REVPH, CHANNEL_GRIPPER};
  frc::Solenoid solonoid_brakes{12, frc::PneumaticsModuleType::REVPH, CHANNEL_BRAKE};
  #ifdef TEST_POOFER
  frc::Solenoid solonoid_poofer{12, frc::PneumaticsModuleType::REVPH, CHANNEL_POOFER};
  #endif
  frc::Compressor compressor_main{12, frc::PneumaticsModuleType::REVPH};

  // Limit switches
  frc::DigitalInput di_extender_upper{CHANNEL_EXTENDER_UPPER};
  frc::DigitalInput di_extender_lower{CHANNEL_EXTENDER_LOWER};
  frc::DigitalInput di_extender_drop{CHANNEL_EXTENDER_DROP};
  
  // Encoders
  rev::SparkMaxRelativeEncoder e_extender = m_extender.GetEncoder();
  rev::SparkMaxRelativeEncoder e_left1 = m_left1.GetEncoder();
  rev::SparkMaxRelativeEncoder e_left2 = m_left2.GetEncoder();
  rev::SparkMaxRelativeEncoder e_right1 = m_right1.GetEncoder();
  rev::SparkMaxRelativeEncoder e_right2 = m_right2.GetEncoder();
#ifdef TEST_PID
  // PID Controllers
  rev::SparkMaxPIDController c_extender = m_extender.GetPIDController();
#endif
};
