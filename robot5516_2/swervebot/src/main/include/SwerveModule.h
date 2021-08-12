// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once



#include <frc/Encoder.h>
//#include <frc/PWMSparkMax.h>
//#include <frc/Talon.h>
#include "ctre/Phoenix.h"
//#include <WPI_TalonFX.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>
#include <utility>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int tuningSRXChannel, double encoderOffset );
  frc::SwerveModuleState GetState() const;
  void SetDesiredState(const frc::SwerveModuleState& state, bool lowPower);

  


 double GetDriveMotorVelocityMeters ();

 // Get the raw output of the absolute encoder.
 double GetTurnMotorPositionRaw ();

 // Add on encoder offset.
 double GetTurnMotorPosition ();

 frc::Rotation2d GetTurnMotorRotation ();

 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kEncoderResolution = 4096;

  static constexpr int kDrivingEncoderResolution = 4096;
  static constexpr int kTurningEncoderResolution = 4096;
  static constexpr double kWheelDiameterMeters = 0.1016;    // I made that up
  static constexpr double kGearRatio = 14.26;               // I made that up
  static constexpr double kWheelDistancePerEncoderPulse =
            (kWheelDiameterMeters * wpi::math::pi) / (kGearRatio * kDrivingEncoderResolution);

    //turing offset 
  double m_encoderOffset = 0 ;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1.5_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2


    //FeedbackDevice fbDevice;
     WPI_TalonFX m_driveMotor;
     WPI_TalonFX m_turningMotor;
     WPI_TalonSRX m_turningEncoder;
    
    // TalonFX * _driveMotor;
    // TalonFX * _turningMotor;
    // TalonSRX * _turningEncoder;

  frc2::PIDController m_drivePIDController{0.25, 0, 0};
  // frc::ProfiledPIDController<units::radians> m_turningPIDController{
  //     0.65,
  //     0.0,
  //     0.0,
  //     {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      0.65,
      0.0,
      0.0,
      frc::TrapezoidProfile<units::radian>::Constraints{
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};



//  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
//                                                                3_V / 1_mps};

//  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
//      1_V, 0.5_V / 1_rad_per_s};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0_V,
                                                                0.25_V / 1_mps};

  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      0.035_V, 0.15_V / 1_rad_per_s};

};
