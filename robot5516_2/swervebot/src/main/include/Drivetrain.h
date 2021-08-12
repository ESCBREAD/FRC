// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>
#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      2.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

    //FOR TEST
    std::string _sb;
	int _loops = 0;

 private:
     bool isSpeedMarked;
     bool isSpeedMarkedLast;
     bool isPowerZeroed;

     units::meters_per_second_t xSpeedLast;
     units::meters_per_second_t ySpeedLast;


    double FRONT_LEFT_OFFSET = 51;
    double FRONT_RIGHT_OFFSET = -1906;
    double BACK_LEFT_OFFSET = -1734;
    double BACK_RIGHT_OFFSET = -1906;

    // double FRONT_LEFT_OFFSET = 0;
    // double FRONT_RIGHT_OFFSET = 0;
    // double BACK_LEFT_OFFSET = 0;
    // double BACK_RIGHT_OFFSET = 0;

/*
    frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
    frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    private final Translation2d m_frontLeftLocation = new Translation2d(0.286, 0.292);
    private final Translation2d m_frontRightLocation = new Translation2d(0.286, -0.292);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.286, 0.292);
    private final Translation2d m_backRightLocation = new Translation2d(-0.286, -0.292);

*/
    frc::Translation2d m_frontLeftLocation{0.3683_m, 0.3683_m};
    frc::Translation2d m_frontRightLocation{0.3683_m, -0.3683_m};
    frc::Translation2d m_backLeftLocation{-0.3683_m, 0.3683_m};
    frc::Translation2d m_backRightLocation{-0.3683_m, -0.3683_m};


    SwerveModule m_frontLeft{4, 3, 10, FRONT_LEFT_OFFSET};
    SwerveModule m_frontRight{2, 1, 9, FRONT_RIGHT_OFFSET};
    SwerveModule m_backLeft{8, 7, 12, BACK_LEFT_OFFSET};
    SwerveModule m_backRight{6, 5, 11, BACK_RIGHT_OFFSET};

    frc::AnalogGyro m_gyro{0};

    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};

    frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};
};
