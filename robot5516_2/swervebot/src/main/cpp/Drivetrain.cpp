// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {

  // Adjust the threshold if you want.
  static double turnModuleThreshold = 0.3;
  // Adds a very small multiplier to drive speed so the module will move very slow but maintain its heading.
  static double epsilonScaler = 0.15;

  // When power under threshold, trigger speed mark;
  //isPowerZeroed = Math.hypot(xSpeed, ySpeed) < turnModuleThreshold;
  //isPowerZeroed = std::hypot((double)xSpeed, (double)ySpeed) < turnModuleThreshold;


_sb.append("\txSpeed:");
_sb.append(std::to_string(xSpeed.to<double>()));
_sb.append("\tySpeed:");
_sb.append(std::to_string(ySpeed.to<double>()));


  if (std::hypot(xSpeed.to<double>(), ySpeed.to<double>()) < turnModuleThreshold) 
  {
    isPowerZeroed = true;
  }
  else
  {
    /* code */
    
    isPowerZeroed = false;
  }

_sb.append("\tHYPOT XY:");
_sb.append(std::to_string(std::hypot(xSpeed.to<double>(), ySpeed.to<double>())));

_sb.append("\tisPowerZeroed:");
_sb.append(std::to_string(isPowerZeroed));

  isSpeedMarked = isPowerZeroed && !isSpeedMarkedLast;
  isSpeedMarkedLast = isPowerZeroed;


  if (isSpeedMarked) {
    //auto xFeedback = units::meters_per_second_t(xSpeed.to<double>() * epsilonScaler);
    xSpeedLast = units::meters_per_second_t(xSpeed.to<double>() * epsilonScaler);
    ySpeedLast = units::meters_per_second_t(ySpeed.to<double>() * epsilonScaler);
  }

_sb.append("\tisSpeedMarked:");
_sb.append(std::to_string(isSpeedMarked));


/*
        var swerveModuleStates = !isPowerZeroed ?
                (m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(getGyroAngle()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot)))
                : (m_kinematics.toSwerveModuleStates(
                    fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedLast, ySpeedLast, rot, new Rotation2d(getGyroAngle()))
                        : new ChassisSpeeds(xSpeedLast, ySpeedLast, rot)));



  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});


  auto states = !isPowerZeroed ? 
                  (m_kinematics.ToSwerveModuleStates(
                    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                            : frc::ChassisSpeeds{xSpeed, ySpeed, rot}))
                  :
                  (m_kinematics.ToSwerveModuleStates(
                    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedLast, ySpeedLast, rot, m_gyro.GetRotation2d())
                            : frc::ChassisSpeeds{xSpeedLast, ySpeedLast, rot}));
*/

  auto states = !isPowerZeroed ? 
                  (m_kinematics.ToSwerveModuleStates(
                    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle())
                            : frc::ChassisSpeeds{xSpeed, ySpeed, rot}))
                  :
                  (m_kinematics.ToSwerveModuleStates(
                    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedLast, ySpeedLast, rot, getGyroAngle())
                            : frc::ChassisSpeeds{xSpeedLast, ySpeedLast, rot}));

_sb.append("\tisPowerZeroed:");
_sb.append(std::to_string(isPowerZeroed));


  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl, 0);
  m_frontRight.SetDesiredState(fr, 0);
  m_backLeft.SetDesiredState(bl, 0);
  m_backRight.SetDesiredState(br, 0);

  /* print every ten loops, printing too much too fast is generally bad for performance */
  //if (++_loops >= 10) {
  if (xSpeed.to<double>() > 0 || ySpeed.to<double>() > 0) {
    _loops = 0;
//    printf("%s\n", _sb.c_str());
  }
  _sb.clear();
}




/*
void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
}
*/


frc::Rotation2d  Drivetrain::getGyroAngle() {
      //double angle = ((int)-m_gyro.GetAngle() + (double)360) % (double)360;
    double angle = ((int)-m_gyro.GetAngle() + 360) % 360;
    return frc::Rotation2d{units::degree_t{angle}};
}