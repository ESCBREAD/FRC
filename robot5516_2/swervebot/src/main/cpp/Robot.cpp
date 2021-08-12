// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    //m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

//FOR TEST
std::string _sb;
int _loops = 0;

  int iJoystickTpye = 0; // 0 xbox ； 1 joystick

 private:
  frc::XboxController m_controller{0};
  frc::Joystick m_controller2{1};
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{4 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{4 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {

  double dX = 0;
  double dY = 0;

  if (iJoystickTpye == 0)
  {
    dX = m_controller.GetX(frc::GenericHID::kLeftHand);
    dY = m_controller.GetY(frc::GenericHID::kLeftHand);
  }
  else
  {
    dX = m_controller2.GetX(frc::GenericHID::kRightHand);
    dY = m_controller2.GetY(frc::GenericHID::kRightHand);
  }


_sb.append("\tGetX:");
_sb.append(std::to_string(dX));
_sb.append("\tGetY:");
_sb.append(std::to_string(dY));
_sb.append("\n");



    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
//    const auto xSpeed = -m_xspeedLimiter.Calculate(
//                            m_controller.GetY(frc::GenericHID::kLeftHand)) *
//                        Drivetrain::kMaxSpeed;


//Math.abs(input) > 0.05 ? input : 0;
//Math.max(low, Math.min(value, high));
/*
        xSpeed =  m_xSpeedLimiter.calculate(MyUtil.applyDeadband(MyUtil.clip(x, -1, 1)))
                        * Drivetrain.kMaxSpeed;

        ySpeed = m_ySpeedLimiter.calculate(MyUtil.applyDeadband(MyUtil.clip(y, -1, 1)))
                        * Drivetrain.kMaxSpeed;

        rot = -m_rotLimiter.calculate(MyUtil.applyDeadband(MyUtil.clip(omega, -1, 1)))
                        * Drivetrain.kMaxAngularSpeed;
*/
//dY = wpi::math::
  dY = std::max((double) -1 , std::min(dY, (double)1) );
  dY = std::abs(dY) > 0.05 ? dY : 0;
    const auto xSpeed = -m_xspeedLimiter.Calculate(dY)  * Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
//    const auto ySpeed = -m_yspeedLimiter.Calculate(
//                            m_controller.GetX(frc::GenericHID::kLeftHand)) *
//                        Drivetrain::kMaxSpeed;
  dX = std::max((double) -1 , std::min(dX, (double)1) );
  dX = std::abs(dX) > 0.05 ? dX : 0;

    const auto ySpeed = -m_yspeedLimiter.Calculate(dX) * Drivetrain::kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
//    const auto rot = -m_rotLimiter.Calculate(
//                         m_controller.GetX(frc::GenericHID::kRightHand)) *
//                     Drivetrain::kMaxAngularSpeed;

    const auto rot = -m_rotLimiter.Calculate(dX)  * Drivetrain::kMaxAngularSpeed;

_sb.append("\tdX:");
_sb.append(std::to_string(dX));
_sb.append("\tdY:");
_sb.append(std::to_string(dY));
_sb.append("\n");

_sb.append("\txSpeed:");
_sb.append(std::to_string(xSpeed.to<double>()));
_sb.append("\tySpeed:");
_sb.append(std::to_string(ySpeed.to<double>()));
_sb.append("\trot:");
_sb.append(std::to_string(rot.to<double>()));
_sb.append("\n");

  /* print every ten loops, printing too much too fast is generally bad for performance */
  if (++_loops >= 10) {
    _loops = 0;
//printf("%s\n", _sb.c_str());
  }
  _sb.clear();

// /* update motor controller */
// _talon.set(ControlMode.PercentOutput, xSpeed);
// /* check our live faults */
// _talon.getFaults(_faults);
// /* hold down btn1 to print stick values */
// if (_joystick.getRawButton(1)) {
//   System.out.println("Sensor Vel:" + _talon.getSelectedSensorVelocity());
//   System.out.println("Sensor Pos:" + _talon.getSelectedSensorPosition());
//   System.out.println("Out %" + _talon.getMotorOutputPercent());
//   System.out.println("Out Of Phase:" + _faults.SensorOutOfPhase);
// }

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);

  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


