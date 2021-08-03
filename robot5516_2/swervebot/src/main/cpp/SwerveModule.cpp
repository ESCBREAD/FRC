// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include "units/math.h"
#include "units/math.h"
#include "units/math.h"


SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int tuningSRXChannel)
    : m_driveMotor(driveMotorChannel), m_turningMotor(turningMotorChannel), m_turningEncoder(tuningSRXChannel){
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
//  m_driveEncoder.SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius /
//                                     kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
//  m_turningEncoder.SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi),
                                               units::radian_t(wpi::math::pi));

        m_driveMotor.SetNeutralMode(Brake);
        m_turningMotor.SetNeutralMode(Brake);
        //m_turningEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        
        m_turningEncoder.ConfigSelectedFeedbackSensor(  FeedbackDevice::CTRE_MagEncoder_Absolute , 0, 10);
        m_turningEncoder.ConfigFeedbackNotContinuous(true, 10);
        m_turningEncoder.SetSensorPhase(false);

        // We have a 60-tooth gear on the encoder shaft
        // and I presume to have a 2-tooth vacancy on the gear xD. (360/60*2 = 6 deg)
        //m_turningPIDController.setTolerance(Math.toRadians(6));


        //frc::ProfiledPIDController::Distance_t tDistance = 6;
        //frc::ProfiledPIDController::Velocity_t tVelocity = 0;
        //m_turningPIDController.template to <Distance_t>
        //positionTolerance.template to<double>();
        //m_turningPIDController.SetTolerance(tDistance, tVelocity );
        //m_turningPIDController.SetTolerance(tDistance, tVelocity );
        //std::numeric_limits<double>::infinity()

        //m_turningPIDController.SetTolerance(frc::ProfiledPIDController::Distance_t 6 , frc::ProfiledPIDController::Velocity_t 0);
        m_turningPIDController.SetTolerance((units::angle::radian_t) 6, (units::angular_velocity::radians_per_second_t) 0);

}

//frc::SwerveModuleState SwerveModule::GetState() const {
//  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
//            frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};

//  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
//          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
//}


double SwerveModule::GetDriveMotorVelocityMeters () {
    return m_driveMotor.GetSelectedSensorVelocity() * 10 * kWheelDistancePerEncoderPulse;
}

// Get the raw output of the absolute encoder.
double SwerveModule::GetTurnMotorPositionRaw () {
    return m_turningEncoder.GetSelectedSensorPosition();
}

// Add on encoder offset.
double SwerveModule::GetTurnMotorPosition () {
    int raw = (int) (GetTurnMotorPositionRaw() - encoderOffset);
    return raw < 0 ? raw + kTurningEncoderResolution : raw;
}

// Get angular position of a module.f
frc::Rotation2d SwerveModule::GetTurnMotorRotation () {

    double encoderOffset =  GetTurnMotorPosition() / (double) kTurningEncoderResolution * 2 * (double)wpi::math::pi - (double)wpi::math::pi;


    return frc::Rotation2d(units::degree_t{encoderOffset});

//    return new frc::Rotation2d(units::radian_t ( (double) GetTurnMotorPosition() / (double) 
//kTurningEncoderResolution * 2 * (double)(double)wpi::math::pi - (double)wpi::math::pi));
}


void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState, bool lowPower) {
  // Optimize the reference state to avoid spinning further than 90 degrees

//  const auto state = frc::SwerveModuleState::Optimize(
//      referenceState, units::radian_t(m_turningEncoder.Get()));
      
        // Optimize the reference state to avoid spinning further than 90 degrees
        //frc::SwerveModuleState state = referenceState.Optimize(referenceState, GetTurnMotorRotation());
        frc::SwerveModuleState state = referenceState.Optimize(referenceState, GetTurnMotorRotation());


        // Calculate the drive output from the drive PID controller.

        double driveOutput =
                m_drivePIDController.Calculate(GetDriveMotorVelocityMeters(), state.speed.to<double>());

        //double driveFeedforward = m_driveFeedforward.Calculate(state.speed.to<double>());
        const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

        // Calculate the turning motor output from the turning PID controller.
        // We have a commented code below.
        // Uncomment that if you the module to turn exactly as your game stick points to.

//units::radian_t()        units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

            
        const auto turnOutput = 
                m_turningPIDController.Calculate(GetTurnMotorRotation().Radians(), state.angle.Radians());
                //m_turningPIDController.calculate(getTurnMotorRotation().getRadians(), desiredState.angle.getRadians());

        const auto turnFeedforward =
                m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

        // Scale the driving motor output for testing.
        double lowPowerScaler = lowPower ? 0.325 : 1;
        double driveScaler = 1;

        double turnMotorOuptut = turnOutput + (double)turnFeedforward;
        double driveMotorOutput = (driveOutput + (double)driveFeedforward) * driveScaler;

        m_turningMotor.Set(turnMotorOuptut);
        //m_driveMotor.set(MyUtil.clip(driveMotorOutput, -1, 1) * lowPowerScaler);

        //m_driveMotor.Set(max(-1, min(driveMotorOutput, 1)) * lowPowerScaler);
        m_driveMotor.Set(std::max((double)-1, std::min(driveMotorOutput,(double) 1) ) * lowPowerScaler);
        

  //m_driveMotor.Set(1);
  //m_turningMotor.Set(1);


  // Set the motor outputs.
  //m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  //m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
