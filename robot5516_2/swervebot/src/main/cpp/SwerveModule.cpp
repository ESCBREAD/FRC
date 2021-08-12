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
                            const int tuningSRXChannel,
                            double encoderOffset)
     : m_driveMotor(driveMotorChannel), m_turningMotor(turningMotorChannel), m_turningEncoder(tuningSRXChannel) {

// SwerveModule::SwerveModule(const int driveMotorChannel,
//                            const int turningMotorChannel,
//                            const int tuningSRXChannel,
//                            double encoderOffset)
// {

    // _driveMotor = new TalonFX(driveMotorChannel);
    // _turningMotor = new TalonFX(turningMotorChannel);
    // _turningEncoder = new TalonSRX(tuningSRXChannel);


  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  //_turningEncoder->SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
    m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi),
                                            units::radian_t(wpi::math::pi));

    // _driveMotor->SetNeutralMode(Brake);
    // _turningMotor->SetNeutralMode(Brake);
    // _turningEncoder->ConfigSelectedFeedbackSensor(  FeedbackDevice::CTRE_MagEncoder_Absolute , 0, 0);
    // _turningEncoder->ConfigFeedbackNotContinuous(true, 0);
    // _turningEncoder->SetSensorPhase(false);

    m_driveMotor.SetNeutralMode(Brake);
    m_turningMotor.SetNeutralMode(Brake);
    m_turningEncoder.ConfigSelectedFeedbackSensor(  FeedbackDevice::CTRE_MagEncoder_Absolute , 0, 0);
    m_turningEncoder.ConfigFeedbackNotContinuous(true, 0);
    m_turningEncoder.SetSensorPhase(false);



    // We have a 60-tooth gear on the encoder shaft
    // and I presume to have a 2-tooth vacancy on the gear xD. (360/60*2 = 6 deg)
    m_turningPIDController.SetTolerance((units::angle::radian_t) 6, (units::angular_velocity::radians_per_second_t) 0);
    m_encoderOffset = encoderOffset;

}

//frc::SwerveModuleState SwerveModule::GetState() const {
//  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
//            frc::Rotation2d(units::radian_t(_turningEncoder->Get()))};

//  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
//          frc::Rotation2d(units::radian_t(_turningEncoder->Get()))};
//}

double SwerveModule::GetDriveMotorVelocityMeters () {
    return m_driveMotor.GetSelectedSensorVelocity() * 10 * kWheelDistancePerEncoderPulse;
}

// Get the raw output of the absolute encoder.
double SwerveModule::GetTurnMotorPositionRaw () {
std::string _sb;
_sb.append("\tencodeID:");
_sb.append(std::to_string(m_turningEncoder.GetDeviceID()));  
_sb.append("\tPositionRaw:");
_sb.append(std::to_string(m_turningEncoder.GetSelectedSensorPosition()));  
//printf("%s\n", _sb.c_str());

    return m_turningEncoder.GetSelectedSensorPosition();
}

// Add on encoder offset.
double SwerveModule::GetTurnMotorPosition () {

    int raw = (int) (GetTurnMotorPositionRaw() - m_encoderOffset);
    // int encoderid = _turningEncoder->GetDeviceID();

    return raw < 0 ? raw + kTurningEncoderResolution : raw;
}

// Get angular position of a module.f
frc::Rotation2d SwerveModule::GetTurnMotorRotation () {

    double encoderOffset =  GetTurnMotorPosition() / (double) kTurningEncoderResolution * 2 * (double)wpi::math::pi 
                                - (double)wpi::math::pi;

    return frc::Rotation2d(units::degree_t{encoderOffset});

//    return new frc::Rotation2d(units::radian_t ( (double) GetTurnMotorPosition() / (double) 
//kTurningEncoderResolution * 2 * (double)(double)wpi::math::pi - (double)wpi::math::pi));
}


void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState, bool lowPower) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        frc::SwerveModuleState state = referenceState.Optimize(referenceState, GetTurnMotorRotation());


        // Calculate the drive output from the drive PID controller.
        double driveOutput =
                m_drivePIDController.Calculate(GetDriveMotorVelocityMeters(), state.speed.to<double>());

        //double driveFeedforward = m_driveFeedforward.Calculate(state.speed.to<double>());
        const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

        // Calculate the turning motor output from the turning PID controller.
        // We have a commented code below.
        // Uncomment that if you the module to turn exactly as your game stick points to.
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
        

        // m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, turnMotorOuptut);
        // m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 
        //                   std::max((double)-1, std::min(driveMotorOutput,(double) 1) ) * lowPowerScaler);

        m_turningMotor.Set( turnMotorOuptut);
        m_driveMotor.Set(std::max((double)-1, std::min(driveMotorOutput,(double) 1) ) * lowPowerScaler);
        //std::clamp(driveMotorOutput, -1, 1);
}
