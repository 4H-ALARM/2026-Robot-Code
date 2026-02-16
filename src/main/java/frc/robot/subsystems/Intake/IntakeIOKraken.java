// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {

  TalonFX m_rotationMotor;
  TalonFX m_rotationMotorFollow;
  TalonFX m_intakingMotor;

  Slot0Configs m_pIDConfigs;
  PositionVoltage m_requestedVoltage;

  IntakeConstants m_constants;

  public IntakeIOKraken() {
    m_intakingMotor = new TalonFX(m_constants.intakingMotorID);
    m_rotationMotor = new TalonFX(m_constants.rotationMotorID);
    m_rotationMotorFollow = new TalonFX(m_constants.rotationMotorFollowID);
    m_rotationMotorFollow.setControl(
        new Follower(m_constants.rotationMotorID, MotorAlignmentValue.Opposed));
    m_pIDConfigs =
        new Slot0Configs()
            .withKP(m_constants.angleMotorkp)
            .withKI(m_constants.angleMotorki)
            .withKD(m_constants.angleMotorkd);
    m_rotationMotor.getConfigurator().apply(m_pIDConfigs);
    m_requestedVoltage = new PositionVoltage(0).withSlot(0);
  }

  public void resetEncoder() {
    m_rotationMotor.setPosition(0);
  }

  public void changeAngleTest(double speed) {
    if (speed > 1) {
      speed = 1;
    }
    if (speed < -1) {
      speed = -1;
    }
    m_rotationMotor.set(speed);
  }

  public void setAngle(double angleDegrees, IntakeIOInputs inputs) {
    // still need to find the angle before this will work;
    double motorRotations =
        (angleDegrees - inputs.rotationDegrees) * m_constants.rotationGearRatio / 360;

    m_rotationMotor.setControl(m_requestedVoltage.withPosition(motorRotations));
  }

  public void setIntakeSpeed(double speed) {
    if (speed > 1) {
      speed = 1;
    }
    if (speed < -1) {
      speed = -1;
    }
    m_intakingMotor.set(speed);
  }

  public void stopIntake() {

    m_intakingMotor.set(0);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rotationMotorConnected = m_rotationMotor.isConnected();
    inputs.intakingMotorConnected = m_intakingMotor.isConnected();
    inputs.rotationMotorFollowerConnected = m_rotationMotorFollow.isConnected();

    // need to figure out how many encoder units equals one full rotation
  }
}
