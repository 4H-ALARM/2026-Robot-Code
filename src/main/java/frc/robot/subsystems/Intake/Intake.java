// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO m_intakeIO;

  private IntakeIOInputs m_inputs;

  public Intake(IntakeIO intake) {
    this.m_intakeIO = intake;
    m_inputs = new IntakeIOInputs();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // this.m_intakeIO.updateInputs(m_inputs);
  }

  // public void resetEncoder() {
  //   m_intakeIO.resetEncoder();
  // }

  // /// don't use for regular code, no automatic stop.
  // public void changeAngleTest(double speed) {
  //   m_intakeIO.changeAngleTest(speed);
  //   // dont use for regular code, no automatic stop
  // }

  public void setAngleDown() {
    m_intakeIO.setAngle(IntakeConstants.downRotation);
  }

  public void setDefaultAngle() {
    m_intakeIO.setAngle(IntakeConstants.startingRotation);
  }

  public void setIntaking() {
    m_intakeIO.setSpeed(-0.6);
  }

  public void stopIntaking() {
    m_intakeIO.setSpeed(0);
  }

  public void resetEncoder() {
    m_intakeIO.resetEncoder();
  }

  // public void setIntakeSpeed(double speed) {
  //   m_intakeIO.setIntakeSpeed(speed);
  // }

  // public void stopIntake() {
  //   m_intakeIO.stopIntake();
  // }

  // public void updateInputs() {
  //   m_intakeIO.updateInputs(m_inputs);
  // }
}
