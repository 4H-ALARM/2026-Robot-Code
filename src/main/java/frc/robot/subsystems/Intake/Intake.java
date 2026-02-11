// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO intake;

  private IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO Intake) {
    this.intake = Intake;
    inputs = new IntakeIOInputsAutoLogged();
    this.intake.setAngle(IntakeConstants.startingRotation);
  }

  public void setToIntake() {
    this.intake.setAngle(IntakeConstants.downRotation);
    this.intake.setSpeed(IntakeConstants.intakeVelocity);
  }

  public void stopIntake() {
    this.intake.setSpeed(0);
  }

  public void collapse() {
    this.intake.setAngle(IntakeConstants.upRotation);
    this.intake.setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.intake.updateTuningValues();
    this.intake.updateInputs(inputs);
  }
}
