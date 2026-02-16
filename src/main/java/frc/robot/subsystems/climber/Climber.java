// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private ClimberIO m_climber;

  private ClimberIOInputs m_climberInputs;

  public Climber(ClimberIO climber) {
    m_climber = climber;
  }

  @Override
  public void periodic() {
    m_climber.updateInputs(m_climberInputs);
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double positionInches, ClimberIOInputs inputs) {
    m_climber.setArmPosition(positionInches, inputs);
  }

  public void moveArm(double speed) {
    m_climber.moveArm(speed);
  }
}
