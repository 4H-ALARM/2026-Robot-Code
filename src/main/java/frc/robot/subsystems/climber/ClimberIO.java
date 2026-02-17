// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public interface ClimberIO {

  public class ClimberIOInputs {
    public double currentLengthInches = 0;
  }

  public default void moveArm(double speed) {}

  public default void setArmPosition(double positionInches, ClimberIOInputs inputs) {}

  public default void updateInputs(ClimberIOInputs inputs) {}
}
