// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean rotationMotorConnected;
    boolean rotationMotorFollowerConnected;
    boolean intakingMotorConnected;

    double rotationDegrees;
    double rotationSpeed;
  }

  public default void resetEncoder() {}
  ;

  public default void changeAngleTest(double speed) {}
  ;

  public default void setAngle(double angleDegrees, IntakeIOInputs inputs) {}
  ;

  public default void setIntakeSpeed(double speed) {}
  ;

  public default void stopIntake() {}
  ;

  public default void updateInputs(IntakeIOInputs inputs) {}
}
