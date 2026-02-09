// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean rotationMotorConnected;
    boolean spinMotorConnected;

    double rotation;
    double speed;
  }

  public void setAngle(Rotation2d targetRotation);

  public void setSpeed(double speed);

  public void stopMotors();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
