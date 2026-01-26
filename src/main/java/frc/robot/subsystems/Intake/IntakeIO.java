// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean pivotLeadMotorConnected;
    boolean pivotFollowerMotorConnected;
    boolean intakeMotorConnected;

    double angleMotorCounts;
    double speed;
    Pose3d pivotPose3d;
  }

  public void setAngle(Rotation2d targetRotation);

  public void setSpeed(double speed);

  public void updateTuningValues();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
