// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean pivotLeadMotorConnected = false;
    boolean pivotFollowerMotorConnected = false;
    boolean intakeMotorConnected = false;

    double angleMotorCounts = 0;
    double speed = 0;
    Pose3d pivotPose3d = new Pose3d();
  }

  public void setAngle(double targetRotation);

  public void setSpeed(double speed);

  public void updateTuningValues();

  public default void updateInputs(IntakeIOInputs inputs) {}

  public void resetEncoder();
}
