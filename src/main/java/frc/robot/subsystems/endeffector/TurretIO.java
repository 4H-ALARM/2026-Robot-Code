// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.enums.TargetEnum;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    double turretRelativeAngleDegrees;
    double robotRelativeAngleDegrees;
    boolean isConnected;
    boolean isReady;
    public Pose2d turretPoseReal;
    TargetEnum target;
  }

  public void changeTarget(TargetEnum target);

  public default boolean isAimed() {
    return true;
  }

  public void toggleNeutral();

  public void resetEncoder();

  public void updateTuningValues();

  public void turretPeriodic(Pose2d currentRobotPose);

  public void manualcontrol(double controllerInput);

  public default void updateInputs(TurretIOInputs inputs) {}

  public Rotation2d getTargetTurretAngle(Pose2d pose);
}
