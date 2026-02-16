// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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

  public default void changeTarget(TargetEnum target) {}

  public default boolean isAimed() {
    return true;
  }

  public default void toggleNeutral() {}

  public default void resetEncoder() {}

  public default void updateTuningValues() {}

  public default void turretPeriodic(Pose2d currentRobotPose) {}

  public default void manualcontrol(double controllerInput) {}

  public default void updateInputs(TurretIOInputs inputs) {}

  public default Rotation2d getTargetTurretAngle(Pose2d pose) {
    return new Rotation2d();
  }
}
