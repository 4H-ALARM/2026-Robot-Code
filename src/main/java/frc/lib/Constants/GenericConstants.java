// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class GenericConstants {
  public static final Pose2d HUB_POSE2D = new Pose2d(4.7, 4.709, new Rotation2d());
  public static final Pose2d LEFTALLIANCE = new Pose2d();
  public static final Pose2d CENTERALLIANCE = new Pose2d();
  public static final Pose2d RIGHTALLIANCE = new Pose2d();

  public static final Translation2d turretPoseDifference = new Translation2d();
}
