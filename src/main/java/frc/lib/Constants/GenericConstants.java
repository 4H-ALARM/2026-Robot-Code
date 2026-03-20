// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class GenericConstants {
  // field constants
  public static final double MIDY = Inches.of(317.69 / 2).in(Meters);
  public static final double REDHUBX = Inches.of(182.11).in(Meters);
  public static final double MIDX = Inches.of(325.61).in(Meters);
  public static final double BLUEHUBX = Inches.of(651.22-182.11).in(Meters);

  // field poses
  public static final double MIDALLIANCE = REDHUBX / 2;
  public static final Translation3d REDHUB_POSE3D =
      new Translation3d(REDHUBX, MIDY, Inches.of(72).in(Meters));
  public static final Translation3d BLUEHUB_POSE3D = 
      new Translation3d(BLUEHUBX, MIDY, Inches.of(72).in(Meters));
  public static final Translation3d LEFTALLIANCE = new Translation3d(MIDALLIANCE, MIDY * .5, 0);
  public static final Translation3d CENTERALLIANCE = new Translation3d(MIDALLIANCE, MIDY, 0);
  public static final Translation3d RIGHTALLIANCE = new Translation3d(MIDALLIANCE, MIDY * 1.5, 0);
  public static final Translation3d LEFTNEUTRAL = new Translation3d(MIDX, MIDY * .5, 0);
  public static final Translation3d RIGHTNEUTRAL = new Translation3d(MIDX, MIDY, 0);
  public static final Translation3d MIDDLENEUTRAL = new Translation3d(MIDX, MIDY * 1.5, 0);

  public static final Translation2d turretPoseDifference = new Translation2d();
}
