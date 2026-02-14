// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

/** Add your docs here. */
public class IntakeConstants {

    public static final double rotationGearRatio = 48.0/11.0;

    // Motor IDs
    public static final int rotationMotorID = -1;
    public static final int rotationMotorFollowID = -1;
    public static final int intakingMotorID = -1;    
    // PID tuning for the intake
    public static final double angleMotorkp = 1;
    public static final double angleMotorki = 0;
    public static final double angleMotorkd = 0;
}
