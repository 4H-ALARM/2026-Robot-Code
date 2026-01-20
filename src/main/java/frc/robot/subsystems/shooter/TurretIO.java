// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.enums.TargetEnum;

/** Add your docs here. */
public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        Rotation2d turretRelativeAngleDegrees;
        Rotation2d robotRelativeAngleDegrees;
        boolean isConnected;
        boolean isReady;
        TargetEnum target;
    }

    public default void changeTarget(TargetEnum target) {}

    public default boolean isAimed() {return true;}

    public default void updateInputs(TurretIOInputs inputs) {}
}
