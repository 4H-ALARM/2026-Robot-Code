// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */

public interface IndexerIO{

public class IndexerIOInputs {
public boolean isIndexerMotorConnected = false;
double speed;


}

public default void setSpeed(double speed){}

public default void setMotorAngle(Rotation2d targetRotation, IndexerIOInputs inputs){}

public default void updateInputs(IndexerIOInputs inputs){}
}