// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    double indexerSpeed;
  }
  ;

<<<<<<< HEAD
  public default void changeTarget(TargetEnum target) {}

  public default void updateTuningValues() {}

  public default void updateInputs(ShooterIOInputs inputs) {}
=======
  public void setIndexerSpeed(double indexerSpeed);

  public void stopIndexer();
>>>>>>> 37ba14d (fixed build time error)
}
