// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private IndexerIO m_indexer;
  private IndexerIOInputs m_indexerInputs;
  
  public Indexer(IndexerIO indexer) {
    m_indexer = indexer;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_indexer.updateInputs(m_indexerInputs);
  }

  public void setSpeed(double speed){
    m_indexer.setSpeed(speed);
  }

  public void setMotorAngle(Rotation2d targetRotation){
    m_indexer.setMotorAngle(targetRotation, m_indexerInputs);
  }
}
