// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.Constants.ShooterConstants;

/** Add your docs here. */
public class IndexerIOKraken implements IndexerIO {

  TalonFX m_rotationMotor;

  ShooterConstants m_constants;

  public IndexerIOKraken() {
    m_rotationMotor = new TalonFX(m_constants.indexerMotorID, "Turret");
  }

  public void setIndexerSpeed(double speed) {
    m_rotationMotor.set(speed);
  }

  public void stopIndexer() {

    m_rotationMotor.set(0);
  }

  public void updateInputs(IndexerIOInputs inputs) {

    // need to figure out how many encoder units equals one full rotation
  }
}
