// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Constants.ShooterConstants;

/** Add your docs here. */
public class IndexerIOKraken implements IndexerIO {

  TalonFX m_indexerMotor;
  TalonFX m_indexerMotorFollower;

  public IndexerIOKraken() {
    m_indexerMotor = new TalonFX(ShooterConstants.indexerMotorID, ShooterConstants.shooterCanbus);
    m_indexerMotorFollower =
        new TalonFX(ShooterConstants.indexerMotorFollowerID, ShooterConstants.shooterCanbus);
    m_indexerMotorFollower.setControl(
        new Follower(ShooterConstants.indexerMotorID, MotorAlignmentValue.Opposed));
  }

  public void setIndexerSpeed(double speed) {
    m_indexerMotor.set(speed);
  }

  public void stopIndexer() {

    m_indexerMotor.set(0);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    SmartDashboard.putNumber("IndexerSpeed", m_indexerMotor.getVelocity().getValueAsDouble() * 60);
    // need to figure out how many encoder units equals one full rotation
  }
}
