// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

/** Add your docs here. */
public interface SpindexerIO {

    public class SpindexerIOInputs {
        public boolean isSpindexerMotorConnected = false;
        public double speed;
        
    }
    
    public default void setSpindexerSpeed(double speed) {}

    /// Not currently working
    public default void activateMotor() {}

    public default void stopMotor() {}

    public default void updateInputs(SpindexerIOInputs inputs) {}
    
}
