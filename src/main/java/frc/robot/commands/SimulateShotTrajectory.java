package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProjectileSimulation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class SimulateShotTrajectory extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private ProjectileSimulation sim;
  private StructPublisher<Pose3d> publisher;

  public SimulateShotTrajectory(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;

    this.publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/ProjectilePose", Pose3d.struct)
            .publish();

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    var robotPose = drive.getPose();
    var start3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0.46, new Rotation3d());

    double v = 100; // m/s
    double pitch = Math.toRadians(35); // launch angle
    double yaw =
        shooter.getTargetTurretAngle(drive.getPose()).getRadians(); // turret angle in radians

    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

    double vx = v * Math.cos(pitch) * Math.cos(yaw) + chassisSpeeds.vxMetersPerSecond;
    double vy = v * Math.cos(pitch) * Math.sin(yaw) + chassisSpeeds.vyMetersPerSecond;
    double vz = v * Math.sin(pitch);

    this.sim = new ProjectileSimulation(start3d, new Translation3d(vx, vy, vz));
  }

  @Override
  public void execute() {
    sim.update(0.02); // 20 ms timestep
    Pose3d pose = sim.getPose();
    publisher.set(pose);

    if (pose.getZ() <= 0) {
      cancel();
    }
  }
}
