package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProjectileSimulation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class SimulateShotTrajectory extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private ProjectileSimulation sim;

  public SimulateShotTrajectory(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    var robotPose = drive.getPose();
    var start3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0.46, new Rotation3d());

    double v = 7; // m/s
    double pitch = Math.toRadians(35); // launch angle
    double vz = v * Math.sin(pitch);

    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

    // Calculate time of flight
    ProjectileSimulation tempSim = new ProjectileSimulation(start3d, new Translation3d(0, 0, vz));
    double timeOfFlight = tempSim.getTimeOfFlight(0.0);

    // Calculate future robot position when ball lands
    double futureX = robotPose.getX() + chassisSpeeds.vxMetersPerSecond * timeOfFlight;
    double futureY = robotPose.getY() + chassisSpeeds.vyMetersPerSecond * timeOfFlight;
    var futureRobotPose = new Pose2d(futureX, futureY, robotPose.getRotation());

    Logger.recordOutput("Trajectory/FutureRobotPose", futureRobotPose);

    // Get target angle using turret's calculation
    double turretAngle = -shooter.getTargetTurretAngle(robotPose).getRadians();
    double neededDirection = turretAngle + robotPose.getRotation().getRadians();

    double neededVx = v * Math.cos(pitch) * Math.cos(neededDirection);
    double neededVy = v * Math.cos(pitch) * Math.sin(neededDirection);

    // Shooter velocity = needed velocity - chassis velocity
    double shooterVx = neededVx - chassisSpeeds.vxMetersPerSecond;
    double shooterVy = neededVy - chassisSpeeds.vyMetersPerSecond;
    Logger.recordOutput(
        "shooterSpeed (x & y, not z)", Math.sqrt(shooterVx * shooterVx + shooterVy * shooterVy));

    // Total ball velocity (this adds back to get needed direction)
    double vx = shooterVx + chassisSpeeds.vxMetersPerSecond;
    double vy = shooterVy + chassisSpeeds.vyMetersPerSecond;

    this.sim = new ProjectileSimulation(start3d, new Translation3d(vx, vy, vz));

    // Entire trajectory
    ArrayList<Pose3d> trajectory = new ArrayList<>();
    ProjectileSimulation trajSim = new ProjectileSimulation(start3d, new Translation3d(vx, vy, vz));

    double dt = 0.02;
    int maxSteps = 200;
    for (int i = 0; i < maxSteps; i++) {
      Pose3d currentPose = trajSim.getPose();
      trajectory.add(currentPose);

      if (currentPose.getZ() <= 0) {
        break;
      }

      trajSim.update(dt);
    }

    Logger.recordOutput("Trajectory/ProjectileTrajectory", trajectory.toArray(new Pose3d[0]));
  }

  @Override
  public void execute() {
    sim.update(0.02);
    Pose3d pose = sim.getPose();
    Logger.recordOutput("Trajectory/ProjectilePose", pose);

    if (pose.getZ() <= 0) {
      cancel();
    }
  }
}
