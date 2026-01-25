package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ProjectileSimulation {

  private final double gravity = 9.81;

  private Pose3d ballPose;
  private Translation3d velocity;

  public ProjectileSimulation(Pose3d startPose, Translation3d startVelocity) {
    this.ballPose = startPose;
    this.velocity = startVelocity;
  }

  public void update(double dt) {
    // Move
    var pos = ballPose.getTranslation();
    var newPos =
        new Translation3d(
            pos.getX() + velocity.getX() * dt,
            pos.getY() + velocity.getY() * dt,
            pos.getZ() + velocity.getZ() * dt);

    // Gravity acceleration
    velocity = new Translation3d(velocity.getX(), velocity.getY(), velocity.getZ() - gravity * dt);

    ballPose = new Pose3d(newPos, new Rotation3d());
  }

  public Pose3d getPose() {
    return ballPose;
  }
}
