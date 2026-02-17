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

  // Add this method
  public double getTimeOfFlight(double targetHeight) {
    double z0 = ballPose.getZ();
    double vz0 = velocity.getZ();

    double a = 0.5 * gravity;
    double b = -vz0;
    double c = z0 - targetHeight;

    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
      return -1;
    }

    double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
    double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);

    // Return the LARGER positive time (when ball lands, not when it's rising)
    if (t1 > 0 && t2 > 0) {
      return Math.max(t1, t2); // Changed from Math.min to Math.max
    } else if (t1 > 0) {
      return t1;
    } else if (t2 > 0) {
      return t2;
    }

    return -1;
  }
}
