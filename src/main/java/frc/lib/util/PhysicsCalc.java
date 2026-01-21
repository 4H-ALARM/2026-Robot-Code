// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/**
 * Simple projectile solver with drag + Magnus lift for a 7 in ball (~0.5 lb).
 *
 * <p>Assumes backspin so Magnus force is upward. Target height defaults to launch height.
 */
public final class PhysicsCalc {
  private PhysicsCalc() {}

  private static final double GRAVITY_MPS2 = 9.80665;
  private static final double AIR_DENSITY_KG_M3 = 1.225;

  private static final double BALL_DIAMETER_M = 0.1778; // 7 inches
  private static final double BALL_RADIUS_M = BALL_DIAMETER_M / 2.0;
  private static final double BALL_AREA_M2 = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M;
  private static final double BALL_MASS_KG = 0.2268; // 0.5 lb

  private static final double DRAG_COEFF = 0.47;
  private static final double DEFAULT_SPIN_RPM = 2500.0;
  private static final double TOP_WHEEL_DIAMETER_M = 0.0508; // 2 inches
  private static final double BOTTOM_WHEEL_DIAMETER_M = 0.0762; // 3 inches

  private static final double MIN_ANGLE_DEG = 5.0;
  private static final double MAX_ANGLE_DEG = 85.0;
  private static final double SEARCH_STEP_DEG = 1.0;
  private static final int ROOT_ITERATIONS = 30;
  private static final double HEIGHT_TOLERANCE_M = 0.02;
  private static final double MAX_SIM_TIME_S = 10.0;
  private static final double TIME_STEP_S = 0.005;

  public static double calculateLaunchAngleDeg(double initialSpeedMps, double distanceMeters) {
    return calculateLaunchAngleDeg(
        initialSpeedMps, distanceMeters, 0.0, DEFAULT_SPIN_RPM, 75.0);
  }

  public static double calculateLaunchAngleDegFromWheels(
      double initialSpeedMps,
      double distanceMeters,
      double targetHeightMeters,
      double topWheelRpm,
      double bottomWheelRpm,
      double minDescentAngleDeg) {
    double spinRpm = estimateSpinRpmFromWheels(topWheelRpm, bottomWheelRpm);
    return calculateLaunchAngleDeg(
        initialSpeedMps, distanceMeters, targetHeightMeters, spinRpm, minDescentAngleDeg);
  }

  public static double calculateLaunchAngleDeg(
      double initialSpeedMps,
      double distanceMeters,
      double targetHeightMeters,
      double spinRpm,
      double minDescentAngleDeg) {
    if (initialSpeedMps <= 0.0 || distanceMeters <= 0.0) {
      return Double.NaN;
    }

    TrajectorySample prevSample =
        simulateToDistance(
            initialSpeedMps, Math.toRadians(MIN_ANGLE_DEG), distanceMeters, spinRpm);
    if (prevSample == null) {
      return Double.NaN;
    }
    double prevError = prevSample.yAtDistanceMeters - targetHeightMeters;
    double prevAngleDeg = MIN_ANGLE_DEG;

    for (double angleDeg = MIN_ANGLE_DEG + SEARCH_STEP_DEG;
        angleDeg <= MAX_ANGLE_DEG;
        angleDeg += SEARCH_STEP_DEG) {
      TrajectorySample sample =
          simulateToDistance(
              initialSpeedMps, Math.toRadians(angleDeg), distanceMeters, spinRpm);
      if (sample == null) {
        continue;
      }

      double error = sample.yAtDistanceMeters - targetHeightMeters;
      if (Math.abs(error) <= HEIGHT_TOLERANCE_M) {
        double descent = descentAngleDeg(sample.vxAtDistanceMps, sample.vyAtDistanceMps);
        if (descent >= minDescentAngleDeg) {
          return angleDeg;
        }
      }

      if (Math.signum(prevError) != Math.signum(error)) {
        double candidate =
            solveForAngle(
                initialSpeedMps,
                distanceMeters,
                targetHeightMeters,
                spinRpm,
                prevAngleDeg,
                angleDeg);
        if (!Double.isNaN(candidate)) {
          TrajectorySample candidateSample =
              simulateToDistance(
                  initialSpeedMps, Math.toRadians(candidate), distanceMeters, spinRpm);
          if (candidateSample != null) {
            double descent =
                descentAngleDeg(candidateSample.vxAtDistanceMps, candidateSample.vyAtDistanceMps);
            if (descent >= minDescentAngleDeg) {
              return candidate;
            }
          }
        }
      }

      prevError = error;
      prevAngleDeg = angleDeg;
    }

    return Double.NaN;
  }

  private static double solveForAngle(
      double initialSpeedMps,
      double distanceMeters,
      double targetHeightMeters,
      double spinRpm,
      double lowAngleDeg,
      double highAngleDeg) {
    double low = Math.toRadians(lowAngleDeg);
    double high = Math.toRadians(highAngleDeg);
    TrajectorySample lowSample =
        simulateToDistance(initialSpeedMps, low, distanceMeters, spinRpm);
    TrajectorySample highSample =
        simulateToDistance(initialSpeedMps, high, distanceMeters, spinRpm);
    if (lowSample == null || highSample == null) {
      return Double.NaN;
    }

    double lowError = lowSample.yAtDistanceMeters - targetHeightMeters;
    double highError = highSample.yAtDistanceMeters - targetHeightMeters;
    if (Math.signum(lowError) == Math.signum(highError)) {
      return Double.NaN;
    }

    for (int i = 0; i < ROOT_ITERATIONS; i++) {
      double mid = (low + high) / 2.0;
      TrajectorySample midSample =
          simulateToDistance(initialSpeedMps, mid, distanceMeters, spinRpm);
      if (midSample == null) {
        return Double.NaN;
      }
      double midError = midSample.yAtDistanceMeters - targetHeightMeters;
      if (Math.abs(midError) <= HEIGHT_TOLERANCE_M) {
        return Math.toDegrees(mid);
      }
      if (Math.signum(midError) == Math.signum(lowError)) {
        low = mid;
        lowError = midError;
      } else {
        high = mid;
        highError = midError;
      }
    }

    return Math.toDegrees((low + high) / 2.0);
  }

  private static TrajectorySample simulateToDistance(
      double initialSpeedMps, double launchAngleRad, double distanceMeters, double spinRpm) {
    double vx = initialSpeedMps * Math.cos(launchAngleRad);
    double vy = initialSpeedMps * Math.sin(launchAngleRad);
    double x = 0.0;
    double y = 0.0;
    double time = 0.0;

    double prevX = 0.0;
    double prevY = 0.0;
    double prevVx = vx;
    double prevVy = vy;

    while (time <= MAX_SIM_TIME_S) {
      double speed = Math.hypot(vx, vy);
      double drag = 0.0;
      double dragAx = 0.0;
      double dragAy = 0.0;
      if (speed > 1e-6) {
        drag = 0.5 * AIR_DENSITY_KG_M3 * speed * speed * DRAG_COEFF * BALL_AREA_M2;
        dragAx = -drag * vx / (speed * BALL_MASS_KG);
        dragAy = -drag * vy / (speed * BALL_MASS_KG);
      }

      double liftCoeff = liftCoefficient(speed, spinRpm);
      double lift = 0.5 * AIR_DENSITY_KG_M3 * speed * speed * liftCoeff * BALL_AREA_M2;
      double liftAy = lift / BALL_MASS_KG;

      double ax = dragAx;
      double ay = -GRAVITY_MPS2 + dragAy + liftAy;

      // Save previous state BEFORE updating
      prevX = x;
      prevY = y;
      prevVx = vx;
      prevVy = vy;

      // Update velocities
      vx += ax * TIME_STEP_S;
      vy += ay * TIME_STEP_S;

      // Update positions
      x += vx * TIME_STEP_S;
      y += vy * TIME_STEP_S;

      if (x >= distanceMeters) {
        double alpha = (distanceMeters - prevX) / (x - prevX);
        double yAtDistance = prevY + alpha * (y - prevY);
        double vxAtDistance = prevVx + alpha * (vx - prevVx);
        double vyAtDistance = prevVy + alpha * (vy - prevVy);
        return new TrajectorySample(yAtDistance, vxAtDistance, vyAtDistance);
      }

      time += TIME_STEP_S;
      if (y < -2.0 * BALL_DIAMETER_M && vy < 0.0) {
        break;
      }
    }

    return null;
  }

  private static double liftCoefficient(double speedMps, double spinRpm) {
    if (speedMps <= 1e-6 || Math.abs(spinRpm) <= 1e-6) {
      return 0.0;
    }
    double omega = spinRpm * 2.0 * Math.PI / 60.0;
    double spinRatio = BALL_RADIUS_M * Math.abs(omega) / speedMps;
    double clMag = 1.2 * spinRatio / (1.0 + spinRatio);
    double cl = clamp(clMag, 0.0, 0.6);
    return Math.signum(spinRpm) * cl;
  }

  private static double descentAngleDeg(double vx, double vy) {
    if (vx <= 0.0 || vy >= 0.0) {
      return 0.0;
    }
    return Math.toDegrees(Math.atan2(-vy, vx));
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public static double estimateSpinRpmFromWheels(double topWheelRpm, double bottomWheelRpm) {
    return estimateSpinRpmFromWheels(
        topWheelRpm, bottomWheelRpm, TOP_WHEEL_DIAMETER_M, BOTTOM_WHEEL_DIAMETER_M);
  }

  public static double estimateSpinRpmFromWheels(
      double topWheelRpm,
      double bottomWheelRpm,
      double topWheelDiameterM,
      double bottomWheelDiameterM) {
    double topSurfaceSpeed = rpmToSurfaceSpeedMps(topWheelRpm, topWheelDiameterM);
    double bottomSurfaceSpeed = rpmToSurfaceSpeedMps(bottomWheelRpm, bottomWheelDiameterM);
    double spinRadPerSec = (topSurfaceSpeed - bottomSurfaceSpeed) / (2.0 * BALL_RADIUS_M);
    return spinRadPerSec * 60.0 / (2.0 * Math.PI);
  }

  private static double rpmToSurfaceSpeedMps(double rpm, double diameterM) {
    return (rpm / 60.0) * (Math.PI * diameterM);
  }

  private static final class TrajectorySample {
    private final double yAtDistanceMeters;
    private final double vxAtDistanceMps;
    private final double vyAtDistanceMps;

    private TrajectorySample(double yAtDistanceMeters, double vxAtDistanceMps, double vyAtDistanceMps) {
      this.yAtDistanceMeters = yAtDistanceMeters;
      this.vxAtDistanceMps = vxAtDistanceMps;
      this.vyAtDistanceMps = vyAtDistanceMps;
    }
  }
}
