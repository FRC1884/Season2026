package org.Griffins1884.frc2026.simulation.engine;

import edu.wpi.first.math.geometry.Rotation3d;

/** Deterministic math helpers for the local physics engine. */
public final class PhysicsMath {
  private PhysicsMath() {}

  public static final double EPSILON = 1e-9;

  public static Vec3 clamp(Vec3 value, double maxMagnitude) {
    double norm = value.norm();
    if (norm <= maxMagnitude || norm < EPSILON) {
      return value;
    }
    return value.scale(maxMagnitude / norm);
  }

  public record Vec3(double x, double y, double z) {
    public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);
    public static final Vec3 UNIT_X = new Vec3(1.0, 0.0, 0.0);
    public static final Vec3 UNIT_Y = new Vec3(0.0, 1.0, 0.0);
    public static final Vec3 UNIT_Z = new Vec3(0.0, 0.0, 1.0);

    public Vec3 add(Vec3 other) {
      return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    public Vec3 subtract(Vec3 other) {
      return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    public Vec3 scale(double scalar) {
      return new Vec3(x * scalar, y * scalar, z * scalar);
    }

    public Vec3 divide(double scalar) {
      if (Math.abs(scalar) < EPSILON) {
        return ZERO;
      }
      return scale(1.0 / scalar);
    }

    public double dot(Vec3 other) {
      return (x * other.x) + (y * other.y) + (z * other.z);
    }

    public Vec3 cross(Vec3 other) {
      return new Vec3(
          (y * other.z) - (z * other.y),
          (z * other.x) - (x * other.z),
          (x * other.y) - (y * other.x));
    }

    public double normSquared() {
      return dot(this);
    }

    public double norm() {
      return Math.sqrt(normSquared());
    }

    public Vec3 normalize() {
      double norm = norm();
      if (norm < EPSILON) {
        return ZERO;
      }
      return divide(norm);
    }

    public Vec3 hadamard(Vec3 other) {
      return new Vec3(x * other.x, y * other.y, z * other.z);
    }

    public Vec3 negate() {
      return new Vec3(-x, -y, -z);
    }

    public double get(int axis) {
      return switch (axis) {
        case 0 -> x;
        case 1 -> y;
        default -> z;
      };
    }
  }

  public record Quat(double w, double x, double y, double z) {
    public static final Quat IDENTITY = new Quat(1.0, 0.0, 0.0, 0.0);

    public Quat normalize() {
      double norm = Math.sqrt((w * w) + (x * x) + (y * y) + (z * z));
      if (norm < EPSILON) {
        return IDENTITY;
      }
      return new Quat(w / norm, x / norm, y / norm, z / norm);
    }

    public Quat multiply(Quat other) {
      return new Quat(
          (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z),
          (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y),
          (w * other.y) - (x * other.z) + (y * other.w) + (z * other.x),
          (w * other.z) + (x * other.y) - (y * other.x) + (z * other.w));
    }

    public Quat conjugate() {
      return new Quat(w, -x, -y, -z);
    }

    public Vec3 rotate(Vec3 vector) {
      Quat pure = new Quat(0.0, vector.x(), vector.y(), vector.z());
      Quat rotated = multiply(pure).multiply(conjugate());
      return new Vec3(rotated.x, rotated.y, rotated.z);
    }

    public Quat integrateAngularVelocity(Vec3 angularVelocityRadPerSec, double dtSeconds) {
      double halfDt = 0.5 * dtSeconds;
      Quat omega =
          new Quat(
              0.0,
              angularVelocityRadPerSec.x() * halfDt,
              angularVelocityRadPerSec.y() * halfDt,
              angularVelocityRadPerSec.z() * halfDt);
      return new Quat(
              w + ((-x * omega.x) - (y * omega.y) - (z * omega.z)),
              x + ((w * omega.x) + (y * omega.z) - (z * omega.y)),
              y + ((w * omega.y) - (x * omega.z) + (z * omega.x)),
              z + ((w * omega.z) + (x * omega.y) - (y * omega.x)))
          .normalize();
    }

    public Rotation3d toRotation3d() {
      return new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(w, x, y, z));
    }

    public double yawRadians() {
      return Math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))));
    }

    public Vec3 axisX() {
      return rotate(Vec3.UNIT_X);
    }

    public Vec3 axisY() {
      return rotate(Vec3.UNIT_Y);
    }

    public Vec3 axisZ() {
      return rotate(Vec3.UNIT_Z);
    }

    public static Quat fromYawPitchRoll(double yaw, double pitch, double roll) {
      double cy = Math.cos(yaw * 0.5);
      double sy = Math.sin(yaw * 0.5);
      double cp = Math.cos(pitch * 0.5);
      double sp = Math.sin(pitch * 0.5);
      double cr = Math.cos(roll * 0.5);
      double sr = Math.sin(roll * 0.5);
      return new Quat(
              (cr * cp * cy) + (sr * sp * sy),
              (sr * cp * cy) - (cr * sp * sy),
              (cr * sp * cy) + (sr * cp * sy),
              (cr * cp * sy) - (sr * sp * cy))
          .normalize();
    }
  }
}
