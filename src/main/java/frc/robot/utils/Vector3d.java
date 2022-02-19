package frc.robot.utils;

/** This is a 3D vector struct that supports basic vector operations. */
public class Vector3d {
  @SuppressWarnings("MemberName")
  public double x;

  @SuppressWarnings("MemberName")
  public double y;

  @SuppressWarnings("MemberName")
  public double z;

  public Vector3d() {
  }

  public Vector3d(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  // TODO: Add 3d matrix & vector mathmatics
}
