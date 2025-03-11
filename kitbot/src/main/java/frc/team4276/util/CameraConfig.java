package frc.team4276.util;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
  public final String name;
  public final Transform3d robotToCamera;
  public final double stdDevFactor;

  public CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.stdDevFactor = stdDevFactor;
  }
}
