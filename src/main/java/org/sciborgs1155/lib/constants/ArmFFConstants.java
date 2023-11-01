package org.sciborgs1155.lib.constants;

public record ArmFFConstants(double s, double g, double v, double a) {

  public ArmFFConstants(double s, double g, double v) {
    this(s, g, v, 0);
  }
}
