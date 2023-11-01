package org.sciborgs1155.lib.constants;

public record ElevatorFFConstants(double s, double g, double v, double a) {

  public ElevatorFFConstants(double s, double g, double v) {
    this(s, g, v, 0);
  }
}
