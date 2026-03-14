package frc.robot.util;

public class WarriorBangBangController {
  private double currentPosition;
  private double goalPosition;
  private double tolerance;
  private double softStop;

  public WarriorBangBangController(double tolerance, double softStop) {
    this.tolerance = tolerance;
    this.softStop = softStop;
  }

  public double calculate(double currentPosition, double goalPosition) {
    this.currentPosition = currentPosition;
    this.goalPosition = goalPosition;
    double motorOutput = 0.0;
    if (currentPosition < goalPosition) { // pivot going down, robot climbing up
      motorOutput = 1.0;

      if (Math.abs(currentPosition - goalPosition) <= tolerance) {
        motorOutput = 0.0;
      }
    } else { // pivot going up, robot going down
      motorOutput = -1.0;
      if (Math.abs(currentPosition - goalPosition) <= tolerance) {
        motorOutput = 0.0;
      }
    }
    if (currentPosition > softStop) {
      motorOutput = 0.0;
    }
    return motorOutput;
  }

  public boolean nearSetpoint() {
    return Math.abs(currentPosition - goalPosition) <= tolerance;
  }
}
