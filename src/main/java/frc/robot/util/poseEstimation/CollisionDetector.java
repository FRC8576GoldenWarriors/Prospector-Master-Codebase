// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.poseEstimation;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.List;
import java.util.Map.Entry;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;

public class CollisionDetector {
  private final double historySizeSeconds = 0.1;

  private final double xCollisionThreshold = 10;
  private final double ycollisionThreshold = 10;
  private final double zCollisionThreshold = 10;

  private final List<StatusSignal<LinearAcceleration>> robotAccelerationSignals;

  private final TimeInterpolatableBuffer<Double> xJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);
  private final TimeInterpolatableBuffer<Double> yJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);
  private final TimeInterpolatableBuffer<Double> zJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);


  public CollisionDetector(List<StatusSignal<LinearAcceleration>> robotAccelerationSignals, Frequency processRateHz) {
    if(robotAccelerationSignals.size() < 3) {
      throw new IllegalArgumentException("Missing acceleration signal. Only " + robotAccelerationSignals.size() + " signals found.");
    }
    this.robotAccelerationSignals = robotAccelerationSignals;
  }

  public void updateBuffer() {
    double currentTime = Timer.getFPGATimestamp();
    xJerkBuffer.addSample(currentTime, robotAccelerationSignals.get(0).getValue().in(MetersPerSecondPerSecond));
    yJerkBuffer.addSample(currentTime, robotAccelerationSignals.get(1).getValue().in(MetersPerSecondPerSecond));
    zJerkBuffer.addSample(currentTime, robotAccelerationSignals.get(2).getValue().in(MetersPerSecondPerSecond));
  }

  @AutoLogOutput
  public boolean hasXDerivative() {
    return xJerkBuffer.getInternalBuffer().size() > 1;
  }

  @AutoLogOutput
  public boolean hasYDerivative() {
    return yJerkBuffer.getInternalBuffer().size() > 1;
  }

  @AutoLogOutput
  public boolean hasZDerivative() {
    return zJerkBuffer.getInternalBuffer().size() > 1;
  }

  @AutoLogOutput
  public double getXJerkMetersPerSecPerSecPerSec() {
    Entry<Double, Double> finalValue = xJerkBuffer.getInternalBuffer().lastEntry();
    Entry<Double, Double> initialValue = xJerkBuffer.getInternalBuffer().firstEntry();

    if(finalValue == null || initialValue == null) {
      return 0;
    }

    return (finalValue.getValue() - initialValue.getValue()) / (finalValue.getKey() - initialValue.getKey());
  }

  @AutoLogOutput
  public double getYJerkMetersPerSecPerSecPerSec() {
    Entry<Double, Double> finalValue = yJerkBuffer.getInternalBuffer().lastEntry();
    Entry<Double, Double> initialValue = yJerkBuffer.getInternalBuffer().firstEntry();

    if(finalValue == null || initialValue == null) {
      return 0;
    }

    return (finalValue.getValue() - initialValue.getValue()) / (finalValue.getKey() - initialValue.getKey());
  }

  @AutoLogOutput
  public double getZJerkMetersPerSecPerSecPerSec() {
    Entry<Double, Double> finalValue = zJerkBuffer.getInternalBuffer().lastEntry();
    Entry<Double, Double> initialValue = zJerkBuffer.getInternalBuffer().firstEntry();

    if(finalValue == null || initialValue == null) {
      return 0;
    }

    return (finalValue.getValue() - initialValue.getValue()) / (finalValue.getKey() - initialValue.getKey());
  }

  @AutoLogOutput
  public boolean isColliding() {
    if(hasXDerivative() && hasYDerivative() && hasZDerivative()) {
      return (getXJerkMetersPerSecPerSecPerSec() > xCollisionThreshold) && (getYJerkMetersPerSecPerSecPerSec() > ycollisionThreshold) && (getZJerkMetersPerSecPerSecPerSec() > zCollisionThreshold);
    }
    return false;
  }

  //Needs to be tested and tuned
  public boolean isDrivingOverBump() {
    return robotAccelerationSignals.get(2).getValue().in(MetersPerSecondPerSecond) > zCollisionThreshold;
  }
}
