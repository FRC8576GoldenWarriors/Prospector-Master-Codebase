// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.poseEstimation;

import java.util.Map.Entry;

import org.littletonrobotics.junction.AutoLogOutput;


import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class CollisionDetector extends SubsystemBase {
  private final double historySizeSeconds = 0.2;

  private final double xCollisionThreshold = 10;
  private final double ycollisionThreshold = 10;
  private final double zCollisionThreshold = 10;

  private final GyroIOInputs gyroIOInputs;

  private final TimeInterpolatableBuffer<Double> xJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);
  private final TimeInterpolatableBuffer<Double> yJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);
  private final TimeInterpolatableBuffer<Double> zJerkBuffer = TimeInterpolatableBuffer.createDoubleBuffer(historySizeSeconds);

  public CollisionDetector(GyroIOInputs gyroIOInputs) {
    this.gyroIOInputs = gyroIOInputs;
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    xJerkBuffer.addSample(currentTime, gyroIOInputs.xAccelerationMetersPerSecondPerSecond);
    yJerkBuffer.addSample(currentTime, gyroIOInputs.yAccelerationMetersPerSecondPerSecond);
    zJerkBuffer.addSample(currentTime, gyroIOInputs.zAccelerationMetersPerSecondPerSecond);
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
    if(hasZDerivative()) {
      return getZJerkMetersPerSecPerSecPerSec() > zCollisionThreshold;
    }
    return false;
  }
}
