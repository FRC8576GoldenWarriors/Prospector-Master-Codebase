// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.poseEstimation;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map.Entry;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class CollisionDetector {

  private final StatusSignal<LinearAcceleration> xAccelerationSignal;
  private final StatusSignal<LinearAcceleration> yAccelerationSignal;

  private final TimeInterpolatableBuffer<Double> xAccelerationInterpolatableBuffer;
  private final TimeInterpolatableBuffer<Double> yAccelerationInterpolatableBuffer;

  private final Notifier signalUpdater;

  private final Debouncer xSignalDebouncer;
  private final Debouncer ySignalDebouncer;

  @AutoLogOutput
  private boolean isColliding = false;

  @AutoLogOutput
  private double currentXJolt = 0;

  @AutoLogOutput
  private double currentYJolt = 0;

  private Timer timeSinceCollision;

  private final Time accelerationBufferHistorySeconds = Seconds.of(0.05);
  private final Time debounceSignalTimeSeconds = Seconds.of(0.01);

  private final double xJoltThreshold = 10;
  private final double yJoltThreshold = 10;

  public CollisionDetector(StatusSignal<LinearAcceleration> xAccelerationSignal, StatusSignal<LinearAcceleration> yAccelerationSignal, Frequency updateFrequency) {
    this.xAccelerationSignal = xAccelerationSignal;
    this.yAccelerationSignal = yAccelerationSignal;

    this.xAccelerationInterpolatableBuffer = TimeInterpolatableBuffer.createDoubleBuffer(accelerationBufferHistorySeconds.in(Seconds));
    this.yAccelerationInterpolatableBuffer = TimeInterpolatableBuffer.createDoubleBuffer(accelerationBufferHistorySeconds.in(Seconds));

    this.signalUpdater = new Notifier(this::updateInputs);

    this.xSignalDebouncer = new Debouncer(debounceSignalTimeSeconds.in(Seconds), DebounceType.kRising);
    this.ySignalDebouncer = new Debouncer(debounceSignalTimeSeconds.in(Seconds), DebounceType.kRising);

    this.timeSinceCollision = new Timer();

    signalUpdater.startPeriodic(updateFrequency);
  }

  public void updateInputs() {
    double currentTime = Timer.getFPGATimestamp();

    xAccelerationInterpolatableBuffer.addSample(currentTime, xAccelerationSignal.getValue().in(MetersPerSecondPerSecond));
    yAccelerationInterpolatableBuffer.addSample(currentTime, yAccelerationSignal.getValue().in(MetersPerSecondPerSecond));

    if(xAccelerationInterpolatableBuffer.getInternalBuffer().size() > 1) {
      Entry<Double, Double> start = xAccelerationInterpolatableBuffer.getInternalBuffer().firstEntry();
      Entry<Double, Double> end = xAccelerationInterpolatableBuffer.getInternalBuffer().lastEntry();

      currentXJolt = (end.getValue() - start.getValue()) / (end.getKey() - start.getKey());
    }

    if(yAccelerationInterpolatableBuffer.getInternalBuffer().size() > 1) {
      Entry<Double, Double> start = yAccelerationInterpolatableBuffer.getInternalBuffer().firstEntry();
      Entry<Double, Double> end = yAccelerationInterpolatableBuffer.getInternalBuffer().lastEntry();

      currentYJolt = (end.getValue() - start.getValue()) / (end.getKey() - start.getKey());
    }

    isColliding = xSignalDebouncer.calculate(Math.abs(currentXJolt) > xJoltThreshold) || ySignalDebouncer.calculate(Math.abs(currentYJolt) > yJoltThreshold);

    if(isColliding) {
      if(!timeSinceCollision.isRunning())
        timeSinceCollision.start();
      else
        timeSinceCollision.reset();
    }

  }

  public boolean isColliding() {
    return isColliding;
  }

  public double getCurrentXJolt() {
    return currentXJolt;
  }

  public double getCurrentYJolt() {
    return currentYJolt;
  }
}
