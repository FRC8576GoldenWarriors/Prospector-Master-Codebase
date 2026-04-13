// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.poseEstimation.collisionDetection;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;


import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;

public class CollisionDetector {

  private final StatusSignal<LinearAcceleration> xAccelerationSignal;
  private final StatusSignal<LinearAcceleration> yAccelerationSignal;

  private final LinearAcceleration collisionThreshold = MetersPerSecondPerSecond.of(2);

  private final Notifier signalUpdater;

  @AutoLogOutput
  private boolean isColliding = false;

  @AutoLogOutput
  private CollisionData currentCollisionVector = CollisionData.zero();

  public CollisionDetector(StatusSignal<LinearAcceleration> xAccelerationSignal, StatusSignal<LinearAcceleration> yAccelerationSignal, Frequency updateFrequency) {
    this.xAccelerationSignal = xAccelerationSignal;
    this.yAccelerationSignal = yAccelerationSignal;

    xAccelerationSignal.setUpdateFrequency(updateFrequency);
    yAccelerationSignal.setUpdateFrequency(updateFrequency);
    this.signalUpdater = new Notifier(this::updateInputs);

    signalUpdater.startPeriodic(updateFrequency);
  }

  public void updateInputs() {
    BaseStatusSignal.refreshAll(xAccelerationSignal, yAccelerationSignal);

    currentCollisionVector = CollisionData.of2DData(
      xAccelerationSignal.getValue(),
      yAccelerationSignal.getValue(),
      Seconds.of(xAccelerationSignal.getTimestamp().getTime()));

    if(currentCollisionVector.getNorm2d().gte(collisionThreshold)) {
      isColliding = true;
    } else {
      isColliding = false;
    }
  }

  public Pair<Double, Double> getCollisionSTDDevs() {
    return Pair.of(Double.valueOf(0), Double.valueOf(0));
  }

  public record CollisionData(
    LinearAcceleration xAcceleration,
    LinearAcceleration yAcceleration,
    LinearAcceleration zAcceleration,
    Time timeStamp) {

    public static CollisionData of2DData(LinearAcceleration xAcceleration, LinearAcceleration yAcceleration, Time timeStamp) {
      return new CollisionData(xAcceleration, yAcceleration, MetersPerSecondPerSecond.zero(), timeStamp);
    }

    public static CollisionData zero() {
      return new CollisionData(MetersPerSecondPerSecond.zero(), MetersPerSecondPerSecond.zero(), MetersPerSecondPerSecond.zero(), Seconds.zero());
    }

    public Translation3d getCollisionVector3d() {
      return new Translation3d(
        this.xAcceleration.in(MetersPerSecondPerSecond),
        this.yAcceleration.in(MetersPerSecondPerSecond),
        this.zAcceleration.in(MetersPerSecondPerSecond));
    }

    public Translation2d getCollisionVector2d() {
      return getCollisionVector3d().toTranslation2d();
    }

    public LinearAcceleration getNorm2d() {
      return MetersPerSecondPerSecond.of(getCollisionVector2d().getNorm());
    }

    public LinearAcceleration getNorm3d() {
      return MetersPerSecondPerSecond.of(getCollisionVector3d().getNorm());
    }
  }
}
