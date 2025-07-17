// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final SparkMax mClawMotor;
  public static final double DIST_TO_HAVE_CORAL = 0.1;
  private static final double DEBOUNCE_TIME = 0.15;
  private final CANrange mBeamBreaker;
  private final Debouncer beamDebouncer;
  
  /** Creates a new Claw. */
  public Claw() {
    mClawMotor = new SparkMax(1, MotorType.kBrushless);
    mBeamBreaker = new CANrange(0);
    beamDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kBoth);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Claw/hasCoral", false);
    SmartDashboard.putNumber("Claw/motorSpeed", mClawMotor.get());
  }

  public boolean hasCoral() {
    boolean broken = mBeamBreaker.getDistance().getValueAsDouble() <= DIST_TO_HAVE_CORAL;
    return beamDebouncer.calculate(broken);
  }

  public void setSpeed(double speed) {
    mClawMotor.set(speed);
  }

  public void stop() {
    mClawMotor.stopMotor();
  }
}
