// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShooterPID extends SubsystemBase { 

  // 1 Neo motor for shooter
  private CANSparkMax shooter = new CANSparkMax(DriveConstants.shooterId, MotorType.kBrushless);

  private RelativeEncoder shooterEncoder; // declare encoder for getting velocity - for Spark Max
  private SparkMaxPIDController shooterPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double targetVelocity;

  public enum ShooterState {SPINUP, SPINNING, READY, SPINDOWN, STOPPED};
  public ShooterState state = ShooterState.STOPPED;

  private static int counter1 = 5; // for limiting display

  /**
   * Creates a new Shooter.
   */
  public ShooterPID() {

    shooter.restoreFactoryDefaults();   // for Spark Max Neo motors
    shooter.setInverted(true); // set direction
    shooter.setIdleMode(IdleMode.kCoast); // set idle mode for Spark Max
    shooter.setSmartCurrentLimit(DriveConstants.kAmpsMax); // set current limit for Spark Max
    shooter.setOpenLoopRampRate(0.75);   // time for Spark Max to get to full speed

    REVLibError setVoltageComp = shooter.enableVoltageCompensation(12.0); // set to compensate for low battery
    if (setVoltageComp != REVLibError.kOk) {
      System.out.println("** shooter enableVoltageCompensation failed **");
    }

    shooterEncoder = shooter.getEncoder();    // declare the encoder
    shooterEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor); // set encoder constants
    shooterEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

    shooterPidController = shooter.getPIDController();   // uses builtin PID controller

    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
    // PID coefficients
    kP = 0.0002;  // was 6e-5 from REV example 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000175;   // using 1/5700, was 0.000015 from REV example
    kMaxOutput = 1; 
    kMinOutput = 0;   // 0 so cant reverse, was -1 from REV example
    maxRPM = 5500;

    // set PID coefficients
    shooterPidController.setP(kP);
    shooterPidController.setI(kI);
    shooterPidController.setD(kD);
    shooterPidController.setIZone(kIz);
    shooterPidController.setFF(kFF);
    shooterPidController.setOutputRange(kMinOutput, kMaxOutput);
  
    /*
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    */
      
    targetVelocity = 0.0;    // setpoint initially zero, will be set below
    state = ShooterState.STOPPED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (counter1++ % 5 == 0) {
      SmartDashboard.putNumber("Shooter SetPoint", targetVelocity);
      SmartDashboard.putNumber("Shooter Velocity", getVelocity());
      SmartDashboard.putString("Shooter State", state.toString());
    }
  }

  /**
   * set shooter motor speeds
   *
   * @param motorPercent  Speed in range [-1,1]
   */
  public void setSpeed(double motorPercent) {

    shooter.set(motorPercent);

    if (targetVelocity == 0.0) {  // set new PID setpoint if none yet
      setVelocity(maxRPM);
    }
  }

  /**
   * get shooter motor velocity
   */
  public double getVelocity() {
    double velocity = shooterEncoder.getVelocity();    // get current velocity
    if (velocity >= 4000) {
      state = ShooterState.READY;
    } else if (velocity >= 1000) {
      state = ShooterState.SPINNING;
    }  else if (velocity >= 50) {
      state = ShooterState.SPINUP;
    } else {
      state = ShooterState.STOPPED;
    }
    return velocity;
  }

  /**
   * set shooter motor velocity for PID control
   */
  public void setVelocity(double velocity) {
    targetVelocity = velocity;
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    shooterPidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
    System.out.println("setting PID setpoint to " + targetVelocity);
    state = ShooterState.SPINUP;
  }

  public void stop() {
    System.out.println("**in shooter stop");
    targetVelocity = 0.0;
    setVelocity(targetVelocity);    // set PID setpoint to 0
    shooter.set(0.0);   // shut off shooter
    state = ShooterState.STOPPED;
  }
}
