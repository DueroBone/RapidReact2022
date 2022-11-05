// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Shooter extends SubsystemBase { 

  // 1 Neo motor for shooter
  private final static CANSparkMax shooter = new CANSparkMax(DriveConstants.shooterId, MotorType.kBrushless);

  private static RelativeEncoder shooterEncoder; // declare encoder for getting velocity - for Spark Max

  public enum ShooterState {SPINUP, SPINNING, READY, SPINDOWN, STOPPED};
  public static ShooterState state = ShooterState.STOPPED;
  
  private static int counter1 = 5; // for limiting display
  private static int counter2 = 5;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    shooter.restoreFactoryDefaults();   // for Spark Max Neo motors
    shooter.setInverted(true);          // set direction
    shooter.setIdleMode(IdleMode.kCoast); // set idle mode for Spark Max
    shooter.setSmartCurrentLimit(DriveConstants.kAmpsMax); // set current limit for Spark Max
    shooter.setOpenLoopRampRate(0.1);   // time for Spark Max to get to full speed

    REVLibError setVoltageComp = shooter.enableVoltageCompensation(12.0); // set to compensate for low battery
    if (setVoltageComp != REVLibError.kOk) {
      System.out.println("** shooter enableVoltageCompensation failed **");
    }

    shooterEncoder = shooter.getEncoder();    // declare the encoder
    shooterEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor); // set encoder constants
    shooterEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

    REVLibError burnFlashSuccess = shooter.burnFlash();   // burn settings into SparkMAX flash
    if (burnFlashSuccess != REVLibError.kOk) {
      System.out.println("** shooter burnFlash() failed **");
    }
    state = ShooterState.STOPPED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * set shooter motor speeds
   *
   * @param motorPercent  Speed in range [-1,1]
   */
  public static void setSpeed(double motorPercent) {

    double currentBatteryVoltage = RobotController.getBatteryVoltage();
    if (currentBatteryVoltage <= DriveConstants.kbatteryLowVoltage) {
      motorPercent = Math.min(1.0, motorPercent * 1.10);  // increase power if battery below cutoff
      System.out.println("*** low battery at " + currentBatteryVoltage);
    }
    //MotorPercent = motorPercent * 0.95;   // if need to limit speed
    shooter.set(motorPercent);
    if (counter1++ % 5 == 0) {
      double currentVelocity = getVelocity();
      System.out.println("**shooter power: "+String.format("%.3f  ", motorPercent)+" velocity: "+currentVelocity);
    }
  }

  /**
   * get shooter motor velocity
   */
  public static double getVelocity() {
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
    if (counter2++ % 5 == 0) {
      SmartDashboard.putNumber("Shooter velocity", velocity);
      double absVelocity = Math.abs(velocity);
      SmartDashboard.putNumber("Shooter velocity abs", absVelocity);
    }
    return velocity;
  }

  public static Command stop() {
    System.out.println("**in shooter stop");
    shooter.set(0.0);   // shut off shooter
    state = ShooterState.STOPPED;
    return null;
  }
}
