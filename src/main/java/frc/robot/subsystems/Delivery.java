// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Delivery extends SubsystemBase {

  // 1 Neo motor for delivery
  private final static CANSparkMax delivery = new CANSparkMax(DriveConstants.deliveryId, MotorType.kBrushless);

  //private static int counter = 5; // for limiting display

  /** Creates a new Delivery. */
  public Delivery() {

    delivery.restoreFactoryDefaults();   // for Spark Max Neo motors
    delivery.setInverted(true); // set direction
    delivery.setIdleMode(IdleMode.kCoast); // set idle mode for Spark Max
    delivery.setSmartCurrentLimit(DriveConstants.kAmpsMax); // set current limit for Spark Max
    delivery.setClosedLoopRampRate(0.3);  // time in seconds to go from 0 to full throttle
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean isRunning = false;
    double tempCurrent = delivery.getOutputCurrent();
    if (tempCurrent > 0.01 ) {
      isRunning = true;
    } else {
      isRunning = false;
    }
    SmartDashboard.putBoolean("Delivery", isRunning);
  }
  
  /**
   * set spinner motor speeds
   *
   * @param motorPercent  Speed in range [-1,1]
   */
  public static void setSpeed(double motorPercent) {
    //if (counter++ % 5 == 0) { System.out.println("**delivery power: "+String.format("%.3f  ", motorPercent));};

    //MotorPercent = motorPercent * 0.75;   // if need to limit speed
    delivery.set(motorPercent);
  }

  public static Command stop() {
    //System.out.println("**in delivery stop");
    setSpeed(0.0);
    return null;
  }
}
