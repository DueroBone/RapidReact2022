// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Climber extends SubsystemBase {

  // 1 Neo motor for climber
  private final static CANSparkMax climber = new CANSparkMax(DriveConstants.climberId, MotorType.kBrushless);

  private static int counter = 2; // for limiting display

   /** Creates a new Climber */
  public Climber() {
    
    climber.restoreFactoryDefaults();   // for Spark Max Neo motors
    climber.setInverted(true); // set direction
    climber.setIdleMode(IdleMode.kCoast); // set idle mode for Spark Max
    climber.setSmartCurrentLimit(DriveConstants.kAmpsMax); // set current limit for Spark Max
    climber.setClosedLoopRampRate(0.5);  // time in seconds to go from 0 to full throttle
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * set spinner motor speeds
   *
   * @param motorPercent  Speed in range [-1,1]
   */
  public static void setSpeed(double motorPercent) {
    if (counter++ % 2 == 0) { System.out.println("**climber power: "+String.format("%.3f  ", motorPercent));};

    //motorPercent = motorPercent * 0.75; // if need to limit speed
    climber.set(motorPercent);
  }

  public static Command stop() {
    System.out.println("**in climber stop");
    setSpeed(0.0);
    return null;
  }
}
