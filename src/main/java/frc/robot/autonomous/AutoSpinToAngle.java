// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoSpinToAngle extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double targetAngle;
  private double turnPower;

  private double currentHeading;
  private double currentDiff;
  private double currentDiffAbs;
  private double turnRate = 0.0;          // adjusted turn rate based on closeness to target 
  private double slowDownCutoff = 10.0;   // degrees near target to slow down
  private double slowDownReducer = 0.85;  // amount to reduce power when near target
  private boolean inRange = false;
  private int counter1 = 2;
  private int counter2 = 2;
  private int counter3 = 2;

  /**
   * Creates a new AutoSpinToAngle.
   */
  public AutoSpinToAngle(double turnPowerIn, double targetAngleIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.targetAngle = targetAngleIn;   // in degrees from current heading
    this.turnPower = turnPowerIn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    DriveTrain.stop();        // make sure robot is stopped
    DriveTrain.resetGyro();   // reset gyro so 0 is current heading
    inRange = false;
    System.out.println("**starting AutoSpinToAngle target angle: " +targetAngle +" heading: "+String.format("%.3f", m_driveTrain.getHeadingAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentHeading = m_driveTrain.getHeadingAngle();    // get current heading
    currentDiff = targetAngle - currentHeading;         // get how far off we are
    currentDiffAbs = Math.abs(currentDiff);
    inRange = (currentDiffAbs  <= DriveConstants.kToleranceDegrees);
    if (counter1++ % 3 == 0) { System.out.println("**diff: "+String.format("%.3f", currentDiff)+" heading: "+String.format("%.3f", currentHeading)+" target: "+targetAngle+" inRange: "+inRange); }
    if (inRange) {
      DriveTrain.stop();  // in range so stop 
      System.out.println("**in range stop turn - heading: "+String.format("%.3f", currentHeading));
    } else {
      // slow rate of turn if close to target angle, cutoff angle is guesstimate
      if (currentDiffAbs > slowDownCutoff) {
        turnRate = turnPower;                     // far away from target so full power
      } else {
        turnRate = turnPower * slowDownReducer;   // close to target so reduce power 
      }

      if (currentDiff < 0) {
        DriveTrain.doTankDrive(-turnRate, turnRate); // turn to left ( reverse left side)
        if (counter2++ % 3 == 0) { System.out.println("**turn Left Correction - heading: "+String.format("%.3f", currentHeading)); }

      } else {
        DriveTrain.doTankDrive(turnRate, -turnRate); // turn to right ( reverse right side)
        if (counter3++ % 3 == 0) { System.out.println("**turn Right Correction - heading: "+String.format("%.3f", currentHeading)); }
      }
    }
  }

  //public static boolean between(double i, double minValueInclusive, double maxValueInclusive) {
  //  return (i >= minValueInclusive && i <= maxValueInclusive);
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.stop();  // stop driveTrain on exit
    System.out.println("**ending AutoSpinToAngle command  current heading: " + String.format("%.3f", currentHeading));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inRange;
  }
}
