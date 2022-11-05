// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.text.MessageFormat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraightTimeGyro extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  private double duration;

  private double endTime = 0.0;
  private boolean inRange = false;
  //private static final double reduceFactor = 0.992;   // amount to reduce speed on 1 side

  private PIDController pid;
  private static final double kP = 0.03;    // P term constant (Proportional)
  private static final double kI = 0.00;    // I term constant (Integral)
  private static final double kD = 0.00;    // D term constant (Derivative)
  //private static final double kF = 0.0001;  // F term constant (Feedforward)

  private static final double kToleranceFromStraight = 1.0;  // degrees acceptable from 0 straight ahead 

  private static int counter1 = 3;
  private static int counter2 = 3;
  private static int counter3 = 3;
  
  /**
   * Creates a new AutoDriveStraightTimeGyro.
   */
  public AutoDriveStraightTimeGyro(double speedIn, double timeIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;
    this.duration = timeIn;       // in seconds

    this.pid = new PIDController(kP, kI, kD);    // create PID object

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    DriveTrain.stop();        // make sure robot is stopped
    DriveTrain.resetGyro();  // reset gyro so 0 is current heading
    pid.reset();
    pid.setSetpoint(0.0);
    pid.setTolerance(DriveConstants.kToleranceDegrees);    // set tolerance around setpoint, targetAngle in this case
    pid.enableContinuousInput(-180.0, 180.0); // Enable continuous input in range from -180 to 180

    endTime = Timer.getFPGATimestamp() + duration;   // get end time
    System.out.println("**starting "+this.getName()+" duration: " +duration +" heading: "+String.format("%.3f", m_driveTrain.getHeadingAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentHeading = m_driveTrain.getHeadingAngle();
    //inRange = (Math.abs(currentHeading)  <= DriveConstants.kToleranceDegrees);  // 0 is straight ahead
    inRange = (Math.abs(currentHeading)  <= kToleranceFromStraight);  // 0 is straight ahead
    
    double leftSpeed = speed;   // can be changed below based on gyro readings
    double rightSpeed = speed;
    if (inRange) {
      // if heading is in range no change (assumes 0 heading for straight)
      if (counter1++ % 2 == 0) { System.out.println("**straight no correction: L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }
    } else {
      double pidOutput = MathUtil.clamp(pid.calculate(currentHeading, 0), -0.05, 0.05);  // check if veered off 0 heading (straight ahead)
      
      if (pidOutput <= 0) {
        leftSpeed = leftSpeed  * (1 - pidOutput);   // turn to left ( slow left side)
        if (counter2++ % 3 == 0) { System.out.println("**veer Left Correction pidOut: "+String.format("%.3f  ", pidOutput)+" heading: "+String.format("%.3f", currentHeading)); }
        
      } else {
        rightSpeed = rightSpeed  * (1 - pidOutput); // turn to right ( slow right side) 
       if (counter3++ % 3 == 0) { System.out.println("**veer Right Correction pidOut: "+String.format("%.3f  ", pidOutput)+" heading: "+String.format("%.3f", currentHeading)); }
      }
    }

    DriveTrain.doTankDrive(leftSpeed, rightSpeed);
  
    /***********************
    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
    double test1 = pid.calculate( 7, 0 );
    System.out.println("**test1 pidOut for heading 7, targetAngle: 0 >  " + String.format("%.4f", test1));
    double test2 = pid.calculate( 6, 0 );
    System.out.println("**test2 pidOut for heading 6, targetAngle: 0 >  " + String.format("%.4f", test2));
    double test3 = pid.calculate( 5, 0 );
    System.out.println("**test3 pidOut for heading 5, targetAngle: 0 >  " + String.format("%.4f", test3));
    double test4 = pid.calculate( 4, 0 );
    System.out.println("**test4 pidOut for heading 4, targetAngle: 0 >  " + String.format("%.4f", test4));
    double test5 = pid.calculate( 3, 0 );
    System.out.println("**test5 pidOut for heading 3, targetAngle: 0 >  " + String.format("%.4f", test5));
    double test6 = pid.calculate( 2, 0 );
    System.out.println("**test6 pidOut for heading 2, targetAngle: 0 >  " + String.format("%.4f", test6));
    double test7 = pid.calculate( 1, 0 );
    System.out.println("**test7 pidOut for heading 1, targetAngle: 0 >  " + String.format("%.4f", test7));
    double test8 = pid.calculate( 0, 0 );
    System.out.println("**test8 pidOut for heading 0, targetAngle: 0 >  " + String.format("%.4f", test8));

    double test11 = pid.calculate( -1, 0 );
    System.out.println("**test11 pidOut for heading -1, targetAngle: 0 >  " + String.format("%.4f", test11));
    double test12 = pid.calculate( -2, 0 );
    System.out.println("**test12 pidOut for heading -2, targetAngle: 0 >  " + String.format("%.4f", test12));
    double test13 = pid.calculate( -3, 0 );
    System.out.println("**test13 pidOut for heading -3, targetAngle: 0 >  " + String.format("%.4f", test13));
    double test14 = pid.calculate( -4, 0 );
    System.out.println("**test14 pidOut for heading -4, targetAngle: 0 >  " + String.format("%.4f", test14));
    double test15 = pid.calculate( -5, 0 );
    System.out.println("**test15 pidOut for heading -5, targetAngle: 0 >  " + String.format("%.4f", test15));
    double test16 = pid.calculate( -6, 0 );
    System.out.println("**test16 pidOut for heading -6, targetAngle: 0 >  " + String.format("%.4f", test16));
    double test17 = pid.calculate( -7, 0 );
    System.out.println("**test17 pidOut for heading -7, targetAngle: 0 >  " + String.format("%.4f", test17));

    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
    **/
  }

  //public static double clampValue(double value, double min, double max) {
  //   return Math.max(min, Math.min(value, max));		// make sure within range
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.stop();  // stop driveTrain on exit
	  System.out.println(MessageFormat.format("**Ended {0}  at {1} secs", this.getName(), duration));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
	return Timer.getFPGATimestamp() >= endTime;   // check if time to end
  }
}
