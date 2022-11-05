// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraightTime extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  private double duration;
  
  private double endTime = 0.0;

  /**
   * Creates a new AutoDriveStraightTime.
   */
  public AutoDriveStraightTime(double speedIn, double timeIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;
    this.duration = timeIn;       // in seconds

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.stop();        // make sure robot is stopped
    endTime = Timer.getFPGATimestamp() + duration;   // get end time
    System.out.println(MessageFormat.format("**Started {0} for {1} secs", this.getName(), duration));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //1 second = 1.5 ft for talon srx
    DriveTrain.doTankDrive(speed, speed);
  }

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