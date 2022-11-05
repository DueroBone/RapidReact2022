// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterOffCommand extends CommandBase {

  private final Shooter shooter;

  /** Creates a new ShooterOnCommand . */
  public ShooterOffCommand () {
    this.shooter = RobotContainer.m_shooter;    // get driveTrain object from RobotContainer
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("**Started {0} ", this.getName()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;   // this command just turns off shooter
  }
}
