// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterOnCommand extends CommandBase {

  private final Shooter shooter;
  private final double speed;  // value for speed

  /** Creates a new ShooterOnCommand . */
  public ShooterOnCommand(double speedIn) {

    this.shooter = RobotContainer.m_shooter;    // get shooter object from RobotContainer
    this.speed = speedIn;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("**Started {0} with {1} power ", this.getName(), speed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.setSpeed( speed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("**in ShooterOnCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;   // this command just turns on shooter
  }
}
