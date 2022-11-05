// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A complex autonomous command that drives forward and turns toward goal in FRC competition
 */
public class AutoStartPos4 extends SequentialCommandGroup {

  /** 
   * Creates a new SequentialCommandGroup AutoStartPos4
   * positive angles turn right, negative angles turn left
   *
   * @param driveTrain shooter intake delivery subsystems this command will run on
   */
  public AutoStartPos4(DriveTrain driveTrain, Shooter shooter, Intake intake, Delivery delivery) {

    addCommands(
      new InstantCommand(() -> DriveTrain.stop(), driveTrain),    // make sure stopped
      new AutoDriveStraightTime(-0.6, 1),  // go straight off starting line
      new AutoSpinToAngle(0.6, -3),
      new WaitCommand(0.5),
      new InstantCommand(() -> Shooter.setSpeed(-0.75), shooter), //Turn shooter on
      new WaitCommand(1),
      new InstantCommand(() -> Delivery.setSpeed(-0.25), delivery), //Turn delivery on
      new WaitCommand(1),
      new InstantCommand(() -> Shooter.stop(), shooter),       //Turn shooter off
      new AutoSpinToAngle(0.6, 3),
      new WaitCommand(2),
      new InstantCommand(() -> Intake.setSpeed(0.5), intake),   //Turn intake on
      new AutoDriveStraightTime(-0.6, 2.5),
      //new WaitCommand(1), //Pick up ball
      new InstantCommand(() -> Intake.stop(), intake),   //Turn intake off
      new AutoDriveStraightTime(0.6, 2.5),
      new AutoSpinToAngle(0.6, -3),
      //new WaitCommand(0.5),
      new PrintCommand("*** Just before shooter ***"),
      new InstantCommand(() -> Shooter.setSpeed(-0.75), shooter),   //Turn shooter on
      new WaitCommand(0.5),
      new InstantCommand(() -> Delivery.setSpeed(-0.25), delivery), //Turn delivery on
      new WaitCommand(1),
      new InstantCommand(() -> Intake.stop(), intake),         //Turn intake off
      new InstantCommand(() -> Delivery.stop(), delivery),     //Turn delivery off
      new InstantCommand(() -> Shooter.stop(), shooter),       //Turn shooter off
      new AutoSpinToAngle(0.6, 3),
      //new WaitCommand(1),
      new AutoDriveStraightTime(-0.6, 4), //Get out of the way
      new InstantCommand(() -> DriveTrain.stop(), driveTrain)  // make sure stopped
    );
  }
}
