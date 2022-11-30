package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DriveCommand;

public class GoTele extends CommandBase {
    @Override
    public void initialize() {
      System.out.println(MessageFormat.format("**Started {0} ", this.getName()));
    }
    double teleLeft = 0;
    double teleRight = 0;
    private final DriveTrain drivetrain;

    public GoTele () {
        this.drivetrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
      
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain); 
      }

    @Override
    public void execute() {
      DriveCommand dcObj = new DriveCommand(() -> teleLeft, () -> teleRight);
      double deadzone = 0.1;
      boolean usingCon0 = Math.abs(RobotContainer.controller0.getLeftY()) < deadzone || Math.abs(RobotContainer.controller0.getRightY()) < deadzone;
      boolean usingCon2 = Math.abs(RobotContainer.controller2.getLeftY()) < deadzone || Math.abs(RobotContainer.controller2.getRightY()) < deadzone;
      boolean usingCon5 = Math.abs(RobotContainer.controller5.getLeftY()) < deadzone || Math.abs(RobotContainer.controller5.getRightY()) < deadzone;
      double teleLeft = 0;
      double teleRight = 0;
      System.out.print(usingCon0);
      //System.out.println(RobotContainer.controller0.getLeftY());
      if (RobotContainer.controller0.isConnected() && usingCon0) {
        //Using two controllers
        teleLeft = RobotContainer.controller0.getLeftY();
        teleRight = RobotContainer.controller0.getRightY();
      }
      else {
        if (RobotContainer.controller2.isConnected() && usingCon2) {
          //Using one controller
          teleLeft = RobotContainer.controller2.getLeftY();
          teleRight = RobotContainer.controller2.getRightY();
        }
        else {
          if (RobotContainer.controller5.isConnected() && usingCon5) {
            //Using the guest controller
            teleLeft = RobotContainer.controller5.getLeftY();
            teleLeft = teleLeft * 0.5;
            teleRight = RobotContainer.controller5.getRightY();
            teleRight = teleRight * 0.5;
          }
          else {
            teleLeft = 0;
            teleRight = 0;
          }
        }
      }
      teleLeft = teleLeft * -1;
      teleRight = teleRight * -1;
      dcObj.execute();
      SmartDashboard.putNumber("Left Drive Speed", teleLeft);
      SmartDashboard.putNumber("Right Drive Speed", teleRight);
    }
    @Override
    public boolean isFinished() {
      return false;   // this command just turns off shooter
    }
}
