// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.commands.DriveCommand;
import frc.robot.commands.GoTele;
import frc.robot.autonomous.AutoDriveStraightTime;
import frc.robot.autonomous.AutoDriveStraightTimeGyro;
import frc.robot.autonomous.AutoDriveStraightUnits;
import frc.robot.autonomous.AutoSpinToAngle;
import frc.robot.autonomous.AutoStartPos1;
import frc.robot.autonomous.AutoStartPos2;
import frc.robot.autonomous.AutoStartPos3;
import frc.robot.autonomous.AutoStartPos4;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShooterOffCommand;
import frc.robot.commands.ShooterOnCommand;
import frc.robot.commands.ProximitysenseCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Proximitysense;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public static final DriveTrain m_driveTrain = new DriveTrain();
    public static final Shooter m_shooter = new Shooter();
    public static final Intake m_intake = new Intake();
    public static final Delivery m_delivery = new Delivery();
    public static final Climber m_climber = new Climber();
    public static final Proximitysense m_colorSensor = new Proximitysense();

    //Controller #0 | Main driver
    public static final XboxController controller0 = new XboxController(0);

    public static final JoystickButton con0ButtonA = new JoystickButton(controller0, OIConstants.kXboxButtonA);
    public static final JoystickButton con0ButtonB = new JoystickButton(controller0, OIConstants.kXboxButtonB);
    public static final JoystickButton con0ButtonX = new JoystickButton(controller0, OIConstants.kXboxButtonX);
    public static final JoystickButton con0ButtonY = new JoystickButton(controller0, OIConstants.kXboxButtonY);
    public static final JoystickButton con0ButtonBack = new JoystickButton(controller0, OIConstants.kXboxButtonBack);
    public static final JoystickButton con0ButtonStart = new JoystickButton(controller0, OIConstants.kXboxButtonStart);
    public static final JoystickButton con0BumperLeft =  new JoystickButton(controller0, OIConstants.kXboxBumperLeft);
    public static final JoystickButton con0BumperRight = new JoystickButton(controller0, OIConstants.kXboxBumperRight);
    public static final JoystickButton con0StickPressLeft = new JoystickButton(controller0, OIConstants.kXboxStickPressLeft);
    public static final JoystickButton con0StickPressRight = new JoystickButton(controller0, OIConstants.kXboxStickPressRight);
    public POVButton con0PovUp = new POVButton(controller0, 0);
    public POVButton con0PovRight = new POVButton(controller0, 90);
    public POVButton con0PovDown = new POVButton(controller0, 180);
    public POVButton con0PovLeft = new POVButton(controller0, 270);


    //Controller #1 | Secondary driver
    public static final Joystick controller1 = new Joystick(1);

    public static final JoystickButton con1ButtonX = new JoystickButton(controller1, OIConstants.kLogiButtonX);
    public static final JoystickButton con1ButtonY = new JoystickButton(controller1, OIConstants.kLogiButtonY);
    public static final JoystickButton con1ButtonA = new JoystickButton(controller1, OIConstants.kLogiButtonA);
    public static final JoystickButton con1ButtonB = new JoystickButton(controller1, OIConstants.kLogiButtonB);
    public static final JoystickButton con1ButtonBack = new JoystickButton(controller1, OIConstants.kLogiButtonBack);
    public static final JoystickButton con1ButtonStart = new JoystickButton(controller1, OIConstants.kLogiButtonStart);
    public static final JoystickButton con1BumperLeft =  new JoystickButton(controller1, OIConstants.kLogiBumperLeft);
    public static final JoystickButton con1BumperRight = new JoystickButton(controller1, OIConstants.kLogiBumperRight);
    public static final JoystickButton con1StickPressLeft = new JoystickButton(controller1, OIConstants.kLogiStickPressLeft);
    public static final JoystickButton con1StickPressRight = new JoystickButton(controller1, OIConstants.kLogiStickPressRight);
    public POVButton con1PovUp = new POVButton(controller1, 0);
    public POVButton con1PovRight = new POVButton(controller1, 90);
    public POVButton con1PovDown = new POVButton(controller1, 180);
    public POVButton con1PovLeft = new POVButton(controller1, 270);


    //Controller #2 | Full driver
    public static final XboxController controller2 = new XboxController(2);

    public static final JoystickButton con2ButtonA = new JoystickButton(controller2, OIConstants.kXboxButtonA);
    public static final JoystickButton con2ButtonB = new JoystickButton(controller2, OIConstants.kXboxButtonB);
    public static final JoystickButton con2ButtonX = new JoystickButton(controller2, OIConstants.kXboxButtonX);
    public static final JoystickButton con2ButtonY = new JoystickButton(controller2, OIConstants.kXboxButtonY);
    public static final JoystickButton con2ButtonBack = new JoystickButton(controller2, OIConstants.kXboxButtonBack);
    public static final JoystickButton con2ButtonStart = new JoystickButton(controller2, OIConstants.kXboxButtonStart);
    public static final JoystickButton con2BumperLeft =  new JoystickButton(controller2, OIConstants.kXboxBumperLeft);
    public static final JoystickButton con2BumperRight = new JoystickButton(controller2, OIConstants.kXboxBumperRight);
    public static final JoystickButton con2StickPressLeft = new JoystickButton(controller2, OIConstants.kXboxStickPressLeft);
    public static final JoystickButton con2StickPressRight = new JoystickButton(controller2, OIConstants.kXboxStickPressRight);
    public POVButton con2PovUp = new POVButton(controller2, 0);
    public POVButton con2PovRight = new POVButton(controller2, 90);
    public POVButton con2PovDown= new POVButton(controller2, 180);
    public POVButton con2PovLeft = new POVButton(controller2, 270);


    //Controller #5 | Guest driver
    public static final XboxController controller5 = new XboxController(5);

    public static final JoystickButton con5ButtonA = new JoystickButton(controller5, OIConstants.kXboxButtonA);
    public static final JoystickButton con5ButtonB = new JoystickButton(controller5, OIConstants.kXboxButtonB);
    public static final JoystickButton con5ButtonX = new JoystickButton(controller5, OIConstants.kXboxButtonX);
    public static final JoystickButton con5ButtonY = new JoystickButton(controller5, OIConstants.kXboxButtonY);
    public static final JoystickButton con5ButtonBack = new JoystickButton(controller5, OIConstants.kXboxButtonBack);
    public static final JoystickButton con5ButtonStart = new JoystickButton(controller5, OIConstants.kXboxButtonStart);
    public static final JoystickButton con5BumperLeft =  new JoystickButton(controller5, OIConstants.kXboxBumperLeft);
    public static final JoystickButton con5BumperRight = new JoystickButton(controller5, OIConstants.kXboxBumperRight);
    public static final JoystickButton con5StickPressLeft = new JoystickButton(controller5, OIConstants.kXboxStickPressLeft);
    public static final JoystickButton con5StickPressRight = new JoystickButton(controller5, OIConstants.kXboxStickPressRight);
    public POVButton con5PovUp = new POVButton(controller5, 0);
    public POVButton con5PovRight = new POVButton(controller5, 90);
    public POVButton con5PovDown = new POVButton(controller5, 180);
    public POVButton con5PovLeft = new POVButton(controller5, 270);


    private static Command autoDriveStraightGyroCommand;
    private static Command autoDriveStraightCommand;
    private static Command autoDriveUnitsCommand;
    private static Command autoDriveSpinCommand;
    private static Command autoDriveTurnCommand;
    private static Command autoStartPos1Command;
    private static Command autoStartPos2Command;
    private static Command autoStartPos3Command;
    private static Command autoStartPos4Command;
  
    // ** set up autonomous chooser
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static boolean inCompetition = false;
    public static String allianceColor;
    public static int startPosition;


  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new GoTele());
    //m_driveTrain.setDefaultCommand(new DriveCommand(() -> -controller0.getLeftY(), () -> -controller0.getRightY()));
    
    m_colorSensor.setDefaultCommand(new ProximitysenseCommand());
    
    buildAutonomousCommands();    // go create autonomous commands
    configureButtonBindings();

    //autoChooser.setDefaultOption("Auto Drive Straight", autoDriveStraightCommand );
    //autoChooser.addOption("Auto Drive Units", autoDriveUnitsCommand );
    //autoChooser.addOption("Auto Drive Gyro", autoDriveStraightGyroCommand );
    //autoChooser.addOption("Auto Spin", autoDriveSpinCommand );
    //autoChooser.addOption("Auto Turn", autoDriveTurnCommand );
    autoChooser.setDefaultOption("Auto Start Postion 1", autoStartPos1Command );
    autoChooser.addOption("Auto Start Postion 2", autoStartPos2Command );
    autoChooser.addOption("Auto Start Postion 3", autoStartPos3Command );
    autoChooser.addOption("Auto Start Postion 4", autoStartPos4Command );
    SmartDashboard.putData("Auto Choices", autoChooser);

    if (DriverStation.getAlliance() == Alliance.Blue) {
        allianceColor = "blue";
    } else {
        allianceColor = "red";
    }
    startPosition = DriverStation.getLocation();
    if (DriverStation.isFMSAttached()) {
        inCompetition = true;
    } else {
        inCompetition = false;
    }
    System.out.println("start Positon: " + startPosition + " alliance: " + allianceColor + " in Competition: " + inCompetition);
  }


  private void configureButtonBindings() {
    //Main controllers
    con0StickPressLeft.whenPressed(() -> DriveTrain.doLowGear(false));
    con0StickPressRight.whenPressed(() -> DriveTrain.doLowGear(true));

    con0BumperRight.whenPressed(() -> Climber.setSpeed(-0.85));
    con0BumperRight.whenReleased(() -> Climber.stop());
    con0BumperLeft.whenPressed(() -> Climber.setSpeed(0.85));
    con0BumperLeft.whenReleased(() -> Climber.stop());

    con0ButtonA.whileHeld(new ShooterOnCommand(0.9));
    con0ButtonA.whenReleased(new ShooterOffCommand());

    con1ButtonB.toggleWhenPressed(new StartEndCommand(() -> Shooter.setSpeed(-0.65), () -> Shooter.stop())); //95% works
    con1ButtonX.toggleWhenPressed(new StartEndCommand(() -> Intake.setSpeed(0.35), () -> Intake.stop())); //50% for upper limit
    con1ButtonY.toggleWhenPressed(new StartEndCommand(() -> Delivery.setSpeed(-0.25), () -> Delivery.stop())); //50% for upper limit
          
    con1BumperLeft.whileHeld(() -> Delivery.setSpeed(-0.25));
    con1BumperLeft.whileHeld(() -> Intake.setSpeed(0.35));
    con1BumperLeft.whileHeld(() -> Shooter.setSpeed(-0.15));
    con1BumperLeft.whenReleased(() -> Delivery.stop());
    con1BumperLeft.whenReleased(() -> Intake.stop());
    con1BumperLeft.whenReleased(() -> Shooter.stop());


    //Single controller
    con2StickPressLeft.whenPressed(() -> DriveTrain.doLowGear(false));
    con2StickPressRight.whenPressed(() -> DriveTrain.doLowGear(true));

    con2PovUp.whenPressed(() -> Climber.setSpeed(-0.85));
    con2PovUp.whenReleased(() -> Climber.stop());
    con2PovDown.whenPressed(() -> Climber.setSpeed(0.85));
    con2PovDown.whenReleased(() -> Climber.stop());

    con2ButtonA.whileHeld(new ShooterOnCommand(0.9));
    con2ButtonA.whenReleased(new ShooterOffCommand());
    con2ButtonB.toggleWhenPressed(new StartEndCommand(() -> Shooter.setSpeed(-0.65), () -> Shooter.stop())); //95% works
    con2ButtonX.toggleWhenPressed(new StartEndCommand(() -> Intake.setSpeed(0.35), () -> Intake.stop())); //50% for upper limit
    con2ButtonY.toggleWhenPressed(new StartEndCommand(() -> Delivery.setSpeed(-0.25), () -> Delivery.stop())); //50% for upper limit 

    con2BumperLeft.whileHeld(() -> Delivery.setSpeed(-0.25));
    con2BumperLeft.whileHeld(() -> Intake.setSpeed(0.35));
    con2BumperLeft.whileHeld(() -> Shooter.setSpeed(-0.15));
    con2BumperLeft.whenReleased(() -> Delivery.stop());
    con2BumperLeft.whenReleased(() -> Intake.stop());
    con2BumperLeft.whenReleased(() -> Shooter.stop());
  

    //Guest controller 
    con5ButtonB.toggleWhenPressed(new StartEndCommand(() -> Shooter.setSpeed(-0.25), () -> Shooter.stop()));
    con5ButtonX.toggleWhenPressed(new StartEndCommand(() -> Intake.setSpeed(0.25), () -> Intake.stop()));
    con5ButtonY.toggleWhenPressed(new StartEndCommand(() -> Delivery.setSpeed(-0.15), () -> Delivery.stop()));
  }

  private void buildAutonomousCommands() {
    autoDriveStraightCommand = new AutoDriveStraightTime(0.6, 4.0);
    autoDriveStraightGyroCommand = new AutoDriveStraightTimeGyro(0.6, 5.0);
    autoDriveUnitsCommand = new AutoDriveStraightUnits(0.6, 120.0).withTimeout(10);
    autoDriveSpinCommand = new AutoSpinToAngle(0.6, 45.0);
    autoDriveTurnCommand = new AutoSpinToAngle(0.5, 45.0);
    autoStartPos1Command = new AutoStartPos1(m_driveTrain, m_shooter, m_intake, m_delivery);
    autoStartPos2Command = new AutoStartPos2(m_driveTrain, m_shooter, m_intake, m_delivery);
    autoStartPos3Command = new AutoStartPos3(m_driveTrain, m_shooter, m_intake, m_delivery);
    autoStartPos4Command = new AutoStartPos4(m_driveTrain, m_shooter, m_intake, m_delivery);
  }

  public Command getAutonomousCommand() {
    System.out.println("***getting Autonomous command");
    Command autoSelected = autoChooser.getSelected();
    return autoSelected;
    //return autoDriveStraightCommand;
    //return autoDriveStraightGyroCommand;
    //return autoDriveUnitsCommand;
    //return autoDriveSpinCommand;
    //return autoDriveTurnCommand;
    //return autoShooterOnCommand;
    //return autoShooterOffCommand;
  }
}