// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.DriveCommand;
//import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DriveTrain extends SubsystemBase {

  // For Neo motors - 6 separate motor controllers with 1 pwm channel per controller
  private final static CANSparkMax motorDriveLeft1 = new CANSparkMax(DriveConstants.leftDrive1Id, MotorType.kBrushless);
  private final static CANSparkMax motorDriveLeft2 = new CANSparkMax(DriveConstants.leftDrive2Id, MotorType.kBrushless);
  private final static CANSparkMax motorDriveLeft3 = new CANSparkMax(DriveConstants.leftDrive3Id, MotorType.kBrushless);
  private final static CANSparkMax motorDriveRight1 = new CANSparkMax(DriveConstants.rightDrive1Id, MotorType.kBrushless);
  private final static CANSparkMax motorDriveRight2 = new CANSparkMax(DriveConstants.rightDrive2Id, MotorType.kBrushless);
  private final static CANSparkMax motorDriveRight3 = new CANSparkMax(DriveConstants.rightDrive3Id, MotorType.kBrushless);

  // define Speed Controller Groups and Differential Drive for use in drive train
  private final static MotorControllerGroup driveGroupLeft = new MotorControllerGroup(motorDriveLeft1, motorDriveLeft2, motorDriveLeft3);
  private final static MotorControllerGroup driveGroupRight = new MotorControllerGroup(motorDriveRight1, motorDriveRight2, motorDriveRight3);
  private final static DifferentialDrive differentialDrive = new DifferentialDrive(driveGroupLeft, driveGroupRight);
 
  // define encoders for tracking distance
  private static RelativeEncoder leftDriveEncoder; //For Spark Max
  private static RelativeEncoder rightDriveEncoder;

  private static double leftEncoderZeroValue = 0;
  private static double rightEncoderZeroValue = 0;

  // define limit switches
  //private SparkMaxLimitSwitch m_forwardLimit;
  //private SparkMaxLimitSwitch m_reverseLimit;

  // navX Gyro on RoboRio
  private static AHRS Gyro;

  private static DoubleSolenoid solenoidGearShift;
  Compressor compressor;

  private static final boolean kSquareInputs = true;
  private static final boolean kSkipGyro = false;
  private static final boolean kSkipEncoder = false;   // true for Spark Max, false for Talon SRX
  private static int counter1 = 5; // for limiting display
  private static int counter2 = 5;
  private static int counter3 = 5;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    motorDriveLeft1.restoreFactoryDefaults();   // For Spark Max Neo motors
    motorDriveLeft2.restoreFactoryDefaults();
    motorDriveLeft3.restoreFactoryDefaults();
    motorDriveRight1.restoreFactoryDefaults();
    motorDriveRight2.restoreFactoryDefaults();
    motorDriveRight3.restoreFactoryDefaults();
 
     motorDriveLeft1.setInverted(true);
     motorDriveLeft2.setInverted(true);
     motorDriveLeft3.setInverted(true);
     motorDriveRight1.setInverted(false);
     motorDriveRight2.setInverted(false);
     motorDriveRight3.setInverted(false);
 
     motorDriveLeft1.setIdleMode(IdleMode.kCoast); // For Spark Max
     motorDriveLeft2.setIdleMode(IdleMode.kCoast);
     motorDriveLeft3.setIdleMode(IdleMode.kCoast);
     motorDriveRight1.setIdleMode(IdleMode.kCoast);
     motorDriveRight2.setIdleMode(IdleMode.kCoast);
     motorDriveRight3.setIdleMode(IdleMode.kCoast);
 
     motorDriveLeft1.setSmartCurrentLimit(DriveConstants.kAmpsMax); // set current limit for Spark Max
     motorDriveLeft2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
     motorDriveLeft3.setSmartCurrentLimit(DriveConstants.kAmpsMax);
     motorDriveRight1.setSmartCurrentLimit(DriveConstants.kAmpsMax);
     motorDriveRight2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
     motorDriveRight3.setSmartCurrentLimit(DriveConstants.kAmpsMax);

     motorDriveLeft1.setOpenLoopRampRate(0.25); // time for Spark Max to get to full speed
     motorDriveLeft2.setOpenLoopRampRate(0.25);
     motorDriveLeft3.setOpenLoopRampRate(0.25);
     motorDriveRight1.setOpenLoopRampRate(0.25);
     motorDriveRight2.setOpenLoopRampRate(0.25);
     motorDriveRight3.setOpenLoopRampRate(0.25);
 
     differentialDrive.setDeadband(0.03);
     differentialDrive.setSafetyEnabled(false); // ***to avoid error 'differentialDrive not fed often enough

    if (kSkipEncoder) {
      leftDriveEncoder = null;
      rightDriveEncoder = null;
    } else {
      //https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
      leftDriveEncoder = motorDriveLeft1.getEncoder();    // Declare the left encoder
      rightDriveEncoder = motorDriveRight1.getEncoder();  // Declare the right encoder

      leftDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor); // set encoder constants
      leftDriveEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor); // can be set via REV Hardware client
      rightDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
      rightDriveEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

      // set limit switches - can be 1 of 2 polarities: normally open or normally closed depending on direction to limit
      //m_forwardLimit = motorDriveLeft1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
      //m_reverseLimit = motorDriveRight1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      //m_forwardLimit.enableLimitSwitch(false);
      //m_reverseLimit.enableLimitSwitch(false);
      //boolean limitEnabled = m_forwardLimit.isLimitSwitchEnabled();   // check if limitSwitch Enabled
      //boolean limitPressed = m_forwardLimit.isPressed();  // returns true if switch is pressed (or not connected), false when released
    }

    if (kSkipGyro) {
      Gyro = null;
    } else {
      // navX-MXP Gyro instantiation
      try {
        // Instantiate Gyro - communicate w/navX-MXP via the MXP SPI Bus
        // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
        // See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        Gyro = new AHRS(SPI.Port.kMXP);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (Gyro.isCalibrating()) {
        try {
          Thread.sleep(500);     //sleep in milliseconds
        } catch (Exception e) {
          System.out.println(e);
        }
        System.out.println("**gyro isCalibrating . . .");
      }
      SmartDashboard.putBoolean("gyro connected", Gyro.isConnected());
      //System.out.println("**gyro connected: " + Gyro.isConnected());
    }

    REVLibError burnFlashSuccess;
    burnFlashSuccess = motorDriveLeft1.burnFlash();   // burn current settings into SparkMAX flash
    burnFlashSuccess = motorDriveRight1.burnFlash();
    if (burnFlashSuccess != REVLibError.kOk) {
      System.out.println("** shooter burnFlash() failed **");
    }
    
    // Initialize the solenoids
    solenoidGearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    // compressor instantiation only needed if want to turn off compressor or check pressure
    //compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    //boolean enabled = compressor.enabled();
    //boolean pressureSwitch = compressor.getPressureSwitchValue();
    //double pressureValue = compressor.getPressure();
    //double current = compressor.getCurrent();
    //System.out.println("**Compressor on: " +enabled +" pressure: "+ pressureValue+" pressure switch: " +pressureSwitch+ " current: " + current);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */ 
  public static void doTankDrive(double leftDrivePercent, double rightDrivePercent) {

    //leftDrivePercent = leftDrivePercent * 0.95;		//throttle power on 1 side or other so drives straight
    //rightDrivePercent = rightDrivePercent * 0.95;

    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
	
    if (counter1++ % 40 == 0) {
      //System.out.println("**encoder count "+ String.format("%.3f", getLeftEncoderCount()));
      System.out.println("**driveTrain power L-R: "+ String.format("%.3f", leftDrivePercent)+" ~ " +String.format("%.3f", rightDrivePercent));
   }

    // can use this instead if get differentialDrive not updated often enough
    //motorDriveLeft1.set(leftDrivePercent);
    //motorDriveLeft2.set(leftDrivePercent);
    //motorDriveLeft3.set(leftDrivePercent);
    //motorDriveRight1.set(rightDrivePercent);
    //motorDriveRight2.set(rightDrivePercent);
    //motorDriveRight3.set(rightDrivePercent);
  }

  /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public static void doArcadeDrive(double speed, double rotation) {

    // speed = speed * 0.95;	//if want to throttle power

    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.arcadeDrive(speed, rotation, kSquareInputs);

    if (counter2++ % 5 == 0) {
      System.out.println("**arcade driveTrain speed: " +String.format("%.3f", speed) +"  rotation: " +String.format("%.3f", rotation));
    }
  }

  // http://pdocs.kauailabs.com/navx-mxp/guidance/terminology (for pitch, roll, yaw, IMU terminology)
  public double getHeadingAngle() {
    double currentAngle = Math.IEEEremainder(Gyro.getYaw(), 360.0);
    if (counter3++ % 5 == 0) {
      SmartDashboard.putNumber("gyro", currentAngle);
    }
    return currentAngle;
  }

  public double getYaw() {
    return Gyro.getYaw(); // get rotation around Z axis for current heading
  }

  public static void resetGyro() {
    // "Zero" yaw (whatever direction sensor is pointing now becomes new "Zero" degrees)
    Gyro.zeroYaw(); // yaw is only thing that can be reset, pitch and roll can't (see docs)
  }

  public static void resetEncoders() {
    double ticks = 0.0;
    float tempVal = 0f;
    //if (kSkipEncoder) { return; }   // quit if no encoder

    REVLibError setPosError = leftDriveEncoder.setPosition(tempVal); // this is for Spark Max only
    if (setPosError == REVLibError.kOk) {
      leftEncoderZeroValue = 0.0;
      System.out.println("** reset left encoder successful");
    } else {
      ticks = leftDriveEncoder.getPosition();
      ticks = Math.floor(ticks * 100.0) / 100.0; // round to 2 decimal places
      leftEncoderZeroValue = ticks >= 0 ? ticks : 0; // only allow positive numbers to substracted later
      System.out.println("** ERROR reset left encoder");
    }
    setPosError = rightDriveEncoder.setPosition(tempVal);
    if (setPosError == REVLibError.kOk) {
      rightEncoderZeroValue = 0.0;
      System.out.println("** reset right encoder successful");
    } else {
      ticks = rightDriveEncoder.getPosition();
      ticks = Math.floor(ticks * 100.0) / 100.0; // round to 2 decimal places
      rightEncoderZeroValue = ticks >= 0 ? ticks : 0; // only allow positive numbers to substracted later
      System.out.println("** ERROR reset right encoder");
    }
    //try { Thread.sleep(250); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds while encoders to reset
    System.out.println("** resetting encoders to initial value: L/R "+leftEncoderZeroValue+" ~ " +rightEncoderZeroValue);
  }

  public double getLeftEncoderCount() {
    //if (kSkipEncoder) { return leftEncoderZeroValue; }    // quit if no encoder

    double currentValue = leftDriveEncoder.getPosition() - leftEncoderZeroValue;
    //System.out.println("**left encoder count: " + currentValue);
    return currentValue; // return position of encoder
  }

  public double getRightEncoderCount() {
    //if (kSkipEncoder) { return rightEncoderZeroValue; }    // quit if no encoder

    double currentValue = rightDriveEncoder.getPosition() - rightEncoderZeroValue;
    //System.out.println("**right encoder count: " + currentValue);
    return currentValue; // return position of encoder
  }

  public double getLeftDistanceInch() {
    // = Math.PI * DriveConstants.kWheelDiameter * (getLeftEncoderCount() / DriveConstants.kEncoderTicksPerRevolution);
    return DriveConstants.kInchesPerPulse * getLeftEncoderCount();
  }

  public double getRightDistanceInch() {
    // = Math.PI * DriveConstants.kWheelDiameter * (getRightEncoderCount() / DriveConstants.kEncoderTicksPerRevolution);
    return DriveConstants.kInchesPerPulse * getRightEncoderCount();
  }

  public double getAveDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  // function to set the solenoid to change gears
  public static void doLowGear(final boolean inLowGear) {
    System.out.println("doLowGear: " + inLowGear);

    if (inLowGear) {    // if true then shift to low gear
      solenoidGearShift.set(DoubleSolenoid.Value.kForward); // solenoid controls output that pulls piston in or out
    } else {
      solenoidGearShift.set(DoubleSolenoid.Value.kReverse);
    }
  }


  public void toggleGearshift() {
    solenoidGearShift.toggle();
  }
  
  public static void stop() {
    System.out.println("**in drivetrain stop");
    doTankDrive(0.0, 0.0);
  }
}