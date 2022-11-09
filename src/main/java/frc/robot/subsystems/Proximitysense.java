// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;

public class Proximitysense extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // A Rev Color Sensor V3 object is constructed with an I2C (Inter-Integrated Circuit) port as parameter
  // The device will be automatically initialized with default parameters
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // A Rev Color Match object is used to register and detect known colors
  // This object uses a simple euclidian distance to estimate the closest match
  private final ColorMatch m_colorMatcher = new ColorMatch();

  // create basic colors using RGB values
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  /** Creates a new Proximitysense. */
  public Proximitysense() {

    // add colors to ColorMatcher
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //getMatchedColor();  // get current color - this now handled in ProximitysenseCommand
    getProximity();     // get Proximity info
  }

  public int getProximity() {
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();

    double shooterVelocity = Shooter.getVelocity();
    //System.out.println("velocity: " + shooterVelocity);
    if (proximity > 1000 && Math.abs(shooterVelocity) < 5) {
      Delivery.stop();    // stop delivery if reading > 1800, means ball is under sensor
    }

    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    return proximity;
  }

  public Color getCurrentColor() {
    // GetColor() returns a normalized color value from the sensor. To read the raw color, use GetRawColor().
    // The color sensor works best when within a few inches from an object in well lit conditions
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    return detectedColor;
}

  public String getMatchedColor() {

    // Run the color match algorithm on current color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(getCurrentColor());

    String colorString;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Confidence", match.confidence);

    return colorString;
  }
}
