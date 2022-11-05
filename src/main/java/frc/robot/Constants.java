// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveConstants {
      //// Drive motor controller IDs
      public static final int leftDrive1Id = 4;
      public static final int leftDrive2Id = 5;
      public static final int leftDrive3Id = 6;
      public static final int rightDrive1Id = 1;
      public static final int rightDrive2Id = 2;
      public static final int rightDrive3Id = 3;

      public static final int deliveryId = 7;
      public static final int shooterId = 8;
      public static final int intakeId = 9;
      public static final int climberId = 10;
      public static final int kAmpsMax = 30;
      public static final double kbatteryLowVoltage = 11.5;

      public static final int LEFT_ENCODER_A = 2;
      public static final int LEFT_ENCODER_B = 4;
      public static final int RIGHT_ENCODER_A = 1;
      public static final int RIGHT_ENCODER_B = 3;

      // circumference = (2 * PI * radius) or (PI * diameter)
      public static final double kWheelDiameterInches = 6.0; // in inches
      public static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);
      public static final double kWheelTrackWidthInches = 22.75; // distance from wheels on 1 side to other side
      public static final double kEncoderTicksPerRevolution = -10.664;  // Hall-Sensor Encoder resolution //was 42
      public static final double kDriveMotorGearRatio = 1.0;  // gear ratio of drive motor // 10.71
      public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kDriveMotorGearRatio));
      public static final double kEncoderVelocityConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(60*kDriveMotorGearRatio));
     
      //public static final double kInchesPerPulse = (Math.PI * kWheelDiameterInches) / (kEncoderTicksPerRevolution * kDriveMotorGearRatio); //formula for in/pulse
      public static final double kInchesPerPulse = 1.5;
      public static final double kInchesPerDegree = (Math.PI * kWheelDiameterInches) /  360.0; // circum / 360

      public static final double kTurnP = 1.0;
      public static final double kTurnI = 0.0;
      public static final double kTurnD = 0.0;
  
      public static final double kToleranceDegrees = 3.0;  //indicates how close to "on target" acceptable
      public static final double kTurnToleranceDeg = 3.0;
      public static final double kTurnRateToleranceDegPerS = 10.0; // degrees per second
      //public static final double kMaxTurnRateDegPerS = 100;
      //public static final double kMaxTurnAccelerationDegPerSSquared = 300;
      //public static final boolean kGyroReversed = false;
    }

    public static final class OIConstants {
        // Xbox controller button mappings
        public static final int kXboxButtonA = 1;
        public static final int kXboxButtonB = 2;
        public static final int kXboxButtonX = 3;
        public static final int kXboxButtonY = 4;
        public static final int kXboxBumperLeft = 5;
        public static final int kXboxBumperRight = 6;
        public static final int kXboxButtonBack = 7;
        public static final int kXboxButtonStart = 8;
        public static final int kXboxStickPressLeft = 9;
        public static final int kXboxStickPressRight = 10;

        public static final int kXboxAxisLeftStickX = 0;  // for .getRawAxis()
        public static final int kXboxAxisLeftStickY = 1;
        public static final int kXboxAxisLeftTrigger = 2;
        public static final int kXboxAxisRightTrigger = 3;
        public static final int kXboxAxisRightStickX = 4;
        public static final int kXboxAxisRightStickY = 5;
 
        // LogiTech controller axis mappings
        public static final int kLogiAxisLeftStickX = 1;
        public static final int kLogiAxisLeftStickY = 2;
        public static final int kLogiAxisTriggers = 3; // left trigger only=-1.0, right only=1.0, both/none=0.0
        public static final int kLogiAxisRightStickX = 4;
        public static final int kLogiAxisRightStickY = 5;
        public static final int kLogiAxisDpad = 6;

        public static final int kLogiButtonA = 1;
        public static final int kLogiButtonB = 2;
        public static final int kLogiButtonX = 3;
        public static final int kLogiButtonY = 4;
        public static final int kLogiBumperLeft = 5;
        public static final int kLogiBumperRight = 6;
        public static final int kLogiButtonBack = 7; 
        public static final int kLogiButtonStart = 8; 
        public static final int kLogiStickPressLeft = 9; 
        public static final int kLogiStickPressRight = 10;
    }
}
