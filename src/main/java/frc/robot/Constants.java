// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

 public final class Constants {

    public static final double RadiansToDegrees = 180.0 / Math.PI;
    public static final double DegreesToRadians = Math.PI / 180.0;
    public static final double Inches_Per_Foot = 12.0;
    public static final double InchesToMeters = 0.0254;

    public static final class DriveConstants {
        public static final int ID_frontLeftMotor = 4;
        public static final int ID_backLeftMotor = 5;
        public static final int ID_frontRightMotor = 1;
        public static final int ID_backRightMotor = 2;

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 22.48 * InchesToMeters;

        // Distance between centers of front and back wheels on robot
        public static final double kWheelBase = 19.81 * InchesToMeters;

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
            );

        public static final double manualVoltageRampingConstant = 0.21;
        public static final double closedVoltageRampingConstant = 0.21;

        public static final int PID_id = 0;
        public static final double PID_Period = 1.0 / 20.0;
        public static final double Drivetrainkf = 0.0485; 
        public static final double DrivetrainkP = 0.001;
//        public static final double DrivetrainkI = 0.0; 
        public static final double DrivetrainkI = 0.0005; 
        public static final double DrivetrainkD = 0.001;

        public static final double Gear_Ratio = 10.71 ;
        public static final double Counts_Per_Wheel_Revolution = 2048.0 * Gear_Ratio;
        public static final double Wheel_Diameter = 6.0 * InchesToMeters;
        public static final double Meters_Per_Revolution = Wheel_Diameter * Math.PI ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Wheel_Revolution;
        public static final double Counts_Per_Meter = 1.0 / Meters_Per_Count ;

        // public static final double WheelDiamater = 6.0 * InchesToMeters; // meters
        // public static final double WheelCircumference = Math.PI * WheelDiamater; // meters
        // public static final double WheelCountsPerMeter = (1.0 / WheelCircumference) * Counts_Per_Wheel_Revolution;
        public static final double MotorVelocityOneMeterPerSecond = Counts_Per_Meter / 10.0;

        public static final double RPMsToUnitsPerHundredMilliseconds = 1.0 / 600.0;
        // public static final double DesiredRPMsForDrive = 560.0;
        // public static final double MaxDriveVelocity = 6000.0;
        // public static final double VelocityInputConversionFactor = DesiredRPMsForDrive * Counts_Per_Revolution * RPMsToUnitsPerHundredMilliseconds;

        public static final int encoderTimeout = 30;  // mSec
        public static final int mainFeedbackLoop = 0;

        public static final double CorrectionRotationSpeed = 2.0 ;  // meters/sec

        // public static final double maxAngleChangeForAlignFinish = 0.5;  // degrees
        // public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
        // public static final double alignTimeoutTime = 1000;
        // public static final double alignMemorySize = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 2;
        public static final int halfSpeedButton = 2;
        public static final int leftYAxis = 1;
        public static final int leftXAxis = 0;
        public static final int rightXAxis = 4;
        public static final int rightYAxis = 5;

        public static final String LogitechF310Name = "Controller (Gamepad F310)";

        public static final int gamepadPort = 1;
    }

    public final class RadioMaster_Controller {

        public static final int Left_Gimbal_X = 3;
        public static final int Right_Gimbal_X = 0;
        public static final int Right_Gimbal_Y = 1;

        public static final int SB3_Axis = 6 ;
        public static final int SF_Axis = 4 ;
        public static final int SE_Axis = 5 ;
        public static final int SH_Momentary = 4 ;
        public static final int SC_Button = 1 ;

        public static final double Forward_Axis_Attenuation = 1.0 ;
        public static final double Lateral_Axis_Attenuation = 1.0 ;
        public static final double Yaw_Axis_Attenuation = 0.6 ;

    }

    public final class Logitech_F310_Controller {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for buttons
        public static final int Button_A = 1;
        public static final int Button_B = 2;
        public static final int Button_X = 3;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }

    public final class Logitech_Dual_Action {

        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Right_Stick_X = 2;
        public static final int Right_Stick_Y = 3;

        // Constants for buttons
        public static final int Left_Trigger = 7;
        public static final int Right_Trigger = 8;
        public static final int Button_A = 2;
        public static final int Button_B = 3;
        public static final int Button_X = 1;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 9;
        public static final int Start_Button = 10;
        public static final int Left_Stick = 11;
        public static final int Right_Stick = 12;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }

    public final class Xbox_Controller {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for Buttons
        public static final int Button_A = 1;
        public static final int Button_B = 2;
        public static final int Button_X = 3;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }

    public final class Fight_Stick {
        public static final int Button_Y = 4;
        public static final int Button_B = 2 ;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
    }


    public static final class UnderGlowConstants {
        public static final SerialPort.Port port = SerialPort.Port.kUSB1;
        public static final int BlueAliance = 1;
        public static final int RedAliance = 2;
        public static final int NeonGreen = 3;
    }

    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = 4 ;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxSpeedMetersPerSecond = 2.25; // this works
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond / 0.5;
        // public static final double kAutoDriveDistanceInches = 1;
        // public static final double kAutoDriveSpeed = 0.2;

        public static final double AutoShootingTimeout = 0.75;
        public static final double AutoIndexRaiseTimeout = 1.00;
        public static final double AutoIndexLowerTimeout = 0.25;

        // these are the constants from waterbury
        // public static final double kPXController = 6.0;
        // public static final double kPYController = 6.0;
        // public static final double kPThetaController = 12.0;

        public static final double kPXController = 6.0 ;
        public static final double kPYController = 6.0 ;
//        public static final double kPThetaController = 4.0;  // this works
        public static final double kPThetaController = 8.0;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints 
            = new TrapezoidProfile.Constraints( kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared );
    }

    public static final class VisionConstants {
        public static final int ID_CameraLight = 5;
        public static final double AlignmentTolerence = 5.0 ; // pixels
    }

    public static final class ArmBarConstants {
        public static final int ID_rightMotor = 10;
        public static final int ID_leftMotor = 15;

        public static final int ID_HallEffectsA1 = 1;
        public static final int ID_HallEffectsA2 = 2;
        public static final int ID_HallEffectsB1 = 3;
        public static final int ID_HallEffectsB2 = 4;

        public static final int ID_ServoA1 = 1;
        public static final int ID_ServoA2 = 2;
        public static final int ID_ServoB1 = 3;
        public static final int ID_ServoB2 = 4;

        public static final double ClawServoLatchPosition = 0.0 ;
        public static final double ClawServoReleasePosition = 180.0 ;

        public static final int UnitsPerMotorRotation = 2048;
        public static final double GearboxGearRatio = 100.0 / 1.0; // farther gear to axel gear
        public static final double ChainGearRatio = 42.0 / 12.0; // (or 15/12) upper gear to lower gear
        public static final double UnitsPerArmRotation = UnitsPerMotorRotation * GearboxGearRatio * ChainGearRatio ;
        public static final double UnitsPerArmDegree = UnitsPerArmRotation / 360.0 ;

        public static final double DegreePerSecond = UnitsPerArmDegree / 10.0 ;

        public static final double MaxMotorRPMs = 6300 ;
        public static final double MaxMotorVelocity = UnitsPerMotorRotation * MaxMotorRPMs / 60.0 / 10.0 ; // per 100 mSec 

        public static final double MaxDegreesPerSecond = MaxMotorVelocity / DegreePerSecond ;

//        public static final double CruiseVelocity = 100 * DegreePerSecond ;
        public static final double CruiseVelocity = 60 * DegreePerSecond ;
        public static final double MaxAcceleration = CruiseVelocity / 0.5 ;  // get to max velocity in 1/2 second
        public static final double LettingGoCruiseVelocity = 35 * DegreePerSecond ;
        public static final double MegaCruiseVelocity = 70 * DegreePerSecond ;

        public static final int AccelerationSmoothing = 2 ; 


        public static final double AngleToNextArm = 33.0 ;

        // This is the angle to place the arm bar so that the driver can cause the hook to catch on the mid bar.
        public static final double MidBarRotationAngle = -55.0;

        // This is the minimum angle required for the arm bar hook to clear the high bar
        public static final double HighBarRotationAngle = 80 ;  // wpk need to refine this number

        // This is the minimum angle required for the arm bar to release the hook from the mid bar
        public static final double ReleaseMidBarAngle = 95 ; // wpk need to refine this number

        // This is the minimum angle required for the arm bar hook to clear the traverse bar
        public static final double TraverseBarRotationAngle = HighBarRotationAngle + 180 ;  // wpk need to refine this number

        // This is the minimum angle required for the arm bar to release the hook from the high bar
//        public static final double ReleaseHighBarAngle = ReleaseMidBarAngle + 180 ; // wpk need to refine this number
        public static final double ReleaseHighBarAngle = 100 ; // wpk need to refine this number
        

        // //public static final double TraverseBarRotationAngle = HighBarRotationAngle + 180.0 ;
        // public static final double FinalRestingAngle = TraverseBarRotationAngle + ( 90.0 - AngleToNextArm) + 90.0 ; // angle to hang straight down.
        public static final double AngleTolerance = 1.0;
        // public static final double LoadReducingAngle = 11.0 ;
        public static final double FreeingBClawAngle = 45.0 ;

        public static final int MinLatchCount = 10 ;


        // public static final double ConsistentRotationAngleTolerance = 6;

        // // public static final double DesiredArmVelocity = 0.5 * (180 - 2 * AngleToNextArm) / (Math.PI * Math.sqrt(14.25/386)) ; // degrees persecond
        // public static final double DesiredArmVelocity = 0.50 * (180 - 2 * AngleToNextArm) / (Math.PI * Math.sqrt(14.25/386)) ; // degrees persecond
        // public static final double CruiseVelocity = (UnitsPerArmDegree * DesiredArmVelocity ) / 10.0 ; // divided by 10 because Falcon Velocities are in 100 mSec units
        // // public static final double MaxAcceleration = CruiseVelocity * 0.75 ;
        // public static final double MaxAcceleration = CruiseVelocity * 0.25 ;
        // public static final int AccelerationSmoothing = 2 ;

        public static final double SlowArmVelocity = 10 ;
        public static final double SlowCruiseVelocity = (UnitsPerArmDegree * SlowArmVelocity ) / 10.0 ; // divided by 10 because Falcon Velocities are in 100 mSec units
        public static final double SlowMaxAcceleration = SlowCruiseVelocity * 0.75 ;

        public static final int Position_PID_id = 0;
        public static final double Position_kF = 0.048;  // wpk need to give this some thought
        public static final double Position_kP = 0.07;  // wpk need to give this some thought
        public static final double Position_kD = 0.01;  // wpk need to give this some thought
        public static final double Position_kI = 0.00003;  // wpk need to give this some thought

        public static final double manualVoltageRampingConstant = 0.05;
        public static final double closedVoltageRampingConstant = 0.05;

        // public static final int Velocity_PID_id = 1;
        // public static final double Velocity_kF = 0.0485;
        // public static final double Velocity_kP = 0.01;
        // public static final double Velocity_kD = 0.00;
    }

    public static final class IntakeConstants {
        public static final int ID_IntakeMotor = 3;
        
        public static final int UnitsPerMotorRotation = 2048;
        public static final double GearboxGearRatio = 1.0 / 1.0; 
        public static final double TotalUnitsPerRotation = (double) UnitsPerMotorRotation * GearboxGearRatio ;

        public static final double RevPerSecondVelocity = TotalUnitsPerRotation / 10.0 ; 
        public static final double RPM = RevPerSecondVelocity / 60.0 ;
        
        public static final double IntakeMotorSpeed = 2500.0 * RPM;
     //   public static final double IntakeMotorSpeed = 1700.0 * RPM;
        public static final int mainFeedbackLoop = 0;
        public static final int encoderTimeout = 0;

        public static final double closedVoltageRampingConstant = 0.0;
        public static final double manualVoltageRampingConstant = 0.0;
        public static final double kF = 0.048;
        public static final double kP = 0.005;
        public static final double kI = 0.0001;
        public static final double kD = 0.0;
        public static final int PID_id = 0;
    }

    public static final class ShooterConstants {
        public static final int UnitsPerMotorRotation = 2048;
        public static final double GearboxGearRatio = 1.0 / 1.0; // farther gear to axel gear
        public static final double TotalUnitsPerRotation = (double) UnitsPerMotorRotation * GearboxGearRatio ;

        public static final double RevPerSecondVelocity = TotalUnitsPerRotation / 10.0 ; 
        public static final double RPM = RevPerSecondVelocity / 60.0 ;

        public static final int ID_BeltMotor = 7;

        public static final double kF = 0.048;
        public static final double kP = 0.001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

//        public static final double FlyWheelMotorIntakeVelocity = -1800 * RPM;
//        public static final double FlyWheelMotorIntakeVelocity = -1900 * RPM;
        public static final double FlyWheelMotorIntakeVelocity = -2500 * RPM;
        public static final double BeltMotorIntakeVelocity = 900 * RPM;
        // public static final double BeltMotorIntakeVelocity = 650 * RPM;

        public static final int ID_FlyWheelMotor = 8;
        public static final double FlyWheelMotorLowGoalShootingVelocity = 2500 * RPM;
        public static final double FlyWheelMinShootingSpeed =  FlyWheelMotorLowGoalShootingVelocity * 0.95;
        public static final double BeltMotorLowGoalShootingVelocity = -FlyWheelMotorLowGoalShootingVelocity * 0.7;

        public static final double FlyWheelMotorHighGoalShootingVelocity = 5750 * RPM;
//        public static final double BeltMotorHighGoalShootingVelocity = -800 * RPM;
//        public static final double BeltMotorHighGoalShootingVelocity = -1200 * RPM;
        public static final double BeltMotorHighGoalShootingVelocity = -1400 * RPM;

        public static final double FlyWheelMotorEjectingVelocity = 2000 * RPM;
        public static final double BeltMotorEjectingVelocity = -1500 * RPM;

        public static final double ShooterTransitionTimeout = 1.25 ;

        public static final class Lift {
            public static final int ID_Extend_Solenoid = 0;    
            public static final int ID_Extend_Solenoid2 = 2;    
            public static final int ID_Retract_Solenoid = 1;
            public static final int ID_Retract_Solenoid2 = 3;
        }
    }
}
