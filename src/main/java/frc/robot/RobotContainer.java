// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand ;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.networktables.*;
import java.util.Map;

import java.lang.Runnable ;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("PMD.ExcessiveImports")
public class RobotContainer {
    // The robot's subsystems
    private final Drive driveSub = new Drive();

    // Joystick Variables
    Joystick driverController; //controller 2

    private int Forward_Speed_Axis;
    private int Lateral_Speed_Axis;
    private int Yaw_Axis;

    private double Forward_Speed_Attenuation = 0.5;
    private double Lateral_Speed_Attenuation = 0.5;
    private double Yaw_Attenuation = 0.5;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        createAutonomousCommands();


        driveSub.setDefaultCommand(
            new RunCommand(() -> {
                driveSub.driveCartesian(
                    -driverController.getY(), driverController.getX(), driverController.getZ(), driveSub.gyro.getAngle());
            }, driveSub)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        if (driverController == null) {
            System.out.println("null controller. Using joystick 2");
            driverController = new Joystick(2);
        }

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);
        boolean controllerFound = false;

        Forward_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_Y;
        Lateral_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_X;
        Yaw_Axis = Constants.RadioMaster_Controller.Left_Gimbal_X;

        Forward_Speed_Attenuation = Constants.RadioMaster_Controller.Forward_Axis_Attenuation;
        Lateral_Speed_Attenuation = Constants.RadioMaster_Controller.Lateral_Axis_Attenuation;
        Yaw_Attenuation = Constants.RadioMaster_Controller.Yaw_Axis_Attenuation;
    }

    private class InitPose implements Runnable {

        Drive drive ;
        Pose2d pose ;

        public InitPose( Drive d, Pose2d p) {
            drive = d ;
            pose = p ;
        }
        @Override
        public void run() {
            drive.resetOdometry(pose);
        }
    }

    public Command getAutonomousCommand() {
        return null;
    }



    private void createAutonomousCommands() {
    }
}