// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.swerve.GyroIO;
import frc.robot.swerve.SDSModuleIOSim;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    private final CommandXboxController driverController;

    private final SwerveDrive swerve;

    public RobotContainer() {

        Preferences.removeAll();

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

        swerve = new SwerveDrive(
            new GyroIO() {},
            new SDSModuleIOSim(),
            new SDSModuleIOSim(),
            new SDSModuleIOSim(),
            new SDSModuleIOSim()
        );
        
        configureBindings();

    }

    private void configureBindings() {
        // swerve default joystick inputs
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,          // vx
            driverController::getLeftY,          // vy
            driverController::getRightX,         // omega
            driverController::getRightY,
            driverController::getLeftTriggerAxis // raw slow input
        ));

        // reset robot orientation (doesn't work with vision or FMS)
        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.a().onTrue(swerve.togglePos());


        // swerve adjustments using POV
        // auxController.povUp().onTrue(swerve.runXSetTime(-0.15));
        // auxController.povDown().onTrue(swerve.runXSetTime(0.15));
        // auxController.povLeft().onTrue(swerve.runOmegaSetTime(0.05));
        // auxController.povRight().onTrue(swerve.runOmegaSetTime(-0.05));

        // shooter flywheel on/off


    }

    public void testPeriodic() {
        swerve.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}