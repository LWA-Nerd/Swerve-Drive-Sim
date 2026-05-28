package frc.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleSim {
    private String name;

    private final DCMotorSim driveMotor;
    private final DCMotorSim turnMotor;

    private static final DCMotor driveGearBox = DCMotor.getNEO(1);
    private static final DCMotor turnGearBox = DCMotor.getNEO(1);

    private final PIDController drivePID = new PIDController(Constants.DriveConstants.driveP, Constants.DriveConstants.driveI, Constants.DriveConstants.driveD);
    private final PIDController turnPID = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI, Constants.DriveConstants.turnD);

    private double xTranslation;
    private double yTranslation;

    public SwerveModuleSim(String name) {
        this.name = name;

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            driveGearBox, 
            Constants.DriveConstants.momentOfInertia,
            Constants.DriveConstants.gearing), 
            driveGearBox
        );

        turnMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            turnGearBox, 
            Constants.DriveConstants.momentOfInertia,
            Constants.DriveConstants.gearing), 
            turnGearBox
        );
        // driveGearBox.getVoltage(driveMotor.getTorqueNewtonMeters(), driveMotor.getAngularVelocityRadPerSec()), driveGearBox.getCurrent(driveMotor.getTorqueNewtonMeters())
    }

}
