package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleSim {
    private final DCMotorSim driveMotor;
    private final DCMotorSim turnMotor;

    private static final DCMotor driveGearBox = DCMotor.getNEO(1);
    private static final DCMotor turnGearBox = DCMotor.getNEO(1);

    private final PIDController drivePID = new PIDController(Constants.DriveConstants.driveP, Constants.DriveConstants.driveI, Constants.DriveConstants.driveD);
    private final PIDController turnPID = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI, Constants.DriveConstants.turnD);

    public SwerveModuleSim() {
    }
}
