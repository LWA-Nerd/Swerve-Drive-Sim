package frc.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private final PIDController drivePID = new PIDController(
        Constants.DriveConstants.driveP,
        Constants.DriveConstants.driveI,
        Constants.DriveConstants.driveD
    );
    private final PIDController turnPID = new PIDController(
        Constants.DriveConstants.turnP,
        Constants.DriveConstants.turnI,
        Constants.DriveConstants.turnD
    );

    private double m_drivePositionRad = 0.0;
    private double m_driveVelocityRadPerSec = 0.0;

    private double m_steerPositionRad = 0.0;
    private double m_steerVelocityRadPerSec = 0.0;

    private double m_driveVoltage = 0.0;
    private double m_steerVoltage = 0.0;

    public SwerveModuleSim(String name) {
        this.name = name;

        driveMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveGearBox,
                Constants.DriveConstants.momentOfInertia,
                Constants.DriveConstants.driveGearRatio
            ),
            driveGearBox,
            new double[]{0.0, 0.0}
        );

        turnMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                turnGearBox,
                Constants.DriveConstants.momentOfInertia,
                Constants.DriveConstants.turnGearRatio
            ),
            turnGearBox,
            new double[]{0.0, 0.0}
        );

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getAngle()
        );
        double targetSpeed = desiredState.speedMetersPerSecond;
        double currentSpeed = getDriveVelocityMetersPerSec();

        m_driveVoltage = drivePID.calculate(
            currentSpeed,
            targetSpeed
        );

        m_steerVoltage = turnPID.calculate(
            getAngle().getRadians(),
            desiredState.angle.getRadians()
        );
        
    }


    public void simulationPeriodic() {
        driveMotor.setInputVoltage(clamp(m_driveVoltage, -12.0, 12.0));
        turnMotor.setInputVoltage(clamp(m_steerVoltage, -12.0, 12.0));

        driveMotor.update(0.020);
        turnMotor.update(0.020);

        m_driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
        m_drivePositionRad += m_driveVelocityRadPerSec * 0.020;

        m_steerPositionRad = turnMotor.getAngularPositionRad();
        m_steerVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
    }

    public Rotation2d getAngle() {
        return new Rotation2d(m_steerPositionRad / Constants.DriveConstants.turnGearRatio);
    }

    public double getDriveVelocityMetersPerSec() {
        return (m_driveVelocityRadPerSec / Constants.DriveConstants.driveGearRatio)
                * Constants.DriveConstants.wheelRadiusMeters;
    }

    public double getDrivePositionMeters() {
        return (m_drivePositionRad / Constants.DriveConstants.driveGearRatio)
                * Constants.DriveConstants.wheelRadiusMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getAngle());
    }

    @Override
    public String toString() {
        return name;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}