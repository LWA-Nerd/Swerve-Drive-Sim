package frc.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{

    private SwerveModuleSim[] modules = {new SwerveModuleSim(), new SwerveModuleSim(), new SwerveModuleSim(), new SwerveModuleSim()};
    
    private DriveStates currentState = DriveStates.IDLE;
    
    public enum DriveStates {
        IDLE, 
        FIELD_ORIENTED, 
        ROBOT_ORIENTED,
        SLOW,
        GOOOOOFY
    }

    @Override
    public void periodic() {
        
    }
}
