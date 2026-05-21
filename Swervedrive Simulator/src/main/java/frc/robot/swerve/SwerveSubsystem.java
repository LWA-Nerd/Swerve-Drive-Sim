package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    
    private DriveStates currentState = DriveStates.IDLE;
    
    public enum DriveStates {
        IDLE, 
        FIELD_ORIENTED, 
        ROBOT_ORIENTED
    }

    @Override
    public void periodic() {
        
    }
}
