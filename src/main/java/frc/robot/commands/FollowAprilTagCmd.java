package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.DriveConstants;

public class FollowAprilTagCmd extends CommandBase{    
    
    private SwerveSubsystem swerveSubsystem;

    private ChassisSpeeds chassisSpeeds;
    private double turningSpeed;
    private PIDController xController;
    private PIDController yController;
 
    public FollowAprilTagCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        xController = new PIDController(0.43, 0, 0);
        yController = new PIDController(0.16, 0, 0);
        addRequirements(swerveSubsystem);
    }   

    @Override
    public void initialize(){}

    @Override
    public void execute(){

        turningSpeed = 0;
        
        //If the limelight sees an Apriltag, it will try to position itself in front of it
        if (LimelightTable.getTV()){
            chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(xController.calculate(LimelightTable.getTA(), 1.7) * 0.65, 
                                                                   yController.calculate(LimelightTable.getTX(), 0) * 0.3, 
                                                                   turningSpeed, 
                                                                   swerveSubsystem.getRotation2d());

            SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }
        //Stops the swerve modules if no Apriltag is seen
        else {
            swerveSubsystem.stopModules();
        }

        //System.out.println(LimelightTable.getTX() + "\t" + LimelightTable.getTY() + "\t" + LimelightTable.getTS());

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
