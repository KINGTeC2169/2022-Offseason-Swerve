package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.DriveConstants;

public class FollowAprilTagCmd extends CommandBase{
    
    private SwerveSubsystem swerveSubsystem;

    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private SwerveDrivePoseEstimator m_PoseEstimator;

    public FollowAprilTagCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        m_PoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, NavX.getCurrentRotation(), swerveSubsystem.getModulePositions(), new Pose2d());
        addRequirements(swerveSubsystem);
    }   

    @Override
    public void initialize(){}

    @Override
    public void execute(){

        double roboPitch = NavX.getX();
        double roboRoll = NavX.getY();
        double roboYaw = NavX.getZ();

        chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

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
