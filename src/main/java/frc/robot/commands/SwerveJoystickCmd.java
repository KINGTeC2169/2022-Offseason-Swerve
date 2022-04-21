package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.utils.Constants.DriveConstants;

public class SwerveJoystickCmd extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
      Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(26);
    this.yLimiter = new SlewRateLimiter(26);
    this.turningLimiter = new SlewRateLimiter(26);
    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    /* Deadband: unsure if necessary for our controllers
    xSpeed = Math.abs(xSpeed) > 1 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 1 ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > 1 ? turningSpeed : 0.0;
    */

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    turningSpeed = turningLimiter.calculate(turningSpeed);


    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  
  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
