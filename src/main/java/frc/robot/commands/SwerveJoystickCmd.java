package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.ModuleConstants;

import frc.robot.utils.Constants.DriveConstants;
import static frc.robot.utils.Constants.*;

public class SwerveJoystickCmd extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  //private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Double> whammy;
  private final Supplier<Integer> strum;
  private final Supplier<Boolean> green, red, yellow, blue, orange;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> whammy, Supplier<Boolean> green, 
      Supplier<Boolean> red, Supplier<Boolean> blue, Supplier<Boolean> yellow, Supplier<Boolean> orange, 
      Supplier<Integer> strum) {
    this.swerveSubsystem = swerveSubsystem;
    //this.xSpdFunction = xSpdFunction;
    //this.ySpdFunction = ySpdFunction;
    //this.turningSpdFunction = turningSpdFunction;
    //this.fieldOrientedFunction = fieldOrientedFunction;
    //this.activeStopFunction = activeStopFunction;
    this.whammy = whammy;
    this.green = green;
    this.red = red;
    this.yellow = yellow;
    this.blue = blue;
    this.orange = orange;
    this.strum = strum;

    this.xLimiter = new SlewRateLimiter(26);
    this.yLimiter = new SlewRateLimiter(26);
    this.turningLimiter = new SlewRateLimiter(26);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    //double xSpeed = xSpdFunction.get();
    //double ySpeed = ySpdFunction.get();
    //double turningSpeed = turningSpdFunction.get();
    double turningSpeed = 0;
    

    
    double speed = (whammy.get() + 1) / 2;
    double xSpeed = 0;
    double ySpeed = 0;
    if(green.get()  && !blue.get()) {
      ySpeed = -speed;
    }
    if(red.get() && !yellow.get()) {
      xSpeed = -speed;
    }
    if(yellow.get() && !red.get()) {
      xSpeed = speed;
    }
    if(blue.get() && !green.get()) {
      ySpeed = speed;
    }

    if(strum.get() == 0)
      turningSpeed = -speed;
    else if(strum.get() == 180)
      turningSpeed = speed;

    // Deadband: unsure if necessary for our controllers
    //xSpeed = Math.abs(xSpeed) > .05 ? xSpeed : 0.0;
    //ySpeed = Math.abs(ySpeed) > .05 ? ySpeed : 0.0;
    //turningSpeed = Math.abs(turningSpeed) > .05 ? turningSpeed : 0.0;
    

    xSpeed = xLimiter.calculate(xSpeed) *  ModuleConstants.maxNeoSpeed;
    ySpeed = yLimiter.calculate(ySpeed) *  ModuleConstants.maxNeoSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.maxNeoRadPerSec;

    /*
    if(activeStopFunction.get()) {
      xSpeed = 0;
      ySpeed = 0;
      turningSpeed = 0;
      swerveSubsystem.setActiveStop();
    } */

    System.out.println(xSpeed + "\t" + ySpeed + "\t" + turningSpeed);

    ChassisSpeeds chassisSpeeds;
    if(False/*fieldOrientedFunction.get()*/) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

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
