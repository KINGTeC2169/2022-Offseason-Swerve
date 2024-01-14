package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.ModuleConstants;

import frc.robot.utils.Constants.DriveConstants;
import static frc.robot.utils.Constants.*;

public class SwerveJoystickCmd extends Command {

  //Initializes the solenoids for the 6 cannons
  private final Solenoid valve1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon1);
  private final Solenoid valve2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon2);
  private final Solenoid valve3 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon3);
  private final Solenoid valve4 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon4);
  private final Solenoid valve5 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon5);
  private final Solenoid valve6 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon6);

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> reset, valveOne, valveTwo, valveThree, valveFour, valveFive, valveSix;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
      Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction, 
      Supplier<Boolean> reset, Supplier<Boolean> valveOne, Supplier<Boolean> valveTwo, Supplier<Boolean> valveThree,
      Supplier<Boolean> valveFour, Supplier<Boolean> valveFive, Supplier<Boolean> valveSix) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.reset = reset;
    this.valveOne = valveOne;
    this.valveTwo = valveTwo;
    this.valveThree = valveThree;
    this.valveFour = valveFour;
    this.valveFive = valveFive;
    this.valveSix = valveSix;
    this.xLimiter = new SlewRateLimiter(5);
    this.yLimiter = new SlewRateLimiter(5);
    this.turningLimiter = new SlewRateLimiter(5);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {

    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = -turningSpdFunction.get();

    //Creates a deadband for controllers
    xSpeed = Math.abs(xSpeed) > .05 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > .05 ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > .02 ? turningSpeed : 0.0;
    

    xSpeed = xLimiter.calculate(xSpeed) *  ModuleConstants.maxNeoSpeed;
    ySpeed = yLimiter.calculate(ySpeed) *  ModuleConstants.maxNeoSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.maxNeoRadPerSec;

    System.out.println(xSpeed + "\t" + ySpeed + "\t" + turningSpeed);

    if (reset.get()){
      swerveSubsystem.resetEncoders();
      swerveSubsystem.zeroHeading();
    }

    ChassisSpeeds chassisSpeeds;
    //Switches between field oriented and robot oriented
    //Defintely not the best practice when it comes to programming 
    if(False/*fieldOrientedFunction.get()*/) {
      //Field oriented mode
      System.out.println(swerveSubsystem.getRotation2d());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    else {
      //Robot oriented mode
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
    
    //These if-else statements determine if the valves on the solenoids should be opened or not
    if(valveOne.get()) {
      valve1.set(true);
    }
    else {
      valve1.set(false);
    }
    if(valveTwo.get()) {
      valve2.set(true);
    }
    else {
      valve2.set(false);
    }
    if(valveThree.get()) {
      valve3.set(true);
    }
    else {
      valve3.set(false);
    }
    if(valveFour.get()) {
      valve4.set(true);
    }
    else {
      valve4.set(false);
    }
    if(valveFive.get()) {
      valve5.set(true);
    }
    else {
      valve5.set(false);
    }
    if(valveSix.get()) {
      valve6.set(true);
    }
    else {
      valve6.set(false);
    }
    
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
