// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.Ports;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.math.*;


/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain driveTrain;

  private TalonSRX rMain = new TalonSRX(1);
  private TalonSRX rOne = new TalonSRX(2);
  private TalonSRX rTwo = new TalonSRX(3);
  private TalonSRX lMain = new TalonSRX(4);
  private TalonSRX lOne = new TalonSRX(5);
  private TalonSRX lTwo = new TalonSRX(6);
  private Control control = new Control();

  private final Supplier<Boolean> valveOne, valveTwo, valveThree, valveFour, valveFive, valveSix;
  
  private final Solenoid valve1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon1);
  private final Solenoid valve2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon2);
  private final Solenoid valve3 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon3);
  private final Solenoid valve4 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon4);
  private final Solenoid valve5 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon5);
  private final Solenoid valve6 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon6);

  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public DriveCommand(Drivetrain driveTrain, 
  Supplier<Boolean> valveOne, Supplier<Boolean> 
  valveTwo, Supplier<Boolean> valveThree,
  Supplier<Boolean> valveFour, Supplier<Boolean> 
  valveFive, Supplier<Boolean> valveSix){

    this.driveTrain = driveTrain;
    this.valveOne = valveOne;
    this.valveTwo = valveTwo;
    this.valveThree = valveThree;
    this.valveFour = valveFour;
    this.valveFive = valveFive;
    this.valveSix = valveSix;

    rMain.set(ControlMode.PercentOutput,0); // the % output of the motor, between -1 and 1
    rOne.set(ControlMode.PercentOutput,0);
    rTwo.set(ControlMode.PercentOutput,0);
    lMain.set(ControlMode.PercentOutput,0);
    lOne.set(ControlMode.PercentOutput,0);
    lTwo.set(ControlMode.PercentOutput,0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void rDrive(double power) {
    rMain.set(ControlMode.PercentOutput, -power);
    rOne.set(ControlMode.PercentOutput, power);
    rTwo.set(ControlMode.PercentOutput, power);
  }

  public void lDrive(double power) {
    lMain.set(ControlMode.PercentOutput, power);
    lOne.set(ControlMode.PercentOutput, -power);
    lTwo.set(ControlMode.PercentOutput, -power);
  }

  private double leftPow = 0;
  private double rightPow = 0;

  private double multiplier = 0.6;

  public void xTurn(){

    leftPow = Control.getLeftControllerY() * multiplier; //One Joystick moves forward and backward
    rightPow = Control.getRightControllerY() * multiplier;
    
    lDrive(leftPow);
    rDrive(rightPow);
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    xTurn();

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
