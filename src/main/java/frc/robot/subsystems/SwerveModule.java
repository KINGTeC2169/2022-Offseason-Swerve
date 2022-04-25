package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule {
    //private TalonFX driveMotor;
    //private TalonFX turnMotor;
    private CANSparkMax driveMotor;
    private CANSparkMax neoTurn;
    private CANCoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private PIDController drivePID;
    private PIDController turningPID;
    private CANCoderConfiguration configTest = new CANCoderConfiguration();

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed, boolean turnEncoderReversed) {
        //driveMotor = new TalonFX(driveMotorID);
        //turnMotor = new TalonFX(turnMotorID);
        neoTurn = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        neoTurn.setInverted(turnMotorReversed);
        driveMotor.setInverted(driveMotorReversed);

        configTest.magnetOffsetDegrees = Units.radiansToDegrees(absoluteOffset);
        configTest.sensorCoefficient = 2 * Math.PI / 4096.0;
        configTest.unitString = "rad";
        
        configTest.sensorTimeBase = SensorTimeBase.PerSecond;
        configTest.sensorDirection = isCancoderReversed;

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = neoTurn.getEncoder();
        //turnEncoder.setInverted(turnEncoderReversed);

        configTest.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        absoluteEncoder = new CANCoder(canCoderID);
        absoluteEncoder.configAllSettings(configTest);
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderToMeter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderRPMToMeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.turnEncoderRPMToRadPerSec);

        //driveMotor.config_kP(0, .25);
        turningPID = new PIDController(ModuleConstants.PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        drivePID = new PIDController(.5, 0, 0);

        resetEncoders();
    }

    
    public double getDrivePosition() {
        //return driveMotor.getSelectedSensorPosition();
        return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
        //return turnMotor.getSelectedSensorPosition();
    }

    
    public double getDriveVelocity() {
        //return driveMotor.getSelectedSensorVelocity();
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
        //return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \n.\n.\n.\n.\n.\n.\n.\n.");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            semiAutoStop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        //driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / ModuleConstants.maxSpeed);
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.maxNeoSpeed);
        neoTurn.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
        //turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        neoTurn.set(0);
        //turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void semiAutoStop() {
        driveMotor.set(drivePID.calculate(getDriveVelocity(), 0));
        neoTurn.set(0);
    }

    public void activeStop(int direction) {
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(drivePID.calculate(getDriveVelocity(), 0));
        neoTurn.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }
}
