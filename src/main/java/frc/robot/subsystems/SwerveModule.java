package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule {
    private TalonSRX driveMotor;
    private TalonSRX turnMotor;
    private CANCoder absoluteEncoder;
    
    private PIDController turningPID;
    private CANCoderConfiguration config;

    public SwerveModule(int driveMotorID, int turnMotorID, int canEncoderID, boolean isCancoderReveresed) {
        driveMotor = new TalonSRX(driveMotorID);
        turnMotor = new TalonSRX(turnMotorID);

        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.sensorDirection = isCancoderReveresed;

        //TODO: should it be signed or unsigned
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        absoluteEncoder = new CANCoder(canEncoderID);
        absoluteEncoder.configAllSettings(config);
        
        
        turningPID = new PIDController(ModuleConstants.PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurnPosition() {
        return turnMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        //TODO convert to meters per second
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond);
        turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }
}
