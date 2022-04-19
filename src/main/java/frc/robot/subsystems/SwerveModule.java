package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANSparkMax neoTurn;
    private CANCoder absoluteEncoder;
    
    private PIDController turningPID;
    private CANCoderConfiguration config;

    public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, boolean isCancoderReveresed) {
        driveMotor = new TalonFX(driveMotorID);
        //turnMotor = new TalonFX(turnMotorID);
        neoTurn = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.sensorDirection = isCancoderReveresed;

        //TODO: should it be signed or unsigned
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        absoluteEncoder = new CANCoder(canCoderID);
        absoluteEncoder.configAllSettings(config);
        
        driveMotor.config_kP(0, .25);
        turningPID = new PIDController(ModuleConstants.PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurnPosition() {
        return neoTurn.getEncoder().getPosition();
        //return turnMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.driveEncoderRPMToMeterPerSec;
    }

    public double getTurnVelocity() {
        return neoTurn.getEncoder().getVelocity();
        //return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteTurnPosition()));
    }

    //TODO: normalize drive power values!!(I forgor where we should do this)
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond);
        neoTurn.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
        //turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        neoTurn.set(0);
        //turnMotor.set(ControlMode.PercentOutput, 0);
    }
}
