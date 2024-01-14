package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static frc.robot.utils.Constants.ModuleConstants.*;

public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax neoTurn;
    private CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private PIDController drivePID;
    private PIDController turningPID;
    private CANcoderConfiguration configTest = new CANcoderConfiguration();

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed, boolean isMK3) {

        //Creates and configures motors
        neoTurn = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        neoTurn.setInverted(turnMotorReversed);
        driveMotor.setInverted(driveMotorReversed);

        //Configuration for CANCoder
        configTest.MagnetSensor.MagnetOffset = Units.radiansToDegrees(absoluteOffset);    
        // configTest.MagnetSensor.sensorCoefficient = 2 * Math.PI / 4096.0;
        // configTest.MagnetSensor.UnitString = "rad";
        // configTest.MagnetSensor.SensorTimeBase = SensorTimeBase.PerSecond;
        // configTest.MagnetSensor.SensorDirection = isCancoderReversed;

        absoluteEncoder = new CANcoder(canCoderID);
        absoluteEncoder.getConfigurator().apply(configTest);

        //Configures integrated motor encoders
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = neoTurn.getEncoder();

        //extra stuff to compensate the different marks of swerve wheels
        if (isMK3) {
            driveEncoder.setPositionConversionFactor(mk3DriveEncoderToMeter);
            driveEncoder.setVelocityConversionFactor(mk3DriveEncoderRPMToMeterPerSec);
        }

        else{
            driveEncoder.setPositionConversionFactor(mk4DriveEncoderToMeter);
            driveEncoder.setVelocityConversionFactor(mk4DriveEncoderRPMToMeterPerSec);
        }

        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        drivePID = new PIDController(PDrive, 0, 0);

        resetEncoders();
    }

    /**
     * Gets the drive position from the drive encoder
     * 
     * @return drivePosition -the position of the relative encoder
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getTurnPosition());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    /**
     * Gets the turn position from the turn encoder
     * 
     * @return turnPosition -the posiiton of the relative encoder
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**
     * Gets the velocity from he drive encoder
     * 
     * @return driveVelocity -returns the velocity from the relative encoder
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the velocity from he turn encoder
     * 
     * @return turnVelocity -returns the velocity from the relative encoder
     */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    /**
     * Gets the absolute encoder value
     * 
     * @return absoluteTurnPosition -returns the position from the absolute encoder
     */
    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition().getValue();
        //return 0; 
    }

    /**
     * Resets the drive encoders to position zero and the turn encoders to the value provided by the absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            //TODO: this causes an aggresive stop, need to test with more weight on bot
            semiAutoStop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / maxNeoSpeed);
        neoTurn.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));    }

    /**
     * sets all motors to zero
     */
    public void stop() {
        driveMotor.set(0);
        neoTurn.set(0);
    }

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        driveMotor.set(drivePID.calculate(getDriveVelocity(), 0));
        neoTurn.set(0);
    }

    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(drivePID.calculate(getDriveVelocity(), 0));
        neoTurn.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }
}
