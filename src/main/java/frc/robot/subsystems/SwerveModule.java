package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
    /* * * INITIALIZATION * * */
    //initialize motors 
    private CANSparkMax driveMotor; 
    private CANSparkMax rotationMotor; 

    //initialize encoders 
    private WPI_CANCoder absoluteEncoder; 
    private RelativeEncoder driveEncoder; 
    //private RelativeEncoder rotationEncoder; 

    //init PID Controller for turning 
    private PIDController rotationPID; 

    /* * * CONSTRUCTOR * * */
    /* 
     * @param drivePort port of drive motor 
     * @param rotationPort port of rotation motor 
     * @param absoluteEncoderPort port of CANCoder (absolute encoder) 
     * @param encoderOffset offset of absolute encoder 
     * @param driveInverted is the drive motor inverted? 
     * @param rotationInverted is the rotation motor inverted? 
     */
    public SwerveModule(int drivePort, int rotationPort, int absoluteEncoderPort, double encoderOffset, boolean driveInverted, boolean rotationInverted) {
        //instantiate drive motor and encoder 
        driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless); 
        driveEncoder = driveMotor.getEncoder();

        //instantiate rotation motor and absolute encoder 
        rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        absoluteEncoder = new WPI_CANCoder(absoluteEncoderPort);

        //reset all motor configuration (as suggested from raid zero) (optional but safe)
        driveMotor.restoreFactoryDefaults();
        rotationMotor.restoreFactoryDefaults();
        absoluteEncoder.configFactoryDefault();

        /* * * DRIVE * * */
        //configure driving motor 
        driveMotor.setInverted(driveInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);

        //set conversion factor for drive enc 
        //reads velocity in meters per second instead of RPM 
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION);
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION);

        /* * * ROTATION * * */
        //configure rotation motor 
        rotationMotor.setInverted(rotationInverted);
        rotationMotor.setIdleMode(IdleMode.kBrake);

        //configure rotation absolute encoder 
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); //abs enc is now +-180 
        absoluteEncoder.configMagnetOffset(encoderOffset); //implements encoder offset?? untested 
        absoluteEncoder.configSensorDirection(SwerveConstants.ROTATION_ENCODER_DIRECTION); //False (default) means positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder.
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        //configure rotation PID controller 
        rotationPID = new PIDController(
            SwerveConstants.KP_TURNING, 
            SwerveConstants.KI_TURNING, 
            SwerveConstants.KD_TURNING);
        rotationPID.enableContinuousInput(-180, 180); //Continuous input considers min & max to be the same point; calculates the shortest route to the setpoint 
    } 

    /* * * GET METHODS * * */
    private double driveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double drivePosition() {
        return driveEncoder.getPosition();
    }

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()));
    }

    public void setState(SwerveModuleState state) {
        //optimize state so the rotation motor doesnt have to spin as much 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); 
        //driveMotor.set(optimizedState.speedMetersPerSecond); 

    }
}