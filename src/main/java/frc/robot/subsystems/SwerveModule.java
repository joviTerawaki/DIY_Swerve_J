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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
    /* * * INITIALIZATION * * */

    public int moduleID; 
    //initialize motors 
    private CANSparkMax driveMotor; 
    private CANSparkMax rotationMotor; 

    //initialize encoders 
    private WPI_CANCoder absoluteEncoder; 
    private RelativeEncoder driveEncoder; 
    //private RelativeEncoder rotationEncoder; 

    //init PID Controller for turning 
    private PIDController rotationPID; 

    //init info 
    //private int port; 
    private double encOffset; 

    //testing 
    // SwerveModuleState desState; 

    /* * * CONSTRUCTOR * * */
    /* 
     * @param drivePort port of drive motor 
     * @param rotationPort port of rotation motor 
     * @param absoluteEncoderPort port of CANCoder (absolute encoder) 
     * @param encoderOffset offset of absolute encoder 
     * @param driveInverted is the drive motor inverted? 
     * @param rotationInverted is the rotation motor inverted? 
     */
    public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID; 
        
        encOffset = moduleConstants.angleOffset;
        //instantiate drive motor and encoder 
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
        driveEncoder = driveMotor.getEncoder();

        //instantiate rotation motor and absolute encoder 
        rotationMotor = new CANSparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        absoluteEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        //reset all motor configuration (as suggested from raid zero) (optional but safe)
        driveMotor.restoreFactoryDefaults();
        rotationMotor.restoreFactoryDefaults();
        absoluteEncoder.configFactoryDefault();

        /* * * DRIVE * * */
        //configure driving motor 
        driveMotor.setInverted(moduleConstants.driveInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);

        //set conversion factor for drive enc 
        //reads velocity in meters per second instead of RPM 
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION);
        //reads velocity in meters instead of rotations 
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION);

        /* * * ROTATION * * */
        //configure rotation motor 
        rotationMotor.setInverted(moduleConstants.rotationInverted);
        rotationMotor.setIdleMode(IdleMode.kBrake);

        //configure rotation absolute encoder 
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); //abs enc is now +-180 
        absoluteEncoder.configMagnetOffset(moduleConstants.angleOffset); //implements encoder offset?? untested 
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

    private double getAbsoluteEncoderDegrees() {
        return absoluteEncoder.getAbsolutePosition(); // + (encOffset);
    }

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    /* * * SET METHODS * * */

    public void setState(SwerveModuleState desiredState) {
        //optimize state so the rotation motor doesnt have to spin as much 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); 
        //driveMotor.set(optimizedState.speedMetersPerSecond); 

        // desState = desiredState; 
    }

    public void setAngle(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput); 
        driveMotor.set(0);
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC RAD", Math.toRadians(getAbsoluteEncoderDegrees()));
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] DRIVE SPEED", driveVelocity());
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] ROTATION SPEED", absoluteEncoder.getVelocity());
        SmartDashboard.putString("S["+absoluteEncoder.getDeviceID()+"] CURRENT STATE", getState().toString());

        // SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] DESIRED STATE", desState.toString());

    }
}
