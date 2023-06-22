package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSubsystem extends SubsystemBase {

  //initialize SwerveModules 
  private SwerveModule frontLeft, backLeft, frontRight, backRight; 

  private SwerveDriveOdometry odometer; 
  private AHRS navx; 

  public SwerveSubsystem() {
    //instantiation of SwerveModules 
    frontLeft = new SwerveModule(
      SwerveConstants.FL_DRIVE_PORT,
      SwerveConstants.FL_ROTATION_PORT,
      SwerveConstants.FL_ABSOLUTE_ENCODER_PORT,
      SwerveConstants.FL_OFFSET,
      true, 
      true
    ); 

    backLeft = new SwerveModule(
      SwerveConstants.BL_DRIVE_PORT, 
      SwerveConstants.BL_ROTATION_PORT, 
      SwerveConstants.BL_ABSOLUTE_ENCODER_PORT, 
      SwerveConstants.BL_OFFSET, 
      true, 
      true
    );

    frontRight = new SwerveModule(
      SwerveConstants.FR_DRIVE_PORT, 
      SwerveConstants.FR_ROTATION_PORT, 
      SwerveConstants.FR_ABSOLUTE_ENCODER_PORT, 
      SwerveConstants.FR_OFFSET, 
      true, 
      true
    );

    backRight = new SwerveModule(
      SwerveConstants.BR_DRIVE_PORT, 
      SwerveConstants.BR_ROTATION_PORT, 
      SwerveConstants.BR_ABSOLUTE_ENCODER_PORT, 
      SwerveConstants.BR_OFFSET, 
      true, 
      true
    );

    //instantiate navx 
    navx = new AHRS();
    navx.zeroYaw();

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRotation2d(), 
      getModulePositions()
    );
  }

  //returns the Rotation2d object 
  //a 2d coordinate represented by a point on the unit circle (the rotation of the robot)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  /* * * POSE * * */
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /* * * STATES * * */

  //SET STATES 
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
    frontLeft.setState(desiredStates[0]);
    backLeft.setState(desiredStates[1]);
    frontRight.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), 
      backLeft.getState(), 
      frontRight.getState(), 
      backRight.getState()
    };
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      backLeft.getPosition(), 
      frontRight.getPosition(), 
      backRight.getPosition()
    };
  }

  //STOP 
  public void stopModules() {
    frontLeft.stop();
    backLeft.stop();
    backRight.stop();
    frontRight.stop();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());
  }
}
