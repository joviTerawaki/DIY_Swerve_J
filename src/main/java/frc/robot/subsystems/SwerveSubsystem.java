package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

public class SwerveSubsystem extends SubsystemBase {

  //initialize SwerveModules 
  // private SwerveModule frontLeft, backLeft, frontRight, backRight; 
  private SwerveModule[] swerveModules; 

  private SwerveDriveOdometry odometer; 
  private AHRS navx; 

  public SwerveSubsystem() {

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), 
      new SwerveModule(1, SwerveConstants.BackLeft.constants), 
      new SwerveModule(2, SwerveConstants.FrontRight.constants), 
      new SwerveModule(3, SwerveConstants.BackRight.constants)
    };

    //instantiation of SwerveModules 
    // frontLeft = new SwerveModule(
    //   SwerveConstants.FL_DRIVE_PORT,
    //   SwerveConstants.FL_ROTATION_PORT,
    //   SwerveConstants.FL_ABSOLUTE_ENCODER_PORT,
    //   SwerveConstants.FL_OFFSET,
    //   false, 
    //   true
    // ); //ba was here hahaa !!!

    // backLeft = new SwerveModule(
    //   SwerveConstants.BL_DRIVE_PORT, 
    //   SwerveConstants.BL_ROTATION_PORT, 
    //   SwerveConstants.BL_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.BL_OFFSET, 
    //   true, 
    //   true
    // );

    // frontRight = new SwerveModule(
    //   SwerveConstants.FR_DRIVE_PORT, 
    //   SwerveConstants.FR_ROTATION_PORT, 
    //   SwerveConstants.FR_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.FR_OFFSET, 
    //   false, 
    //   true
    // );

    // backRight = new SwerveModule(
    //   SwerveConstants.BR_DRIVE_PORT, 
    //   SwerveConstants.BR_ROTATION_PORT, 
    //   SwerveConstants.BR_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.BR_OFFSET, 
    //   true, 
    //   true
    // );

    //instantiate navx 
    navx = new AHRS();
    navx.zeroYaw();

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      navx.getRotation2d(), 
      getModulePositions()
    );

    //CONFIGURE AUTO BUILDER FOR HOLONOMIC DRIVE 
    //FIXME idk if the poseSupplier (first param) is odometer obj or a method updating the odometer 
    //HOW DO U GET A POSE SUPPLIER 
    // AutoBuilder.configureHolonomic(null, null, null, null, null, null);
  }

  //returns the Rotation2d object 
  //a 2d coordinate represented by a point on the unit circle (the rotation of the robot)
  public Rotation2d getRotation2d() {
    // return Rotation2d.fromDegrees(navx.getYaw());
    return navx.getRotation2d();
  }

  public void resetNavx() {
    navx.reset();
  }

  /* * * ODOMETRY * * */
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // FIXME i dont think this works as intended,, resetPosition should reset everything to 0 
  public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /* * * STATES * * */

  //SET STATES 
  //gets a SwerveModuleStates array from driver control and sets each module 
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }

    // frontLeft.setState(desiredStates[0]);
    // backLeft.setState(desiredStates[1]);
    // frontRight.setState(desiredStates[2]);
    // backRight.setState(desiredStates[3]);
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states; 
    // return new SwerveModulePosition[] {
      // frontLeft.getState(), 
      // backLeft.getState(), 
      // frontRight.getState(), 
      // backRight.getState()
    // }
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
    // return new SwerveModulePosition[] {
      // frontLeft.getPosition(), 
      // backLeft.getPosition(), 
      // frontRight.getPosition(), 
      // backRight.getPosition()
    // };
  }

  //LOCK 
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }

    // frontLeft.setAngle(fl);
    // backLeft.setAngle(bl);
    // frontRight.setAngle(fr);
    // backRight.setAngle(br);
  }

  //STRAIGHTEN THE WHEELS 
  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }

    // frontLeft.setAngle(fl);
    // backLeft.setAngle(bl);
    // frontRight.setAngle(fr);
    // backRight.setAngle(br);
  }

  //STOP 
  public void stopModules() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }

    // frontLeft.stop();
    // backLeft.stop();
    // backRight.stop();
    // frontRight.stop();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(navx.getRotation2d(), getModulePositions());
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }
    // frontLeft.print();
    // backLeft.print();
    // frontRight.print();
    // backRight.print();
    SmartDashboard.putNumber("NAVX", navx.getYaw());
  }
}
