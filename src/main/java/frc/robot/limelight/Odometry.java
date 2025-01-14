// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Odometry extends SubsystemBase {

  SwerveDriveKinematics swerveKinematics = null;
  Rotation2d gyroAngle = null;
  SwerveModulePosition[] modulePositions = null;
  Pose2d initialPose = null;

  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, gyroAngle, modulePositions, initialPose);

  double tx = 0.0;
  double ty = 0.0;
  double ta = 0.0;

  boolean isUpdateRejected = true;
  boolean debugging = true;

  /** Creates a new Odometry. */
  public Odometry(SwerveDriveKinematics swerveKinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
    this.gyroAngle = gyroAngle;
    this.initialPose = initialPose;
    this.modulePositions = modulePositions;
    this.swerveKinematics = swerveKinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    poseUpdate();

    if (debugging) {
      setupSmartdashboard();
    }



  }

  public void poseUpdate() {

    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    
    if (megaTag2.tagCount == 1 && megaTag2.rawFiducials.length == 1) {
      if(megaTag2.rawFiducials[0].ambiguity > 0.7) {
        isUpdateRejected = true;
      } 
      else if (megaTag2.rawFiducials[0].distToCamera > 3) {
        isUpdateRejected = true;
      }
    }
    else if (megaTag2.tagCount == 0) {
      isUpdateRejected = true;
    }
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    else if(Math.abs(RobotContainer.m_swerve.getGyroRate()) > 720) 
      {
        isUpdateRejected = true;
      }

     if(!isUpdateRejected)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
      }

      tx = LimelightHelpers.getTX("");
      ty = LimelightHelpers.getTY("");
      ta = LimelightHelpers.getTA("");
  }

  public void setupSmartdashboard() {

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);

    // ?
    SmartDashboard.putNumber("gyro angle", gyroAngle.getDegrees());
    SmartDashboard.putNumber("initial pose x", initialPose.getX());
    SmartDashboard.putNumber("initial pose y", initialPose.getY());

    SmartDashboard.putNumber("estimated pose distance x", m_poseEstimator.getEstimatedPosition().getMeasureX().abs(Units.Meters));
    SmartDashboard.putNumber("estimated pose distance y", m_poseEstimator.getEstimatedPosition().getMeasureY().abs(Units.Meters));


  }

 
}
