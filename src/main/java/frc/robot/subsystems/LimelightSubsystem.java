// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;


public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new Limelight. */
    public LimelightSubsystem() {}

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /**
     * Gets the yaw angle from the Limelight to the middle of the apriltag
     */
    /*public double getTargetRotation() {
        // Returns a rotation value if the limelight has a valid apriltag target
        if (LimelightHelpers.getTV("") == true) {
            return getCameraTransform(4); // I hope this is yaw
        }
        // Otherwise, return 0
        return 0;
    }

    /**
     * Returns the angle the wrist should be at to shoot the speaker
     */
   /*  public double getLimelightWristAngle() {
        double adjacent = getCameraTransform(2) + LimelightConstants.wristPivotOffsetX - LimelightConstants.speakerOffsetX;
        double opposite = LimelightConstants.speakerHeight - LimelightConstants.wristPivotOffsetY;
        return (LimelightConstants.wristAngleOffset + (Math.atan(opposite/adjacent)*180/Math.PI));
    }


    // Custom method to get camtran from network table
    // 0,                   1,         2,                  3,     4,   5
    // x(lateral distance), y(height), z(length distance), pitch, yaw, roll
    public double getCameraTransform(int index) {
        double[] camtrans = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
        return camtrans[index];
    }

*/
    @Override
    public void periodic() {
        SmartDashboard.putNumber("getTX", LimelightHelpers.getTX("limelight"));
        SmartDashboard.putNumber("getTY", LimelightHelpers.getTY("limelight"));
        SmartDashboard.putNumber("getLatency_Pipeline", LimelightHelpers.getLatency_Pipeline(""));

        SmartDashboard.putBoolean("get TV", LimelightHelpers.getTV(""));
        SmartDashboard.putNumber("getFiducialID", LimelightHelpers.getFiducialID(""));
        SmartDashboard.putNumber("get Target Area", LimelightHelpers.getTA(""));

        SmartDashboard.putNumber("getBotPose2d X", LimelightHelpers.getBotPose2d("").getX());
        SmartDashboard.putNumber("getBotPose2d Y", LimelightHelpers.getBotPose2d("").getY());

      /*   SmartDashboard.putNumber("Target Distance", -(getCameraTransform(2)));
        SmartDashboard.putNumber("Target Lateral", getCameraTransform(0));
        SmartDashboard.putNumber("Target Height", getCameraTransform(1));
        SmartDashboard.putNumber("Target Rotation", getCameraTransform(4));*/

  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0205; //.02

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveConstants.MAX_ROTATION_SPEED;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = -.005;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= SwerveConstants.MAX_SPEED;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

 /*  private void drive(boolean fieldRelative, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier Abutton) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(leftX.getAsDouble(), 0.02))
            * SwerveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(leftY.getAsDouble(), 0.02))
            * SwerveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(rightX.getAsDouble(), 0.02))
            * SwerveConstants.MAX_ROTATION_SPEED;
  }*/

}

