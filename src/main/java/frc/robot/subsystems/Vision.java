/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private double tv, tx, ta, ty, tid, fieldX, fieldY, fieldZ, fieldRoll, fieldPitch, fieldYaw;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry m_led_entry;
  private final DoubleArraySubscriber botposeSigh;
  private double[] botpose;
  private int idIndex = 0;

  /**
   * Creates a new Vision.
   */
  public Vision() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    m_led_entry = m_limelightTable.getEntry("ledMode");
    botposeSigh = m_limelightTable.getDoubleArrayTopic("botpose").subscribe(new double[] {});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = m_limelightTable.getEntry("tv").getDouble(0);
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0);
    ta = m_limelightTable.getEntry("ta").getDouble(0);
    tid = m_limelightTable.getEntry("tid").getDouble(0);
    botpose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    //x, y, z, roll, pitch, yaw


    if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }
    
    m_targetList.add(ta);

    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    LimelightHelpers.LimelightTarget_Fiducial[] llArr = llresults.targets_Fiducials;

    if(llArr.length > 0) {
    fieldX = -llArr[0].getRobotPose_FieldSpace().getX();
    fieldY = llArr[0].getRobotPose_FieldSpace().getY();
    fieldZ = llArr[0].getRobotPose_FieldSpace().getZ();
    fieldRoll = llArr[0].getRobotPose_FieldSpace().getRotation().getX();
    fieldPitch = llArr[0].getRobotPose_FieldSpace().getRotation().getY();
    fieldYaw = llArr[0].getRobotPose_FieldSpace().getRotation().getZ();
    }

    if(llArr.length > 0) {
        SmartDashboard.putNumber("Current ID", tid);
        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putNumber("X", fieldX);
        SmartDashboard.putNumber("Y", fieldY);
        SmartDashboard.putNumber("Z", fieldZ);
        SmartDashboard.putNumber("Roll", fieldRoll);
        SmartDashboard.putNumber("Pitch", fieldPitch);
        SmartDashboard.putNumber("Yaw", fieldYaw);
    }

    else {
        SmartDashboard.putNumber("Current ID", -1);
        SmartDashboard.putNumber("Distance", -1);
        SmartDashboard.putNumber("X", -1);
        SmartDashboard.putNumber("Y", -1);
        SmartDashboard.putNumber("Z", -1);
        SmartDashboard.putNumber("Roll", -1);
        SmartDashboard.putNumber("Pitch", -1);
        SmartDashboard.putNumber("Yaw", -1);
    }
  }

  public double getFieldX() {
    return fieldX;
  }

  public double getFieldY() {
    return fieldY;
  }

  public double getFieldZ() {
    return fieldZ;
  }

  public double getFieldRoll() {
    return fieldRoll;
  }

  public double getFieldPitch() {
    return fieldPitch;
  }

  public double getFieldYaw() {
    return fieldYaw;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    double sum = 0;

    for (Double num : m_targetList) {
      sum += num.doubleValue();
    }
    return sum/m_targetList.size();
  }

  public double getDistance() {
    //return Constants.OperatorConstants.yDisplace * Math.tan((90 - Constants.OperatorConstants.llAngle - getTY()) * Math.PI/180) - 2;
    return Math.sqrt(Math.pow(fieldX-Units.inchesToMeters(Constants.OperatorConstants.aprilTagX[(int)(getTID() - 1)]), 2) + Math.pow(fieldY-Units.inchesToMeters(Constants.OperatorConstants.aprilTagY[(int)(getTID() - 1)]), 2));
  }

  public double getTID() {
    return tid;
  }

  public double getIDIndex(){
    return idIndex;
  }

  public boolean isTargetValid() {
    return (tv == 1.0); 
  }

  public void add(){
    idIndex++;
    idIndex %= Constants.OperatorConstants.idsLength;
  }

}