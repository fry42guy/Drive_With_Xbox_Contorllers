// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class DriveSubsystem extends SubsystemBase {

public final WPI_TalonFX m_FrontRight;
public final WPI_TalonFX m_RearRight;
public final WPI_TalonFX m_FrontLeft;
public final WPI_TalonFX m_RearLeft;

public final MotorControllerGroup m_LeftDrive;
public final MotorControllerGroup m_RightDrive;

public final DifferentialDrive m_TankDrive;


/** Creates a new DriveSubsytem. */
  public DriveSubsystem() {
m_FrontRight = new WPI_TalonFX(Constants.Front_Right_Drive);
m_RearRight = new WPI_TalonFX(Constants.Rear_Right_Drive);
m_FrontLeft = new WPI_TalonFX(Constants.Front_Left_Drive);
m_RearLeft = new WPI_TalonFX(Constants.Rear_Left_Drive);

m_RightDrive = new MotorControllerGroup(m_FrontRight, m_RearRight);
m_LeftDrive = new MotorControllerGroup(m_FrontLeft, m_RearLeft);
m_LeftDrive.setInverted(true);

m_TankDrive = new DifferentialDrive(m_LeftDrive,m_RightDrive);

m_TankDrive.setSafetyEnabled(false);

  }

  public void Drive(double leftspeed, double rotatespeed){

    m_TankDrive.arcadeDrive(leftspeed, rotatespeed);
    System.out.println(leftspeed);

  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
