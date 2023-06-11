// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TelopDrive extends CommandBase {

  public final DriveSubsystem m_DriveSubsystem;
  DoubleSupplier xspeed;
  DoubleSupplier rotatespeed; 

  /** Creates a new TelopDrive. */
  public TelopDrive(DriveSubsystem m_DriveSubsystem, DoubleSupplier xspeed, DoubleSupplier rotatespeed) {
this.m_DriveSubsystem = m_DriveSubsystem;
this.xspeed = xspeed;
this.rotatespeed = rotatespeed;
    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_DriveSubsystem.ArcadeDrive(xspeed.getAsDouble() *Constants.MaxDriveSpeed, rotatespeed.getAsDouble()*Constants.MaxRoationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
