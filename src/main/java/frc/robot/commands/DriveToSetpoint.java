// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToSetpoint extends CommandBase {

private PIDController m_DrivController;
  private final DriveSubsystem m_DriveSubsystem;
private double setpoint;

  /** Creates a new DriveToSetpoint. */
  public DriveToSetpoint(DriveSubsystem m_DriveSubsystem, double m_setpoint) {

    //SmartDashboard.setDefaultNumber("DriveKP", Constants.DrivePID.DrivekP);
   // SmartDashboard.setDefaultNumber("DriveKI", Constants.DrivePID.DrivekI);
    
    
        // The controller that the command will use
this.m_DriveSubsystem = m_DriveSubsystem;

this.setpoint = m_setpoint;








        addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  @Override
  public void initialize() {
    m_DrivController = new PIDController(SmartDashboard.getNumber("DriveKP",Constants.DrivePID.DrivekP), SmartDashboard.getNumber("DriveKI", Constants.DrivePID.DrivekI), Constants.DrivePID.DrivekD);
    m_DrivController.setTolerance(Constants.DrivePID.tolerance);
  }


  @Override
  public void execute() 
  {
    double feedforward = Constants.DrivePID.DriveFF;
    double speed = m_DrivController.calculate(m_DriveSubsystem.getLeftEncoder(),setpoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    speed = (speed > 1 ) ? 1.0 : speed;
    speed = (speed < -1 ) ? -1 : speed; 

    m_DriveSubsystem.TankDrive(-speed*Constants.MaxDriveSpeed,-speed*Constants.MaxDriveSpeed);
   
    SmartDashboard.putNumber("pidspeed", speed);
  
  }

    @Override
  public void end(boolean interrupted) 
  {
    m_DriveSubsystem.TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(m_DrivController.atSetpoint())
    //   return true;
    
    return false;
  }
}
