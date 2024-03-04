// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_Drivetrain;
  public static final ADIS16470_IMU gyro1 = new ADIS16470_IMU();
  private double m_angleToTurn;
  private double error;
  private double m_kP;
  private double currentAngle;
  public static double getAngle() {
    return gyro1.getAngle(gyro1.getYawAxis());
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnAutoCommand(Drivetrain drivetrain, double angleToTurn, double kP) {
    m_Drivetrain = drivetrain;
    m_angleToTurn = angleToTurn;
    m_kP = kP;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro1.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = gyro1.getAngle(gyro1.getYawAxis());
    error = m_angleToTurn - currentAngle;
    m_Drivetrain.move(m_kP*error, -m_kP*error);
    SmartDashboard.putNumber("Turnerorr", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(error < 3 && error > -3) {
      return true;
    }
    else {
    return false;
    }
  }
}