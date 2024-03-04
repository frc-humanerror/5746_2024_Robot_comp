// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.WristIntake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ReverseWristAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristIntake m_WristIntake;
  public boolean m_intakeIn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseWristAutoCommand(WristIntake wristintake, boolean intakeIn) {
    m_WristIntake = wristintake;
    m_intakeIn = intakeIn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_WristIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeIn == true) {
      m_WristIntake.wrist(true, false, false, true);
    }
    else if(m_intakeIn == false) {
      m_WristIntake.wrist(false, true, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_WristIntake.reversed == true) {
      return true;
    }
    else {
      return false;
    }
  }
}