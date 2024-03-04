// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomi;

import frc.robot.commands.DriveAutoCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.LaunchAutoCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class OneNote_noMove extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain m_drivetrain;
  private Launcher m_launcher;
  private Intake m_intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OneNote_noMove(Launcher launcher, Intake intake) {
    m_launcher = launcher;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
addCommands(
    new ParallelCommandGroup(
      new LaunchAutoCommand(launcher, 1),
      new IntakeAutoCommand(intake, 1, .5, false)
    )
);
  }
}