// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_Intake;
  private double m_intakeDuration;
  private boolean m_pullIn;
  private double m_delay;
  private Timer timer = new Timer();
  private double time;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeAutoCommand(Intake intake, double intakeDuration, double delay, boolean pullIn) {
    m_Intake = intake;
    m_intakeDuration = intakeDuration;
    m_delay = delay;
    m_pullIn = pullIn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("Intake Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      time = timer.get();
    if(m_pullIn == true && time >= m_delay) {
    m_Intake.pull(true, false, false);
    }
    else if(m_pullIn == false && time >= m_delay) {
      m_Intake.pull(false, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(time);
    System.out.println(m_intakeDuration);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time > m_intakeDuration) {
      return true;
    }
    else{
    return false;
    }
  }
}