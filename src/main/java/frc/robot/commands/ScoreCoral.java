// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private boolean previousBeamBroken = false;
  /** Creates a new ScoreCoral. */
  public ScoreCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousBeamBroken = RobotContainer.s_Claw.hasCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_Claw.setSpeed(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (previousBeamBroken) {
      return false;
    }
    return !RobotContainer.s_Claw.hasCoral();
  }
}
