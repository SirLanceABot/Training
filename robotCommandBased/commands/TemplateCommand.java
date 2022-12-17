/**
 * A state machine representing a complete action to be performed by the robot. Commands are run by
 * the {@link CommandScheduler}, and can be composed into CommandGroups to allow users to build
 * complicated multi-step actions without the need to roll the state machine logic themselves.
 *
 * <p>Commands are run synchronously from the main robot loop; no multithreading is used, unless
 * specified explicitly from the command implementation.
 *
 */

package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TemplateCommand extends CommandBase {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  private boolean runsWhenDisabled;

  /** The class constructor. */
  protected TemplateCommand(boolean runsWhenDisabled)
    {
        this.runsWhenDisabled = runsWhenDisabled;
    }
 
  /** The initial subroutine of a command.
   *  Called once when the command is initially scheduled.
   *  The scheduler does not have to run */
  public void initialize() {}

  /** The main body of a command. Called repeatedly
   *  while the command is scheduled and scheduler runs. */
  public void execute() {}

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   * The scheduler does not have to be run.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {}

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
    return false;
  }
  
  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   *
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

}
