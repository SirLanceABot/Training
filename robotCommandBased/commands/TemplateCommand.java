/*
Generally use Commands for any operations that it supports.
They are the shortcut versions and all return a CommandBase.

Use CommandBase to extend our custom command classes.

Commands are represented in the command-based library by the Command interface

To write a custom command class, subclass the abstract CommandBase classTo write
 a custom command class, subclass the abstract CommandBase class

Inheriting from CommandBase rather than Command provides several convenience features.
 It automatically overrides the getRequirements() method for users, returning a list of requirements that is empty by default, but can be added to with the addRequirements() method. It also implements the Sendable interface, and so can be sent to the dashboard - this provides a handy way for scheduling commands for testing (via a button on the dashboard) without needing to bind them to buttons on a controller.

Important
After calling a decorator or being passed to a composition, the command object cannot
 be reused! Use only the command object returned from the decorator.
-----------------

Control Algorithm Commands
There are commands for various control setups:

PIDCommand uses a PID controller. For more info, see PIDCommand.

TrapezoidProfileCommand tracks a trapezoid motion profile. For more info, see
 TrapezoidProfileCommand.

ProfiledPIDCommand combines PID control with trapezoid motion profiles. For more
 info, see ProfiledPIDCommand.

MecanumControllerCommand (Java, C++) is useful for controlling mecanum drivetrains.
 See API docs and the MecanumControllerCommand (Java, C++) example project for more info.

SwerveControllerCommand (Java, C++) is useful for controlling swerve drivetrains. See
 API docs and the SwerveControllerCommand (Java, C++) example project for more info.

RamseteCommand (Java, C++) is useful for path following with differential drivetrains
 (“tank drive”). See API docs and the Trajectory Tutorial for more info.


-------------
I think I can simplify the best practice that the WPILib authors recommend:
-------------
Generally use Commands from a specific static import for any operations that it supports.
They are the shortcut versions and all return a CommandBase.

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.run; // allows requirements
import static edu.wpi.first.wpilibj2.command.Commands.runEnd; // allows requirements
import static edu.wpi.first.wpilibj2.command.Commands.runOnce; // allows requirements
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd; // allows requirements
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
It is not recommended to use
import static edu.wpi.first.wpilibj2.command.Commands.*;
or
import edu.wpi.first.wpilibj2.command.Commands;
although both of these are much easier to use than the specific static import.
------------
Use extend CommandBase for all our custom command classes.
------------
Generally don't use Command
------------
As a rule, command compositions require all subsystems their components require,
may run when disabled if all their component set runsWhenDisabled as true, and
are kCancelIncoming if all their components are kCancelIncoming as well.
-----------
Be aware of the difference between a command that runs nearly instantly and once
versus its side effects that may continue.
A command to turn on a motor my run only for an instant but the motor
continues to run because nothing turned it off.

Usually best practice for a motor is to refresh the speed command often -
even every cycle and that type of command would run indefinitely 

*/

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
