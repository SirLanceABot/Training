package frc.robot.commands;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.FanFSMSubsystem;

public class FanFSM {

    static
    {
        System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
    }
    
    // information needed from the calling subsystem
    FanFSMSubsystem subsystemRequirement;
    XboxController driverController;

    // States are implemented by Commands
    Command offState ;
    Command highState;
    Command medState;
    Command lowState;

    Command initialState;
    Command currentState;
    
    // the triggers (events) to change states
    List<Trigger> triggers = new ArrayList<Trigger>();
    public NextStateTrigger nst;
    public OffStateTrigger ost;

    // Store Transitions defined for this FSM
    static List<Transition> transitions = new ArrayList<Transition>(10); // specify size as number of transitions or more
/**
 * 
 * @param subsystem caller (parent) is the required subsystem
 * @param driverController game controller used for the FSM
 */
    public FanFSM(FanFSMSubsystem subsystem, XboxController driverController)
    {
      // info from calling subsystem
        subsystemRequirement = subsystem;
        this.driverController = driverController;

      // instantiate states (Commands)
        offState = new OffState();
        highState = new HighState();
        medState = new MedState();
        lowState = new LowState();

        initialState = offState;
        currentState = initialState;

      // instantiate events (Triggers)
        nst = new NextStateTrigger();
        ost = new OffStateTrigger();
    
      // instantiate the Transition Table
        // set ArrayList size greater or equal to number of transitions

        // The FSM's Transition Table
        // Add Current State (Command), Event (Trigger), Next State (Next Command)
        transitions.add(new Transition(offState,    (Trigger)nst,   highState));
        transitions.add(new Transition(highState,   (Trigger)nst,   medState));
        transitions.add(new Transition(medState,    (Trigger)nst,   lowState));
        transitions.add(new Transition(lowState,    (Trigger)nst,   offState));

        transitions.add(new Transition(highState,   (Trigger)ost,   offState));
        transitions.add(new Transition(medState,    (Trigger)ost,   offState));
        transitions.add(new Transition(lowState,    (Trigger)ost,   offState));

        // transitions.forEach((transition)-> System.out.println(transition.toString() ) );

      // schedule the Command for the initial state but doesn't matter because robot is disabled
      // and this doesn't do anything since Triggers and Commands are all for ENABLED teleop only
        // initialState.schedule(); // start at initialState
    }

//STATES
//STATES
//STATES

// These states all have an execute() to refresh the state as needed but no
// initialize() nor end() nor isFinished().
// initialize() and end() would be reasonable if the device (FSM) logic needs them.
// isFinished() maybe useful in some circumstances but can't think of any except hard
// until a trigger revives the FSM.
// The states all run until interrupted by the scheduling of a new state

//___________________________________________________________________________________________________________

    // OffState
    public class OffState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public OffState() { addRequirements(subsystemRequirement); }

      @Override
      public void execute() { System.out.println("Off"); } // refresh state as needed
    }    // end OffState

//___________________________________________________________________________________________________________

    public class HighState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public HighState() { addRequirements(subsystemRequirement); }

      @Override
      public void execute() { System.out.println("High"); } // refresh state as needed
    }    // end HighState

//___________________________________________________________________________________________________________

    // MedState
    public class MedState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public MedState() { addRequirements(subsystemRequirement); }

      @Override
      public void execute() { System.out.println("Med"); } // refresh state as needed
    }    // end MedState

//___________________________________________________________________________________________________________

    // LowState
    public class LowState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public LowState() { addRequirements(subsystemRequirement); }

      @Override
      public void execute() { System.out.println("Low"); } // refresh state as needed
    }   // end LowState

//___________________________________________________________________________________________________________
//end STATES
//end STATES
//end STATES

//TRANSITIONS
//TRANSITIONS
//TRANSITIONS
  /**
   * Transitions of the FSM
   * 
   * each Transition is composed of 2 independent variables - current state and event - 
   * and a dependent variable - new state
   */
  private class Transition
  {
    private Command currentState;
    private final Trigger event;
    private final Command nextState;
    
    /**
     * Add a transition to the FSM transition table
     * @param currentState - independent variable
     * @param event - independent variable
     * @param nextState - dependent variable
     */
    Transition(Command currentState, Trigger event, Command nextState)
    {
      this.currentState = currentState;
      this.event = event;
      this.nextState = nextState;
    }
  }

  public void checkStateChange()
  {
    // System.out.println("check state change");

    // make the transition to a new currentState if an event triggered it - first found active trigger wins
    
    // loop through all triggers assumed to be added in priority order
    // find first active trigger then loop through all states

    Command newFanState = null;

    for(Trigger trigger:triggers) // loop through all the triggers
    {
      if(trigger.get()) // check if this trigger was triggered this iteration
      {
        newFanState = findNextState (currentState, trigger); // see if a transition to the next state
        if(newFanState != null) // found in transition table; use it and stop looking for more; first has won
        {
          System.out.println(newFanState.getClass().getName());
          currentState = newFanState; // transition to next state even if it's the same
          currentState.schedule(); // new state interrupts previous state
          // System.out.println("new state scheduled " + currentState.isScheduled());
          break;
        }
      }
    }
  }

  /**
   * table lookup to determine new state given the current state and the event
   * if state transition is not in the transition table then return null.
   * 
   * @param currentState
   * @param event
   * @return
   */
  private static Command findNextState (Command currentState, Trigger event)
  {
    // System.out.print(currentState + " " + event);
    for (Transition transition : transitions)
    {
      if (transition.currentState == currentState && transition.event == event)
      {
        // System.out.println(" transition change to " + transition.nextState.toString());
        return transition.nextState;
      }
    }
    // System.out.println(" no transition");
    return null;
  }
//end TRANSITIONS
//end TRANSITIONS
//end TRANSITIONS

//TRIGGERS
//TRIGGERS
//TRIGGERS
  public class NextStateTrigger extends Trigger
  {
    NextStateTrigger()
    {
      triggers.add(this); // add to the list of triggers
    }

    // This returns whether the trigger is active
    @Override
    public boolean get()
    {
      if(DriverStation.isTeleopEnabled())
        return subsystemRequirement.mPeriodicIO.AButtonPressed;
      else
        return false; // ignore events (triggers) unless
    }
  }

  public class OffStateTrigger extends Trigger
  {
    OffStateTrigger()
    {
      triggers.add(this); // add to the list of triggers
    }

    // This returns whether the trigger is active
    @Override
    public boolean get()
    {
      if(DriverStation.isTeleopEnabled())
        return subsystemRequirement.mPeriodicIO.BButtonPressed;
      else
        return false; // ignore events (triggers) unless 
    }
}
//end TRIGGERS
//end TRIGGERS
//end TRIGGERS
}
