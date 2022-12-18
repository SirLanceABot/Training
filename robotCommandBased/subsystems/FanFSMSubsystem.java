/**
 * Finite State Machine for a three-speed ceiling.
 * The fan has a pull chain (Xbox button A) that switches off - high - medium - low - off.
 * The fan has an off switch (Xbox button B) that goes to off no matter the current speed.
 * The buttons only respond in teleop enabled
 */

package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FanFSMSubsystem  extends SubsystemTeam {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  public FanFSMSubsystem(XboxController driverController) // pass in all the stuff this class needs from above
    {  
        this.driverController = driverController;
        mPeriodicIO = new PeriodicIO(); // all the inputs appear here
        initializeFSM();
    }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run without regard to DISABLED or ENABLED
      checkStateChange();
      }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * define all the I/O to be read and written at once periodically
   */
  public PeriodicIO mPeriodicIO;
  private XboxController driverController;
  
  @Override
  public void readPeriodicInputs()
    {
      // populate each input variable
      mPeriodicIO.AButtonPressed = driverController.getAButtonPressed();
      mPeriodicIO.BButtonPressed = driverController.getBButtonPressed();
    }

    public void writePeriodicOutputs()
    {
      // act on (put out) data others have populated
      System.out.println(mPeriodicIO.speed);
    } 
    
  public class PeriodicIO {
  // INPUTS
    public boolean AButtonPressed;
    public boolean BButtonPressed;
  // OUTPUTS
    public double speed;
  }
  /**
   * end define I/O
   */

//////////////////////////////////////////////////////////////////////////////
//////////// SETUP FSM ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

    // States are implemented by Commands
    Command offState ;
    Command highState;
    Command medState;
    Command lowState;

    Command initialState;
    Command currentState;
    
    // the triggers (events) to change states
    static ArrayList<Trigger> triggers = new ArrayList<Trigger>(); // easier to loop if all are also in a list
    public NextStateTrigger nst;
    public OffStateTrigger ost;

    // Store Transitions defined for this FSM
    static ArrayList<Transition> transitions = new ArrayList<Transition>(10); // specify size as number of transitions or more

    private void initializeFSM()
    {
      // instantiate states (as Commands)
        offState = new OffState();
        highState = new HighState();
        medState = new MedState();
        lowState = new LowState();

        initialState = offState;
        currentState = initialState;

      // instantiate events (as Triggers)
        nst = new NextStateTrigger();
        ost = new OffStateTrigger();
    
      // The FSM's Transition Table
      // Add Current State (Command), Event (Trigger), Next State (Next Command)
        transitions.add(new Transition(offState,    (Trigger)nst,   highState));
        transitions.add(new Transition(highState,   (Trigger)nst,   medState));
        transitions.add(new Transition(medState,    (Trigger)nst,   lowState));
        transitions.add(new Transition(lowState,    (Trigger)nst,   offState));

        transitions.add(new Transition(highState,   (Trigger)ost,   offState));
        transitions.add(new Transition(medState,    (Trigger)ost,   offState));
        transitions.add(new Transition(lowState,    (Trigger)ost,   offState));

        System.out.println(dumpFSM());

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
// stop until a trigger revives the FSM.
// The states all run until interrupted by the scheduling of a new state.

//___________________________________________________________________________________________________________

    // OffState
    public class OffState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public OffState() { addRequirements(FanFSMSubsystem.this); }

      @Override
      public void execute() { mPeriodicIO.speed = 0.; } // refresh state as needed
    }    // end OffState

//___________________________________________________________________________________________________________

    public class HighState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public HighState() { addRequirements(FanFSMSubsystem.this); }

      @Override
      public void execute() { mPeriodicIO.speed = 1.; } // refresh state as needed
    }    // end HighState

//___________________________________________________________________________________________________________

    // MedState
    public class MedState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public MedState() { addRequirements(FanFSMSubsystem.this); }

      @Override
      public void execute() { mPeriodicIO.speed = 0.7; } // refresh state as needed
    }    // end MedState

//___________________________________________________________________________________________________________

    // LowState
    public class LowState extends CommandBase
    {
      // using default the command is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior happens
      public LowState() { addRequirements(FanFSMSubsystem.this); }

      @Override
      public void execute() { mPeriodicIO.speed = 0.3; } // refresh state as needed
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


  /**
   * Check if a transition to the next state was triggered this iteration
   * If there is a transition then schedule the new state
   */
  public void checkStateChange()
  {
    Command newFanState = null;
    newFanState = findNextState (currentState); // see if a transition was triggered to the next state
    if(newFanState != null) // found in transition table
    {
      System.out.println(newFanState.getClass().getName());
      currentState = newFanState; // transition to next state even if it's the same since it was in the transition table
      currentState.schedule(); // new state interrupts previous state, end old and init new methods are run if available
      // System.out.println("new state scheduled " + currentState.isScheduled());
    }
  }

  /**
   * Table lookup to determine new state triggered given the current state and an event.
   * If state transition is not in the transition table then return null.
   * 
   * @param currentState
   * @param event
   * @return new state or null if no transition was triggered
   */
  private static Command findNextState (Command currentState)
  {
    for (Transition transition : transitions) // loop through transition table
    {
      if (transition.currentState == currentState) // found an entry for the current state
      {
          if(transition.event.get()) // check if this entry was triggered this iteration
          {
            // System.out.println(" transition change to " + transition.nextState.toString());
            return transition.nextState; // return the new state
          }
        }
    }
    return null; // no new state triggered
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
        return FanFSMSubsystem.this.mPeriodicIO.AButtonPressed;
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
        return FanFSMSubsystem.this.mPeriodicIO.BButtonPressed;
      else
        return false; // ignore events (triggers) unless 
    }
}
//end TRIGGERS
//end TRIGGERS
//end TRIGGERS

  /**
   * print information about the FSM
   * @return String of information about the FSM
   */
  public String dumpFSM()
  {
    StringBuilder sb = new StringBuilder(500);
    sb.append("\nTransitions\nFrom State   +   Event   ->   Next State\n");
    for (Transition transition : transitions)
    {
      sb.append( String.format("%-12s + %-12s -> %-12s\n",
        transition.currentState.getClass().getSimpleName(),
        transition.event.getClass().getSimpleName(),
        transition.nextState.getClass().getSimpleName()));
    }
    sb.append("initial state: " + initialState.getClass().getSimpleName());
    sb.append(", current state: " + currentState.getClass().getSimpleName() + "\n");
    
    return sb.toString();
  }
}
