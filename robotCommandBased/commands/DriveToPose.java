package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends CommandBase {
      double startEncoder;
      double stopEncoder;

      double startAngle;
      double stopAngle;

      double startRotation;
      double stopRotation;

      DriveSubsystem driveSubsystem;
  
    public DriveToPose(DriveSubsystem driveSubsystem, double angle, double distance, double rotation) 
    {
        this.driveSubsystem = driveSubsystem;
        
        startEncoder = this.driveSubsystem.periodicIO.encoder;
        stopEncoder = startEncoder + distance;

        // startAngle = this.driveSubsystem.mPeriodicIO.angle;
        stopAngle = startAngle + angle;

        


        System.out.println("drive forward");
  
        addRequirements(this.driveSubsystem); //all subsystems that might be used in any command in the group

        // setTestMotorPctVBus.accept(speed);
    }  
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}
