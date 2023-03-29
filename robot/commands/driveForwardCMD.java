package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class driveForwardCMD extends CommandBase{
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double distance;
    private double encoderSetpoint;
    private Timer Timer;

    public driveForwardCMD(DrivetrainSubsystem drivetrainSubsystem, double distance){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.distance = distance;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        drivetrainSubsystem.resetEncoders();
        encoderSetpoint = distance;
        Timer.reset();
        Timer.start();
    }


    @Override
    public void execute(){
        drivetrainSubsystem.DistancePID(encoderSetpoint);
        
    }


    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.tankDrive(0, 0, null);

    }


    @Override
    public boolean isFinished(){ //Fix this
        return drivetrainSubsystem.Done(encoderSetpoint);
    }

}
