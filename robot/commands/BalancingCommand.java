package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class BalancingCommand extends CommandBase{
    
static final double kOffBalanceAngleThresholdDegrees = 10;
static final double kOonBalanceAngleThresholdDegrees  = 2.5;

    @Override
    public void execute() {
System.out.println("Working");
    }

}