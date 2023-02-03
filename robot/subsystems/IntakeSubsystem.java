package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController; //Xbox controller, use this one
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubsystem {

    public void armControl(MotorController Motor, XboxController Controll, DigitalInput Switch) {
        // When left or right triggers are pressed move the arm out and in
        if(Switch.get() == false){
          Motor.setVoltage(0);//Does not move
        }
        else if (Controll.getLeftTriggerAxis() > 0) {//Checks for left trigger input
          Motor.setVoltage(Controll.getLeftTriggerAxis()*-1);//Negative so it turns Left
        }
        else if (Controll.getRightTriggerAxis() > 0){//Checks for right trigger input
          Motor.setVoltage(Controll.getRightTriggerAxis()*1);//Positive so it turns Right
        }
        else {//Idle if no trigger input
          Motor.setVoltage(0);//Does not move
        }
    }
    public void handControl(MotorController motor, XboxController controll) {
        // When left or right bumpers are pressed move the arm out and in
        if (controll.getLeftBumper()) {//Checks for left trigger input
            motor.setVoltage(-1);//Negative so it turns Left
          }
          else if (controll.getRightBumper()){//Checks for right trigger input
            motor.setVoltage(1);//Positive so it turns Right
          }
          else {//Idle if no trigger input
            motor.setVoltage(0);//Does not move
          }
    }
}
