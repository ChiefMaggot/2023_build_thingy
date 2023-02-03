package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController; //Xbox controller, use this one
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;


public class BalanceSubsystem {
private final CANSparkMax m_leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
private final CANSparkMax m_rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
private final CANSparkMax m_leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);
private final CANSparkMax m_rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);

MotorControllerGroup m_motorsLeft = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
MotorControllerGroup m_motorsRight = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
private DifferentialDrive m_myRobot = new DifferentialDrive(m_motorsLeft, m_motorsRight);

  private AHRS m_Ahrs;
  
  
    public void drive(XboxController controller){
      if ((Math.abs(controller.getLeftY()) > 0.05) || (Math.abs(controller.getRightY()) > 0.05)){
      m_myRobot.tankDrive(controller.getLeftY()*0.6, controller.getRightY()*0.6);//Grabs inputs for each motor from "myRobot", starts with the defined first one.
      }
      else{
        m_myRobot.tankDrive(0, 0);//Grabs inputs for each motor from "myRobot", starts with the defined first one.
      }
    }
}
