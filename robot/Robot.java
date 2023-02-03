/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

//import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController; //Grabs things for Xbox controller, use this one if using Xbox controller
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.BalancingCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BalanceSubsystem;
//import com.revrobotics.SparkMaxLimitSwitch;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private IntakeSubsystem m_intake;//Defines intake
  private BalanceSubsystem m_drive;
  private XboxController m_controller;//Defines controller
  private static final int armID = 6;
  private static final int handID = 7;
  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightBackMotor;
  private CANSparkMax m_armMotor;
  private CANSparkMax m_handMotor;
  private AHRS m_Ahrs;
  private BalancingCommand m_balancing;
  
  DigitalInput m_switch;

  private double startTime;

  @Override
  public void robotInit() {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_Ahrs = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }
  /*
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
  
    m_armMotor = new CANSparkMax(armID, MotorType.kBrushless);
    m_handMotor = new CANSparkMax(handID, MotorType.kBrushless);

    MotorControllerGroup m_motorsLeft = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    MotorControllerGroup m_motorsRight = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
    m_switch = new DigitalInput(0);


    /*
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */

    m_myRobot = new DifferentialDrive(m_motorsLeft, m_motorsRight);//sets motor groups for "myRobot"
    m_intake = new IntakeSubsystem();//Need this to run subsystem
    m_balancing = new BalancingCommand();
    m_controller = new XboxController(0);//sets port for controller, used in driver station
    m_drive = new BalanceSubsystem();

  }

  @Override
  public void robotPeriodic(){

    CommandScheduler.getInstance().schedule(m_balancing);
  }

  //Auto
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

    if (time - startTime < 1) {//Durring auto
    }
    else if(time - startTime < 1.5) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 2) {//Durring auto
    }
    else if(time - startTime < 2.5) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 3) {//Durring auto
    }
    else if(time - startTime < 4) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 5) {//Durring auto
    }
    else if(time - startTime < 5.5) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 6) {//Durring auto
    }
    else if(time - startTime < 6.5) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 7) {//Durring auto
    }
    else if(time - startTime < 8) {//Durring auto
      m_rightFrontMotor.setVoltage(2);//Move motor
    }
    else if(time - startTime < 9) {//Durring auto
    }
  }
  
  //Driver controll
  @Override
  public void teleopPeriodic() {
  m_drive.drive(m_controller);
  m_intake.armControl(m_armMotor, m_controller, m_switch);
    //System.out.println(m_switch.get());
    
    m_intake.handControl(m_handMotor, m_controller);
if(m_controller.getAButton()){}
    //System.out.println("Hello");//Hi

  }
}