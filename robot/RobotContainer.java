// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.lang.Object;

// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final Timer m_timer = new Timer();

  private XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController); 
  private XboxController m_armController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings(); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //set up the drivetrain command that runs all the time
    m_drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        m_drivetrain.tankDrive(
          MathUtil.applyDeadband(m_driveController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(m_driveController.getRightY(), Constants.OIConstants.kDriveDeadband),
          m_driveController)
      , m_drivetrain)
    );

    //set up gripper open/close
    new JoystickButton(m_armController, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> m_gripper.openGripper()))
      .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));

    //set up arm preset positions
    new JoystickButton(m_armController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
    new JoystickButton(m_armController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition, m_gripper)));
    new JoystickButton(m_armController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition, m_gripper)));
    new JoystickButton(m_armController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition, m_gripper)));

    //set up arm manual and auto functions
    m_arm.setDefaultCommand(new RunCommand(
      () ->
        m_arm.runAutomatic()
      , m_arm)
    );
    new Trigger(() -> 
      Math.abs(m_armController.getRightTriggerAxis() - m_armController.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
          m_arm.runManual((m_armController.getRightTriggerAxis() - m_armController.getLeftTriggerAxis()) * Constants.OIConstants.kArmManualScale)
        , m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_timer.reset();
    m_timer.start();

    while (m_timer.get() < 0.05){
      m_drivetrain.tankDrive(-1, -1, m_driveController);
    }
    while (m_timer.get() < 0.1 & m_timer.get() > 0.05){
    }
    while (m_timer.get() < 2.5 & m_timer.get() > 0.1){
    m_drivetrain.tankDrive(0.3, 0.3, m_driveController);
    }
    while (m_timer.get() < 2.6 & m_timer.get() > 2.5){
    }
    // while (m_timer.get() < 2.55 & m_timer.get() > 1.7){
    // m_drivetrain.tankDrive(0.25, -0.25, m_driveController);
    // }

    return null;
  }
}