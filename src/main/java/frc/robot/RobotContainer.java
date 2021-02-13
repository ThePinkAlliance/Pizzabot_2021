// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PizzaPath;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ResetEncoders;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain m_DriveTrain = new DriveTrain();

  XboxController m_driverController = new XboxController(Constants.JOYSTICK_ID_DRIVER);

  private final PizzaDash pizzaDash = new PizzaDash();
  private final PizzaPath pizzaPath = new PizzaPath();
  private final Command m_autoCommand = pizzaPath.getAutonomousTrajectoryTest(m_DriveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_DriveTrain.setDefaultCommand(new DefaultDrive(m_DriveTrain, 
                                                    ()->m_driverController.getY(GenericHID.Hand.kLeft),
                                                    ()->m_driverController.getY(GenericHID.Hand.kRight)));
    
  }

  public void updateDashboard() {
    if (pizzaDash != null) {
      pizzaDash.putEncoder(m_DriveTrain);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value).whenPressed(new ResetEncoders(m_DriveTrain));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
