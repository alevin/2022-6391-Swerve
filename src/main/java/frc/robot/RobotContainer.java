// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(0);

  public RobotContainer() {
    m_drivetrainSubsystem.zeroGyroscope();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * 1, // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * 1, // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // () -> -modifyAxis(m_controller.getRightX()) *
        // 2//DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        () -> modifyAxis(m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis()) * 3));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton)
    // // No requirements because we don't need to interrupt anything
    // .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    JoystickButton buttonA = new JoystickButton(m_controller, XboxController.Button.kA.value);
    JoystickButton buttonB = new JoystickButton(m_controller, XboxController.Button.kB.value);
    JoystickButton buttonX = new JoystickButton(m_controller, XboxController.Button.kX.value);
    JoystickButton buttonY = new JoystickButton(m_controller, XboxController.Button.kY.value);
    JoystickButton start = new JoystickButton(m_controller, XboxController.Button.kStart.value);

    start.whenPressed(m_drivetrainSubsystem::toggleDriveMode);
    buttonY.whenPressed(() -> m_drivetrainSubsystem.setWheelAngle(0));
    buttonA.whenPressed(() -> m_drivetrainSubsystem.changeWheelAngleBy45());
    buttonX.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Straight", 0.01, 0.01);
    
    buttonB.whenPressed(PPSwerveCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2); // 0.05, 0.1 seems to work

    return value;
  }

  public void testPath() {
    // Since a PathPlannerTrajectory extends the WPILib Trajectory, it can be
    // referenced as one
    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 8 m/s and a max acceleration of 5 m/s^2

    Trajectory examplePath = PathPlanner.loadPath("Straight", 0.01, 0.01);

    // This trajectory can then be passed to a path follower such as a
    // RamseteCommand
    // Or the path can be sampled at a given point in time for custom path following

    // Sample the state of the path at 1.2 seconds
    Trajectory.State exampleState = examplePath.sample(1.2);

    // Print the velocity at the sampled time
    System.out.println("HEY LOOK AT ME" + exampleState.velocityMetersPerSecond);

  }

  public PPSwerveControllerCommand PPSwerveCommand() {
  //public void PPSwerveCommand() {
    System.out.println("IN PPSWERVECOMMAND");
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Turn", 1, 2);
    System.out.println("TOTAL TIME SECONDS " + examplePath.getTotalTimeSeconds());
    System.out.println("exampleaPath getInitialState: " + examplePath.getInitialState());
    System.out.println("examplePath getEndState: " + examplePath.getEndState());
    System.out.println("examplePath getState 0: " + examplePath.getState(0));
    System.out.println("examplePath getState 1: " + examplePath.getState((1)));
    System.out.println("examplePath getState 2: " +  examplePath.getState(2));
    System.out.println("examplePath getState 3: " +  examplePath.getState(3));
    System.out.println("examplePath getState 4: " +  examplePath.getState(4));

    System.out.println("WHERE ARE YOU ERIKA: " + examplePath.sample(0.5).velocityMetersPerSecond);
    System.out.println("acceleration: " + examplePath.sample(0.5).accelerationMetersPerSecondSq);
    System.out.println("WHERE ARE YOU ERIKA 2: " + examplePath.sample(1).velocityMetersPerSecond);
    System.out.println("acceleration: " + examplePath.sample(1).accelerationMetersPerSecondSq);
    System.out.println("WHERE ARE YOU ERIKA 3: " + examplePath.sample(1.5).velocityMetersPerSecond);
    System.out.println("acceleration: " + examplePath.sample(1.5).accelerationMetersPerSecondSq);
    System.out.println("WHERE ARE YOU ERIKA 4: " + examplePath.sample(2).velocityMetersPerSecond);
    System.out.println("acceleration: " + examplePath.sample(2).accelerationMetersPerSecondSq);
    

    
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        examplePath,
        m_drivetrainSubsystem::getPose,
        m_drivetrainSubsystem.getKinematics(),
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        m_drivetrainSubsystem.getThetaController(),
        m_drivetrainSubsystem::setStates,
        m_drivetrainSubsystem);

    
    return command;
  }

}
