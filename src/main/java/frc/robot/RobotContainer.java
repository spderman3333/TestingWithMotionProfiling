// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystem.Elevator;

public class RobotContainer {

  private final Elevator elevator;

  private final CommandPS4Controller mainController;

  public RobotContainer() {

    elevator = new Elevator(NetworkTableInstance.getDefault());

    mainController = new CommandPS4Controller(0);

    configureBindings();
  }

  private void configureBindings() {
    System.out.println("Bindings Configured.");
    mainController.povUp().onTrue(elevator.setMotorPosition(Elevator.ElevatorMotorPosition.TOP));
    mainController.povDown().onTrue(elevator.setMotorPosition(Elevator.ElevatorMotorPosition.BASE));

  }

  public Command getAutonomousCommand() {
    return elevator.getSysIdRoutine();
  }
}
