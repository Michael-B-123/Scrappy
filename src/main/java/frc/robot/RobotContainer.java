// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
    public CommandXboxController driveController = new CommandXboxController(0);
    CommandXboxController coDriveController = new CommandXboxController(1);
    CommandXboxController ohShitController = new CommandXboxController(2);

    private final Trigger driveRightTrigger = driveController.rightTrigger(0.5);
    private final Trigger drivekLeftBumper = driveController.leftBumper();
    private final Trigger drivekRightBumper = driveController.rightBumper();

    public final Drivetrain drivetrain = new Drivetrain(Constants.Modules.moduleArray, driveController, Constants.Drivetrain.CameraName);
    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Nothing");
        SmartDashboard.putData("Auto chooser", autoChooser);

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}