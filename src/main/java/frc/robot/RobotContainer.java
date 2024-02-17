// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PathPlannerAutoCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
  private final NavX navx = new NavX();
  private final DriveSubsystem drive = new DriveSubsystem(navx);
  private final OI oi = new OI();

  private SendableChooser<Command> autos;

  public RobotContainer() {

    // NamedCommands.registerCommand("PathPlannerAuto", new PathPlannerAutoCommand(drive, oi));

    autos = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autons", autos);

    drive.resetOdometry(new Pose2d());

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new PathPlannerAutoCommand(drive, oi));
  }

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
