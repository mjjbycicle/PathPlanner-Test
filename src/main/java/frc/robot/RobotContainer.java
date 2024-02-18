// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
  private final NavX navx = new NavX();
  private final DriveSubsystem drive = new DriveSubsystem(navx);
  private final OI oi = new OI();

  private SendableChooser<Command> autos;

  public RobotContainer() {
    //autos are in deploy/pathplanner/autos
    autos = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autons", autos);

    drive.resetOdometry(new Pose2d());

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new ManualSwerveDriveCommand(drive, oi));
  }

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
