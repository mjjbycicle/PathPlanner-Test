package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class PathPlannerAutoCommand extends Command{
    private final DriveSubsystem driveSubsystem;
    private final OI oi;

    public PathPlannerAutoCommand(DriveSubsystem driveSubsystem, OI oi){
        this.driveSubsystem = driveSubsystem;
        this.oi = oi;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        driveSubsystem.stop();
    }

    @Override
    public void execute(){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            oi.getDriverController().getAxis(OI.Axes.LEFT_STICK_Y),
            oi.getDriverController().getAxis(OI.Axes.LEFT_STICK_X),
            oi.getDriverController().getAxis(OI.Axes.RIGHT_STICK_X)
        );

        driveSubsystem.setMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
