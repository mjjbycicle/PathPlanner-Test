package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ManualSwerveDriveCommand extends Command{
    private final DriveSubsystem driveSubsystem;
    private final OI oi;

    public ManualSwerveDriveCommand(DriveSubsystem driveSubsystem, OI oi){
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
            oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveForwardAxis),
            -oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveStrafeAxis),
            oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveRotationAxis)
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
