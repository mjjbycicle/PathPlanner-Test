package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    private final OdometrySubsystem odometrySubsystem;

    private final NavX navx;

    private ChassisSpeeds chassisSpeeds;

    public DriveSubsystem(NavX navx) {
        moduleFrontLeft = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);

        this.odometrySubsystem = new OdometrySubsystem(this, navx);
        this.navx = navx;
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setRawMovement, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    public void setMovement(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navx.getRotation2d());
        setRawMovement(robotCentricSpeeds);
        chassisSpeeds = robotCentricSpeeds;
    }

    public void setRawMovement(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.RobotInfo.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModulesStates(
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        );
        this.chassisSpeeds = chassisSpeeds;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return chassisSpeeds;
    }

    private void setModulesStates(
            SwerveModuleState stateFrontLeft,
            SwerveModuleState stateFrontRight,
            SwerveModuleState stateBackLeft,
            SwerveModuleState stateBackRight
    ) {
        moduleFrontLeft.setState(stateFrontLeft);
        moduleFrontRight.setState(stateFrontRight);
        moduleBackLeft.setState(stateBackLeft);
        moduleBackRight.setState(stateBackRight);
    }

    public void stop() {
        moduleBackLeft.stop();
        moduleBackRight.stop();
        moduleFrontLeft.stop();
        moduleFrontLeft.stop();
    }

    @Override
    public void periodic() {
        moduleFrontLeft.periodic();
        moduleFrontRight.periodic();
        moduleBackLeft.periodic();
        moduleBackRight.periodic();
    }

    public void setBrakeMode(){
        moduleFrontLeft.setBrakeMode();
        moduleFrontRight.setBrakeMode();
        moduleBackLeft.setBrakeMode();
        moduleBackRight.setBrakeMode();
    }

    public void setCoastMode(){
        moduleFrontLeft.setCoastMode();
        moduleFrontRight.setCoastMode();
        moduleBackLeft.setCoastMode();
        moduleBackRight.setCoastMode();
    }

    public Pose2d getPose(){
        return odometrySubsystem.getOdometry().getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        resetPose(pose);
    }

    public void resetPose(Pose2d pose){
        odometrySubsystem.resetOdometry(pose);
    }

    public SwerveModule getSwerveModule(String module){
        switch (module){
            case "FL":
                return moduleFrontLeft;
            case "BL":
                return moduleBackLeft;
            case "FR":
                return moduleFrontRight;
            case "BR":
                return moduleBackRight;
            default:
                return null;
        }
    }
}