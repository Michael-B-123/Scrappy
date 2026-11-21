package frc.robot.Subsystems;

import frc.robot.Utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Commands.DriveFieldRelative;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Drivetrain extends SubsystemBase {
    Pigeon2 IMU;
    CANrange CANRange;
    public SwerveModule[] modules;
    SwerveDriveKinematics kinematics;
    double translationMaxAccelerationMetersPerSecondSquared = 25;
    double rotationMaxAccelerationRadiansPerSecondSquared = 50;
    SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);
    private final PIDController visionForwardBackController = new PIDController(Constants.Drivetrain.SpeedKP, 0, 0);
    private final PIDController visionSidewaysController = new PIDController(Constants.Drivetrain.SpeedKP, 0, 0);
    private final PIDController visionRotationsController = new PIDController(Constants.Drivetrain.SteerKP, 0, 0);

    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean hasRecievedVisionMeasurement;
    private boolean headingInitialized;
    private boolean crashAvoidance;
    private final Field2d field = new Field2d();
    private record cameraSetup(PhotonCamera camera, PhotonPoseEstimator estimator) {}
    private final List<cameraSetup> cameras= new ArrayList<>();
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller) {
        IMU = new Pigeon2(Constants.PigeonID, new CANBus("*"));

        CANRange = new CANrange(Constants.CANRangeID, new CANBus("*"));
        CANrangeConfiguration crConfig = new CANrangeConfiguration();
        crConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        crConfig.ToFParams.UpdateFrequency = 50;
        crConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        crConfig.ProximityParams.ProximityThreshold = Constants.ProximityThreshold;
        crConfig.ProximityParams.ProximityHysteresis = Constants.ProximityHysteresis;
        CANRange.getConfigurator().apply(crConfig);

        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        visionRotationsController.enableContinuousInput(-Math.PI, Math.PI);
        setDefaultCommand(new DriveFieldRelative(this, controller));

        for (Constants.Drivetrain.Camera cameraConfig : Constants.Drivetrain.Cameras) {
            cameras.add(new cameraSetup(
                new PhotonCamera(cameraConfig.name()), 
                new PhotonPoseEstimator(Constants.Drivetrain.FieldLayout, cameraConfig.transform()))
            );
        }

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroscopeRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.AngleStdDev),
            // We calculate these upon update provide those real details 
            VecBuilder.fill(999999, 999999, 999999)
        );

        SmartDashboard.putData("Field", field);

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(this);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChasisSpeed,
            (speeds, feedforwards) -> drive(speeds),

            new PPHolonomicDriveController(
                new PIDConstants(Constants.Drivetrain.SpeedKP, Constants.Drivetrain.SpeedKI, Constants.Drivetrain.SpeedKD),
                new PIDConstants(Constants.Drivetrain.SteerKP, Constants.Drivetrain.SteerKI, Constants.Drivetrain.SteerKD)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (!alliance.isPresent()) {
                return false;
              }
              return alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );

        SmartDashboard.putBoolean("Crash avoidance", false);
    }

    public Optional<Pose2d> getTargetPose(int tagId, Translation2d offset, Rotation2d theta) {
        Optional<Pose3d> tag = Constants.Drivetrain.FieldLayout.getTagPose(tagId);
        if (tag.isEmpty()) return Optional.empty();

        Pose2d targetPose = tag.get().toPose2d().transformBy(new Transform2d(offset.unaryMinus(), theta));

        return Optional.of(targetPose);
    }

    public Command aimAtTag(int tagId, Translation2d offset, Rotation2d theta) {
        return getTargetPose(tagId, offset, theta)
            .map(this::driveToPose)
            .orElse(Commands.none());
    }

    public Command driveToPose(Pose2d targetPose) {
        return run(() -> {
            Pose2d robotPose = getPose();

            double deadband = Constants.Drivetrain.autoAimDeadband;

            double forwardVelocity = visionForwardBackController.calculate(robotPose.getX(), targetPose.getX());
            forwardVelocity = Utils.deadband(forwardVelocity, deadband);
            double sidewaysVelocity = visionSidewaysController.calculate(robotPose.getY(), targetPose.getY());
            sidewaysVelocity = Utils.deadband(sidewaysVelocity, deadband);
            double angularVelocity = visionRotationsController.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            angularVelocity = Utils.deadband(angularVelocity, deadband);

            driveFieldRelative(new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity), robotPose.getRotation());
        });
    }

    public Command aimAtTagPP(int tagId, Translation2d offset, Rotation2d theta) {
        return getTargetPose(tagId, offset, theta)
            .map(this::driveToPosePP)
            .orElse(Commands.none());
    }

    public Command driveToPosePP(Pose2d targetPose) {
        return Commands.defer(() -> {
            Pose2d robotPose = getPose();
            Translation2d delta = targetPose.getTranslation().minus(robotPose.getTranslation());

            if (delta.getNorm() < 0.05) {
                return Commands.none();
            }

            Rotation2d travelDirection = delta.getAngle();

            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(List.of(
                    new Pose2d(robotPose.getTranslation(), travelDirection),
                    new Pose2d(targetPose.getTranslation(), travelDirection)
                )),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                null,
                new GoalEndState(0.0, targetPose.getRotation())
            );
            path.preventFlipping = true;

            return AutoBuilder.followPath(path);
        }, Set.of(this));
    }

    @Override
    public void periodic() {
        for (cameraSetup camSetup : cameras) {
            PhotonCamera camera = camSetup.camera();
            PhotonPoseEstimator photonEstimator = camSetup.estimator();
            
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEstimate.isEmpty()) {
                    visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
                }

                visionEstimate.ifPresent(e -> {
                    var targets = e.targetsUsed;
                    Pose2d estPose = e.estimatedPose.toPose2d();
                    field.getObject("vision pose").setPose(estPose);
                    
                    if ((targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.2) ||
                            targets.size() == 0) {
                        return;
                    }

                    if (Math.abs(e.estimatedPose.getZ()) > 0.5) {
                        return;
                    }

                    if (estPose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) > 5 && 
                        hasRecievedVisionMeasurement) {
                        return;
                    }

                    if (estPose.getX() < 0 || estPose.getX() > Constants.Drivetrain.FieldLayout.getFieldLength() ||
                        estPose.getY() < 0 || estPose.getY() > Constants.Drivetrain.FieldLayout.getFieldWidth()) {
                        return;
                    }

                    if (!hasRecievedVisionMeasurement) {
                        poseEstimator.resetPose(estPose);
                        hasRecievedVisionMeasurement = true;
                        return;
                    }

                    Matrix<N3, N1> stdDevs = getStdDevs(e, targets);
                    poseEstimator.addVisionMeasurement(estPose, e.timestampSeconds, stdDevs);
                    field.getObject("good vision pose").setPose(estPose);

                    double lag = Timer.getFPGATimestamp() - e.timestampSeconds;
                    SmartDashboard.putNumber("Vision/MeasurementLag_s", lag);
                });            
            }
        }

        poseEstimator.update(getGyroscopeRotation(), getModulePositions());
        setHeadingOffset();

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        crashAvoidance = SmartDashboard.getBoolean("Crash avoidance", false);
        SmartDashboard.putBoolean("CANRange proximity", CANRange.getIsDetected().getValue());
        SmartDashboard.putNumber("CANRange distance", Utils.RoundByPlaces(CANRange.getDistance().getValueAsDouble(), 2));
        SmartDashboard.putNumber("CANRange distance to center", Utils.RoundByPlaces((CANRange.getDistance().getValueAsDouble() + Constants.CANRangeToCenter), 2));
    }

    private void setHeadingOffset() {
        if (!headingInitialized
            && DriverStation.isDisabled()
            && DriverStation.getAlliance().isPresent()
            && !hasRecievedVisionMeasurement
        ) {
            poseEstimator.resetPosition(
                getGyroscopeRotation(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), getFieldRotationTransform())
            );
            headingInitialized = true;
        }
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        var estStdDevs = Constants.Drivetrain.SingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            var tagPose = Constants.Drivetrain.FieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags > 0) {
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = Constants.Drivetrain.MultiTagStdDevs;
            } 
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
        }
        
        return estStdDevs;
    }

    public boolean getIsDetected() {
        return CANRange.getIsDetected().getValue();
    }

	public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getFieldRotationTransform() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        if (isRed) {
            return Rotation2d.fromDegrees(180);
        }
        return new Rotation2d();
    }

    public Rotation2d getFieldRelativeRotation() {
        return getGyroscopeRotation().rotateBy(getFieldRotationTransform());
    }

    public Rotation2d getGyroscopeRotation() {
        return IMU.getRotation2d();
    }

    public void resetGyro() {
        Pose2d pose = getPose();
        IMU.reset();
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
    }

    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void resetPose(Pose2d pose) {
        if (!hasRecievedVisionMeasurement) {
            poseEstimator.resetPose(pose);
            headingInitialized = true;
        }
    }
    
    public Command calibrate() {
        return parallel(
            runOnce(() -> {System.out.println("Swerve calibration triggered");}),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).ignoringDisable(true).withName("Calibrating");
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        setModuleTargetStates(chassisSpeeds, centerOfRotation);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
        driveFieldRelative(chassisSpeeds, getGyroscopeRotation());
    }

    public void driveFieldRelative(ChassisSpeeds fieldSpeeds, Rotation2d heading) {
        double vx = translationXLimiter.calculate(fieldSpeeds.vxMetersPerSecond);
        double vy = translationYLimiter.calculate(fieldSpeeds.vyMetersPerSecond);
        double omega = rotationLimiter.calculate(fieldSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading);

        if (crashAvoidance && getIsDetected() && robotRelative.vxMetersPerSecond > 0) {
            robotRelative.vxMetersPerSecond = 0;

            ChassisSpeeds fieldAfter = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading);
            translationXLimiter.reset(fieldAfter.vxMetersPerSecond);
            translationYLimiter.reset(fieldAfter.vyMetersPerSecond);
        }

        setModuleTargetStates(robotRelative);
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.attainableMaxModuleSpeedMPS);
        for (int i = 0; i < modules.length; ++i) {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
    }

    public ChassisSpeeds getChasisSpeed() {
        return kinematics.toChassisSpeeds(
            modules[0].getSwerveModuleState(),
            modules[1].getSwerveModuleState(),
            modules[2].getSwerveModuleState(),
            modules[3].getSwerveModuleState()
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        var frontLeftModule = modules[1];
        var frontRightModule = modules[0];
        var backLeftModule = modules[2];
        var backRightModule = modules[3];
    
        builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroscopeRotation().getRadians(), null);
    }
}