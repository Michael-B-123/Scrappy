package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.SwerveModule;

public class Constants {

    public static final double Delta = 1e-2;
    
    public static final Translation2d[] moduleLocations = {
        new Translation2d(-0.29845,0.29845),  //front right ++
        new Translation2d(-0.29845,-0.29845), //front left  -+
        new Translation2d(0.29845,-0.29845),  //rear left   --
        new Translation2d(0.29845,0.29845)  //rear right  +-
    };
    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = 0.42207203769;
    public static final double attainableMaxModuleSpeedMPS = 4.572;
    public static final double attainableMaxTranslationalSpeedMPS = attainableMaxModuleSpeedMPS;
    public static final double attainableMaxRotationalVelocityRPS = attainableMaxModuleSpeedMPS/DriveBaseRadius;

    public static final int PigeonID = 22;   

    public static double controllerDeadband = 0.15; 

    public interface Modules{
        public static final double SpeedKP = 0.001, SpeedKI = 0, SpeedKD = 0.0005;
        public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
    
        public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;
        public static final double FrontLeftEncoderOffset = -0.456;//-0.423340 rotations raw = 0.000000 rotations
        // public static final double FrontLeftEncoderOffset = 0;

        public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;
        public static final double FrontRightEncoderOffset = -0.347;//0.484131 rotations raw = -0.000244 rotations
        // public static final double FrontRightEncoderOffset = 0;

        public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;
        public static final double RearLeftEncoderOffset = 0.386;//0.283691 rotations raw = -0.000244 rotations
        // public static final double RearLeftEncoderOffset = 0;

        public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;
        public static final double RearRightEncoderOffset = 0.131;//0.448730 rotations raw = 0.000244 rotations
        // public static final double RearRightEncoderOffset = 0;

        SwerveModule[] moduleArray = new SwerveModule[] {
            new SwerveModule(FrontRightDriveID, FrontRightSteerID, FrontRightEncoderID),
            new SwerveModule(FrontLeftDriveID, FrontLeftSteerID, FrontLeftEncoderID),
            new SwerveModule(RearLeftDriveID, RearLeftSteerID, RearLeftEncoderID),
            new SwerveModule(RearRightDriveID, RearRightSteerID, RearRightEncoderID)
        };
        
    }

    public interface Drivetrain {
        public static final double SpeedKP = 5, SpeedKI = 0, SpeedKD = 0;
        public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;

        public interface Odometry {
            public static final double PositionStdDev = 0.1;
            public static final double AngleStdDev = 0.05;
        }

        public static final String CameraName = "front";

        public static final Transform3d RobotToCamera = new Transform3d(
            new Translation3d(0.0889, -0.00635, 0.47625),
            new Rotation3d(0, 0, 0)
        );

        public static final AprilTagFieldLayout FieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 999999);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 999999);

        public static final double TranslationPow = 3;
        public static final double RotationPow = 3;

        public static final double SlowFactor = 3;
        public static final double SlowFactorOffset = 1;
    }

    public interface Motion {
        public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
        public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
    }

}


