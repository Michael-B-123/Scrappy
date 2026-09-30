package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    //drive 
    SparkMax driveMotor;
    int driveMotorID;
    SparkAbsoluteEncoder driveMotorEncoder;
    SparkClosedLoopController  driveController;

    //steer
    SparkMax steerMotor;
    SparkAbsoluteEncoder steerMotorEncoder;
    PIDController steerController;

    //module encoder 
    int encoderID;
    CANcoder moduleEncoder;
    private static final String EncoderPreferenceKey = "EncoderOffset";

    //conversion factors
    final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    final double GEAR_RATIO = 1.0 / 6.75;
    final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
    final double STEER_POSITION_CONVERSION = 1;
    final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID){
        this.driveMotorID = driveMotorID;

        //drive motor 
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(40);
        driveConfig.idleMode(IdleMode.kBrake);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //drive encoder
        driveMotorEncoder = driveMotor.getAbsoluteEncoder();

        //steer motor
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);
        steerConfig.smartCurrentLimit(20);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // module encoder
        moduleEncoder = new CANcoder(encoderID);

        //controllers
        //driveController = driveMotor.getPIDController();
        //driveController.setP(Constants.Modules.SpeedKP);
        //driveController.setI(Constants.Modules.SpeedKI);
        //driveController.setD(Constants.Modules.SpeedKD);

        steerController = new PIDController(Constants.Modules.SteerKP, Constants.Modules.SteerKI, Constants.Modules.SteerKD);
        steerController.enableContinuousInput(0, 1);

    }

    public void setTargetState(SwerveModuleState targetState) {
        double currentAngle = getModuleAngRotations();
        targetState.optimize(Rotation2d.fromRotations(currentAngle));
        steerMotor.set(-steerController.calculate(currentAngle, targetState.angle.getRotations()));
        targetState.speedMetersPerSecond *= targetState.angle.minus(Rotation2d.fromRotations(currentAngle)).getCos();
        driveMotor.set(targetState.speedMetersPerSecond/Constants.attainableMaxModuleSpeedMPS); 
    }

    public Command calibrate() {
        return runOnce(() -> {
            var adjustedEncoderValue = moduleEncoder.getAbsolutePosition().getValueAsDouble();
            var steerEncoderConfig = new CANcoderConfiguration();
            moduleEncoder.getConfigurator().refresh(steerEncoderConfig);
            double offset = steerEncoderConfig.MagnetSensor.MagnetOffset;
            var rawEncoderValue = adjustedEncoderValue - offset;
            steerEncoderConfig.MagnetSensor.MagnetOffset = -rawEncoderValue;
            moduleEncoder.getConfigurator().apply(steerEncoderConfig);
            Preferences.setDouble(EncoderPreferenceKey + encoderID, rawEncoderValue);
            if (Math.abs(moduleEncoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble()) < 0.01) {
                System.out.println("Succesfully calibrated swerve " + encoderID);
            }
            else {
                System.out.println("Swerve " + encoderID + " calibration failed");
            }
        }).ignoringDisable(true).withName("Calibrate");
    }

    public void periodic() {
        SmartDashboard.putNumber("S" + driveMotorID, getModuleAngRotations());
    }

    public double getModuleAngRotations(){
        return moduleEncoder.getAbsolutePosition().getValueAsDouble();
    }
    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotorEncoder.getPosition(),
            Rotation2d.fromRotations(getModuleAngRotations())
        );  
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            driveMotorEncoder.getVelocity(),
            Rotation2d.fromRotations(getModuleAngRotations()));
    }
}
