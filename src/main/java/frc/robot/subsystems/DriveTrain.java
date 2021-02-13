package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.commands.ResetEncoders;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;



public class DriveTrain extends SubsystemBase {



    private WPI_TalonSRX m_frontRightMotor = new WPI_TalonSRX(Constants.CAN_ID_MOTOR_DRIVETRAIN_FRONT_RIGHT);
    private WPI_TalonSRX m_frontLeftMotor = new WPI_TalonSRX(Constants.CAN_ID_MOTOR_DRIVETRAIN_FRONT_LEFT);
    private WPI_TalonSRX m_backRightMotor = new WPI_TalonSRX(Constants.CAN_ID_MOTOR_DRIVETRAIN_BACK_RIGHT);
    private WPI_TalonSRX m_backLeftMotor = new WPI_TalonSRX(Constants.CAN_ID_MOTOR_DRIVETRAIN_BACK_LEFT);
    private final SpeedControllerGroup m_rightSpeedControllerGroup = new SpeedControllerGroup(m_frontRightMotor, m_backRightMotor);
    private final SpeedControllerGroup m_leftSpeedControllerGroup = new SpeedControllerGroup(m_frontLeftMotor, m_backLeftMotor);
    
    
    private Encoder m_frontRightEncoder = new Encoder(Constants.ENCODER_ID_DRIVETRAIN_FRONT_RIGHT_A, Constants.ENCODER_ID_DRIVETRAIN_FRONT_RIGHT_B, true, Encoder.EncodingType.k4X);
    private Encoder m_frontLeftEncoder = new Encoder(Constants.ENCODER_ID_DRIVETRAIN_FRONT_LEFT_A, Constants.ENCODER_ID_DRIVETRAIN_FRONT_LEFT_B, false, Encoder.EncodingType.k4X);
    private Encoder m_backRightEncoder = new Encoder(Constants.ENCODER_ID_DRIVETRAIN_BACK_RIGHT_A, Constants.ENCODER_ID_DRIVETRAIN_BACK_RIGHT_B, true, Encoder.EncodingType.k4X);
    private Encoder m_backLeftEncoder = new Encoder(Constants.ENCODER_ID_DRIVETRAIN_BACK_LEFT_A, Constants.ENCODER_ID_DRIVETRAIN_BACK_LEFT_B, false, Encoder.EncodingType.k4X);

    private AHRS m_navXmxp = new AHRS(SPI.Port.kMXP);


    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftSpeedControllerGroup, m_rightSpeedControllerGroup);

    private final DifferentialDriveOdometry m_odometry;


    public DriveTrain() {

        ConfigureEncoders();

        m_odometry = new DifferentialDriveOdometry(m_navXmxp.getRotation2d());


    }

    @Override
    public void periodic() {
        m_odometry.update(m_navXmxp.getRotation2d(), m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance());

    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_frontLeftEncoder.getRate(), m_frontRightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_navXmxp.getRotation2d());
    }

    public void arcardeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftSpeedControllerGroup.setVoltage(leftVolts);
        m_rightSpeedControllerGroup.setVoltage(rightVolts);
        m_drive.feed();
    }






    private void ConfigureEncoders() {

        if (m_frontRightEncoder != null) {
            m_frontRightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
            m_frontRightEncoder.setMaxPeriod(Constants.ENCODER_MAX_PERIOD);
            m_frontRightEncoder.setMinRate(Constants.ENCODER_MIN_RATE);
            m_frontRightEncoder.setSamplesToAverage(Constants.ENCODER_SAMPLES_TO_AVERAGE);
            //m_frontRightEncoder.setReverseDirection(true);
            ResetEncoder(m_frontRightEncoder);
        }

        if (m_frontLeftEncoder != null) {
            m_frontLeftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
            m_frontLeftEncoder.setMaxPeriod(Constants.ENCODER_MAX_PERIOD);
            m_frontLeftEncoder.setMinRate(Constants.ENCODER_MIN_RATE);
            m_frontLeftEncoder.setSamplesToAverage(Constants.ENCODER_SAMPLES_TO_AVERAGE);
            //m_frontLeftEncoder.setReverseDirection(true);
            ResetEncoder(m_frontLeftEncoder);

        }

        if (m_backRightEncoder != null) {
            m_backRightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
            m_backRightEncoder.setMaxPeriod(Constants.ENCODER_MAX_PERIOD);
            m_backRightEncoder.setMinRate(Constants.ENCODER_MIN_RATE);
            m_backRightEncoder.setSamplesToAverage(Constants.ENCODER_SAMPLES_TO_AVERAGE);
            //m_backRightEncoder.setReverseDirection(true);
            ResetEncoder(m_backRightEncoder);
        }

        if (m_backLeftEncoder != null) {
            m_backLeftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
            m_backLeftEncoder.setMaxPeriod(Constants.ENCODER_MAX_PERIOD);
            m_backLeftEncoder.setMinRate(Constants.ENCODER_MIN_RATE);
            m_backLeftEncoder.setSamplesToAverage(Constants.ENCODER_SAMPLES_TO_AVERAGE);
            //m_backLeftEncoder.setReverseDirection(true);
            ResetEncoder(m_backLeftEncoder);
        }


    }

    private void ResetEncoder(Encoder enc) {
        if (enc != null) {
            enc.reset();
        }
    }

    public void resetEncoders() {
        ResetEncoder(m_frontRightEncoder);
        ResetEncoder(m_frontLeftEncoder);
        ResetEncoder(m_backRightEncoder);
        ResetEncoder(m_backLeftEncoder);
    }

    private double GetDistance(Encoder enc1, Encoder enc2) {
        double distance = 0.0;
        if (enc1 != null && enc2 != null) {
            double distance1 = enc1.getDistance();
            double distance2 = enc2.getDistance();
            distance = (distance1 + distance2) / 2.0;
        } else if (enc1 != null) {
            distance = enc1.getDistance();
        }
        return distance;
    }

    public double getDistance() {
        return GetDistance(m_frontRightEncoder, null);
    }

    public double getAverageEncoderDistance() {
        return GetDistance(m_frontRightEncoder, m_frontLeftEncoder);
    }

    private double GetRawDistance(Encoder enc) {
        double distance = 0.0;
        if (enc != null) {
            distance = enc.getDistance();
        }
        return distance;
    }

    public double getFrontLeftDistance() {
        return GetRawDistance(m_frontLeftEncoder);
    }

    public double getBackRightDistance() {
        return GetRawDistance(m_backRightEncoder);
    }

    public double getBackLeftDistance() {
        return GetRawDistance(m_backLeftEncoder);
    }

    public Encoder getLeftEncoder() {
        return m_frontLeftEncoder;
    }

    public Encoder getRightEncoder() {
        return m_frontRightEncoder;
    }

    public double getFrontRightDistance() {
        return GetRawDistance(m_frontRightEncoder);
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
    
    public void zeroHeading() {
        m_navXmxp.reset();
    }

    public double getHeading() {
        return m_navXmxp.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return m_navXmxp.getRate();
    }

    public void resetGyro() {
        if (m_navXmxp != null) {
            m_navXmxp.reset();
        }
    }

    public double getGyroAngle() {
        double value = 0.0;
        if (m_navXmxp != null) {
            value = m_navXmxp.getAngle();
        }
        return value;
    }

    public double getGyroYaw() {
        double value = 0.0;
        if (m_navXmxp != null) {
            value = m_navXmxp.getYaw();
        }
        return value;
    }

    public void drive(double left, double right) {
        m_drive.tankDrive(left, right);
    }



    

    

}