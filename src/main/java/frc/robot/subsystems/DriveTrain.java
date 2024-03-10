package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.DashboardConfiguration;

import static edu.wpi.first.units.Units.*;

public class DriveTrain extends SubsystemBase implements DashboardConfiguration {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.MotorConstants.LEFT_FRONT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.MotorConstants.LEFT_BACK_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_FRONT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_BACK_MOTOR,
      MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveTrainConstants.TRACK_WIDTH_METERS);

  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

  private final SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(3).per(Seconds.of(1)),
          Volts.of(3),
          Seconds.of(3),
          null);

  private final SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
          this::voltageDrive,
          this::log,
          this);

  private final SysIdRoutine driveTrainRoutine = new SysIdRoutine(config, mechanism);

  private final DifferentialDriveOdometry odometry;

  private boolean isBoosted = false;
  private boolean isCreeping = false;
  private boolean isNormal = true;
  private boolean isInverted = false;

  public DriveTrain() {
    zeroHeading();
    resetEncoders();

    // set back motors to follow the front ones.
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    leftFrontMotor.setInverted(true);

    // put into break mode for safety and ease of use!
    leftFrontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leftBackMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    rightFrontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    rightBackMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());

    leftEncoder.setPositionConversionFactor(Constants.DriveTrainConstants.LINEAR_CONVERSION_FACTOR);
    rightEncoder.setPositionConversionFactor(Constants.DriveTrainConstants.LINEAR_CONVERSION_FACTOR);

    leftEncoder.setVelocityConversionFactor(Constants.DriveTrainConstants.LINEAR_CONVERSION_FACTOR / 60);
    rightEncoder.setVelocityConversionFactor(Constants.DriveTrainConstants.LINEAR_CONVERSION_FACTOR / 60);

    AutoBuilder.configureRamsete(
            this::getPose2d,
            this::resetOdometry,
            this::getWheelSpeeds,
            this::pathDrive,
            new ReplanningConfig(),
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();
            },
            this
    );
  }

  public void arcadeDrive(double rotate, double drive) {
    double maximum = Math.max(Math.abs(rotate), Math.abs(drive));
    double total = drive + rotate;
    double difference = drive - rotate;

    if (drive >= 0) {
      if (rotate >= 0) {
        leftFrontMotor.set(maximum);
        rightFrontMotor.set(difference);

      } else {
        leftFrontMotor.set(total);
        rightFrontMotor.set(maximum);
      }
    } else {
      if (rotate >= 0) {
        leftFrontMotor.set(total);
        rightFrontMotor.set(-maximum);
      } else {
        leftFrontMotor.set(-maximum);
        rightFrontMotor.set(difference);
      }
    }

  }

  public void voltageDrive(Measure<Voltage> volts) {
    leftFrontMotor.setVoltage(volts.in(Volts));
    rightFrontMotor.setVoltage(volts.in(Volts));
  }

  public void pathDrive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    SimpleMotorFeedforward feeder = new SimpleMotorFeedforward(Constants.SysIDValues.KS, Constants.SysIDValues.KV);

    double leftRadPerSeconds = wheelSpeeds.leftMetersPerSecond / edu.wpi.first.math.util.Units.inchesToMeters(Constants.DriveTrainConstants.WHEEL_RADIUS_INCHES);
    double rightRadPerSeconds = wheelSpeeds.rightMetersPerSecond / edu.wpi.first.math.util.Units.inchesToMeters(Constants.DriveTrainConstants.WHEEL_RADIUS_INCHES);

    leftFrontMotor.setVoltage(feeder.calculate(leftRadPerSeconds));
    rightFrontMotor.setVoltage(feeder.calculate(rightRadPerSeconds));
  }

  public double applyCurve(double position) {
    // first part of equation is the same so extract to variable
    double part1 = (1 - Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD) * Math.pow(position, 3);

    // apply piecewise logic
    if (position > 0) {
      return part1 + Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD;
    } else if (position < 0) {
      return part1 - Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD;
    }

    // else joystick position is 0 so return 0
    return 0;
  }

  public double handleDeadzone(double value, double deadZone) {
    // why is this breaking?
    if (Math.abs(value) < deadZone) {
      return 0;
    }

    return value;
  }

  public void log(SysIdRoutineLog log) {
    int numberOfEntries = 2;

    double averageVoltage = ((leftFrontMotor.getAppliedOutput() * leftFrontMotor.getBusVoltage()) +
            (rightFrontMotor.getAppliedOutput() * rightFrontMotor.getBusVoltage())) / numberOfEntries;
    double averageLinearPosition = (getLeftEncoderPosition() + getRightEncoderPosition()) / numberOfEntries;
    double averageLinearVelocity = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / numberOfEntries;

    // drivetrain
    log.motor("drivetrain")
            .voltage(appliedVoltage.mut_replace(averageVoltage, Volts))
            .linearPosition(distance.mut_replace(averageLinearPosition, Meters))
            .linearVelocity(velocity.mut_replace(averageLinearVelocity, MetersPerSecond));
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -navx.getRate();
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navx.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(),
            pose);
  }

  public ChassisSpeeds getWheelSpeeds() {
    return kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                    leftEncoder.getVelocity(), rightEncoder.getVelocity()));
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
  }

  public void zeroHeading() {
    navx.reset();
  }

  public CANSparkMax getRightLeader() {
    return this.rightFrontMotor;
  }

  public CANSparkMax getLeftLeader() {
    return this.leftFrontMotor;
  }

  public Boolean isBoosted() {
    return this.isBoosted;
  }

  public Boolean isCreeping() {
    return this.isCreeping;
  }

  public boolean isNormal() {
    return this.isNormal;
  }

  public void setCreeping(boolean isCreeping) {
    this.isCreeping = isCreeping;
  }

  public void setBoosted(boolean isBoosted) {
    this.isBoosted = isBoosted;
  }

  public void setNormal(boolean isNormal) {
    this.isNormal = isNormal;
  }

  public boolean isInverted() {
    return this.isInverted;
  }

  public void setInverted(boolean isInverted) {
    this.isInverted = isInverted;
  }

  public void setNormalDriving() {
    this.setBoosted(false);
    this.setCreeping(false);
    this.setNormal(true);
  }

  public void setBoostedDriving() {
    this.setBoosted(true);
    this.setCreeping(false);
    this.setNormal(false);
  }

  public void setCreepedDriving() {
    this.setBoosted(false);
    this.setCreeping(true);
    this.setNormal(false);
  }

  public SysIdRoutine getRoutine() {
    return this.driveTrainRoutine;
  }

  @Override
  public void periodic() {
    this.configureDashboard();
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void configureDashboard() {
    // Log current driving mode to smart dashboard
    SmartDashboard.putBoolean("Inverted", this.isInverted());
    SmartDashboard.putBoolean("Boosted", this.isBoosted());
    SmartDashboard.putBoolean("Creep", this.isCreeping());
    SmartDashboard.putBoolean("Normal", this.isNormal());

    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("Left Encoder Value (feet)", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Value (feet) ", getRightEncoderPosition());

    if (RobotContainer.isDebugMode()) {
      SmartDashboard.putBoolean("IMU_Connected", navx.isConnected());
      SmartDashboard.putBoolean("IMU_IsCalibrating", navx.isCalibrating());
      SmartDashboard.putNumber("IMU_Yaw", navx.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", navx.getPitch());
      SmartDashboard.putNumber("IMU_Roll", navx.getRoll());

      /* Display tilt-corrected, Magnetometer-based heading (requires */
      /* magnetometer calibration to be useful) */

      SmartDashboard.putNumber("IMU_CompassHeading", navx.getCompassHeading());

      /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
      SmartDashboard.putNumber("IMU_FusedHeading", navx.getFusedHeading());

      /* These functions are compatible w/the WPI Gyro Class, providing a simple */
      /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

      SmartDashboard.putNumber("IMU_TotalYaw", navx.getAngle());
      SmartDashboard.putNumber("IMU_YawRateDPS", navx.getRate());

      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

      SmartDashboard.putNumber("IMU_Accel_X", navx.getWorldLinearAccelX());
      SmartDashboard.putNumber("IMU_Accel_Y", navx.getWorldLinearAccelY());
      SmartDashboard.putBoolean("IMU_IsMoving", navx.isMoving());
      SmartDashboard.putBoolean("IMU_IsRotating", navx.isRotating());

      /* Display estimates of velocity/displacement. Note that these values are */
      /* not expected to be accurate enough for estimating robot position on a */
      /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
      /* of these errors due to single (velocity) integration and especially */
      /* double (displacement) integration. */

      SmartDashboard.putNumber("Velocity_X", navx.getVelocityX());
      SmartDashboard.putNumber("Velocity_Y", navx.getVelocityY());
      SmartDashboard.putNumber("Displacement_X", navx.getDisplacementX());
      SmartDashboard.putNumber("Displacement_Y", navx.getDisplacementY());

      /* Display Raw Gyro/Accelerometer/Magnetometer Values */
      /* NOTE: These values are not normally necessary, but are made available */
      /* for advanced users. Before using this data, please consider whether */
      /* the processed data (see above) will suit your needs. */

      SmartDashboard.putNumber("RawGyro_X", navx.getRawGyroX());
      SmartDashboard.putNumber("RawGyro_Y", navx.getRawGyroY());
      SmartDashboard.putNumber("RawGyro_Z", navx.getRawGyroZ());
      SmartDashboard.putNumber("RawAccel_X", navx.getRawAccelX());
      SmartDashboard.putNumber("RawAccel_Y", navx.getRawAccelY());
      SmartDashboard.putNumber("RawAccel_Z", navx.getRawAccelZ());
      SmartDashboard.putNumber("RawMag_X", navx.getRawMagX());
      SmartDashboard.putNumber("RawMag_Y", navx.getRawMagY());
      SmartDashboard.putNumber("RawMag_Z", navx.getRawMagZ());
      SmartDashboard.putNumber("IMU_Temp_C", navx.getTempC());

      /* Omnimount Yaw Axis Information */
      /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
      AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
      SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
      SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

      /* Sensor Board Information */
      SmartDashboard.putString("FirmwareVersion", navx.getFirmwareVersion());

      /* Quaternion Data */
      /* Quaternions are fascinating, and are the most compact representation of */
      /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
      /* from the Quaternions. If interested in motion processing, knowledge of */
      /* Quaternions is highly recommended. */
      SmartDashboard.putNumber("QuaternionW", navx.getQuaternionW());
      SmartDashboard.putNumber("QuaternionX", navx.getQuaternionX());
      SmartDashboard.putNumber("QuaternionY", navx.getQuaternionY());
      SmartDashboard.putNumber("QuaternionZ", navx.getQuaternionZ());

      /* Connectivity Debugging Support */
      SmartDashboard.putNumber("IMU_Byte_Count", navx.getByteCount());
      SmartDashboard.putNumber("IMU_Update_Count", navx.getUpdateCount());
    }
  }
}
