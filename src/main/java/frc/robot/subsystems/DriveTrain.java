package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.DriveTrainConstants.TRACK_WIDTH_METERS);

  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

  private final SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)),
      Volts.of(4),
      Seconds.of(5.5),
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

  public void voltageTankDrive(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    differentialDrive.feed();
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
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

  public DifferentialDriveKinematics getKinematics() {
    return this.kinematics;
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
    odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
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

    if (RobotContainer.isDebugMode()) {
      SmartDashboard.putNumber("Gyro Heading", getHeading());
      SmartDashboard.putNumber("Left Encoder Value (feet)", getLeftEncoderPosition());
      SmartDashboard.putNumber("Right Encoder Value (feet) ", getRightEncoderPosition());
    }
  }
}
