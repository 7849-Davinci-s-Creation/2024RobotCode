package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class DriveTrain extends SubsystemBase implements DashboardConfiguration {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.MotorConstants.LeftFrontMotor,
      MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.MotorConstants.LeftBackMotor,
      MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.MotorConstants.RightFrontMotor,
      MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.MotorConstants.RightBackMotor,
      MotorType.kBrushless);

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  private boolean isBoosted = false;
  private boolean isCreeping = false;
  private boolean isNormal = true;

  private boolean navXDebugMode = false;

  public DriveTrain() {
    // set back motors to follow the front ones.
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    leftFrontMotor.setInverted(true);
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

  public double applyCurve(double position) {
    // first part of equation is the same so extract to variable
    double part1 = (1 - Constants.DriveTrainConstants.TORQUE_RESITANCE_THRESHOLD) * Math.pow(position, 3);

    // apply piecewise logic
    if (position > 0) {
      return part1 + Constants.DriveTrainConstants.TORQUE_RESITANCE_THRESHOLD;
    } else if (position < 0) {
      return part1 - Constants.DriveTrainConstants.TORQUE_RESITANCE_THRESHOLD;
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

  public void setNavXDebugMode(boolean debug) {
    this.navXDebugMode = debug;
  }

  @Override
  public void periodic() {
    if (navXDebugMode) {
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

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void configureDashboard() {

  }
}
