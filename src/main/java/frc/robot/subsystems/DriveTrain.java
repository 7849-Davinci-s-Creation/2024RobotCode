package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public  class DriveTrain extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.MotorConstants.LeftFrontMotor, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.MotorConstants.LeftBackMotor, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.MotorConstants.RightFrontMotor, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.MotorConstants.RightBackMotor, MotorType.kBrushless);
  
  
  public DriveTrain() {
    // set back motors to follow the front ones.
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
  }

  public void arcadeDrive(double rotate,double drive) {
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
      return part1 + Constants.DriveTrain.TORQUE_RESITANCE_THRESHOLD;
    } else if (position < 0) {
      return part1 - Constants.DriveTrain.TORQUE_RESITANCE_THRESHOLD;
    }

    // else joystick position is 0 so return 0
    return 0;
  }

  public double handleDeadzone(double value, double deadZone) {
    if (Math.abs(value) < deadzone) {
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
  
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
