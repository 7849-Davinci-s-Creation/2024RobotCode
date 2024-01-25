package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  WPI_VictorSPX LeftFrontMotor = new WPI_VictorSPX(Constants.MotorConstants.LeftFrontMotor);
  WPI_VictorSPX LeftBackMotor = new WPI_VictorSPX(Constants.MotorConstants.LeftBackMotor);
  WPI_VictorSPX RightFrontMotor = new WPI_VictorSPX(Constants.MotorConstants.RightFrontMotor);
  WPI_VictorSPX RightBackMotor = new WPI_VictorSPX(Constants.MotorConstants.RightBackMotor);
  
  MotorControllerGroup LeftMotors = new MotorControllerGroup(LeftBackMotor,LeftFrontMotor);
  MotorControllerGroup RightMotors = new MotorControllerGroup(RightBackMotor,RightFrontMotor);
  
  public DriveTrain() {
  }

  public void arcadeDrive (double rotate,double drive) {
    double maximum = Math.max(Math.abs(rotate), Math.abs(drive));
    double total = drive + rotate;
    double difference = drive - rotate;

    if (drive >= 0) {
      if (rotate >= 0) {
        LeftMotors.set(maximum);
        RightMotors.set(difference);
      
      } else {
        LeftMotors.set(total);
        RightMotors.set(maximum);
      } 
    } else { 
      if (rotate >= 0) {
        LeftMotors.set(total);
        RightMotors.set(-maximum);
      } else {
        LeftMotors.set(-maximum);
        RightMotors.set(difference);
      }
    }

  }
  
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
