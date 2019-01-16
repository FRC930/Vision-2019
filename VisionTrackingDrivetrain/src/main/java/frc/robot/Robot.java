
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
 
  Joystick stick = new Joystick(0);

  WPI_TalonSRX leftMotor = new WPI_TalonSRX(0);
  WPI_TalonSRX rightMotor = new WPI_TalonSRX(3);

  VictorSPX leftFollow1 = new VictorSPX(1);
  VictorSPX leftFollow2 = new VictorSPX(2);
  VictorSPX rightFollow1 = new VictorSPX(4);
  VictorSPX rightFollow2 = new VictorSPX(5);

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  
  @Override
  public void robotInit() {

    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
   
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double target = tv.getDouble(0.0);
 
    int tshortVal = (int)tshort.getDouble(0.0);
    int tlongVal = (int)tlong.getDouble(0.0);

    double leftMovement = stick.getRawAxis(1);
    double rightMovement = stick.getRawAxis(5);
    double adjust = 0;

    drive.setDeadband(0.1);

    if (target == 1.0) {
      if(stick.getRawButton(1) && x != 0) {
        adjust = 0;

        adjust = x * 0.1;

        leftMovement += adjust;
        rightMovement -= adjust;
      }
    }

    drive.tankDrive(leftMovement, rightMovement);
  }

  @Override
  public void testPeriodic() {
  }

  private double lerp(double a, double b, double t) {
    return a * (1 - t) + (b * t);
  }
}
