package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final double opposite = 12.5;
  private double derivative = 0;
  private double previous_error = 0;
  private double Kp = 0;
  private double Ki = 0;
  private double Kd = 0;
  private double integral = 0;
  private double distance;

  //Andrew's Code
  public final double DISTANCE_FROM_TARGET = 40.0;
  public final double CAM_HEIGHT = 14.375;
  public final double TARGET_HEIGHT = 27.5;
  public final double HEIGHT = TARGET_HEIGHT - CAM_HEIGHT;
  public final double INITIAL_ANGLE = Math.atan(HEIGHT / DISTANCE_FROM_TARGET);
  public final double ANGLE_ERROR = 0.45;

  Joystick stick = new Joystick(0);

  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);

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

    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");

    final double defaultDistanceSpeed = -0.1;
    final double defaultHorizontalSpeed = -0.01;
    double x = tx.getDouble(0.1234);
    double y = ty.getDouble(0.1234);
    double area = ta.getDouble(0.1234);

    int tshortVal = (int)tshort.getDouble(0.1234);
    int tlongVal = (int)tlong.getDouble(0.1234);

    double leftMovement = 0.0; //stick.getRawAxis(1) * 0.5;
    double rightMovement = 0.0; //stick.getRawAxis(5) * 0.5;

    double horizontalSpeed = 0;
    double minCommand = 0.05;
    double distance = 0;//distanceCalc(degreeToRadian(y))

    double distanceSpeed = 0;
    double automatedBaseSpeed = 0.05;

    distance = HEIGHT / Math.tan(degreeToRadian(y) + INITIAL_ANGLE);

    //if horizontal angle is positive, turn robot to negative and run curve code
    //if horizontal angle is negative, turn robot to positive and run curve code
    if (distance >= 40)
    {
      if (stick.getRawButton(1))
      { 
         if (x > 5.0)
         {
           runAt(0.1, speedCalc(distance));
         }
         else if (x < -5.0)
         {
           runAt(speedCalc(distance), 0.1);
         }
      }
    }
  }

  
  @Override
  public void testPeriodic() {
    
  }

  private static double degreeToRadian(double degree)
  {
      return Math.toRadians(degree);
  } 

  private static double distanceCalc(double radian)
   {
       double opposite = 12.5;
       double final1 = opposite / Math.tan(radian);
       return final1;
   }

   public void runAt(double leftSpeed, double rightSpeed) 
   {        
      leftMotor.set(leftSpeed);
      rightMotor.set(rightSpeed);
   }


public static double speedCalc(double d)
{
      final int widthRobot = 70;
      double radian;
      double orad, irad;
      double Rtn;

      radian = d / Math.sqrt(2);        
      orad = radian + (0.5 * widthRobot);
      irad = radian - (0.5 * widthRobot);

      return Rtn = 0.1 * (irad / orad);
  }
}
