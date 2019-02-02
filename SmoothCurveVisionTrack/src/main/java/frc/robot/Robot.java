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
  private double derivative = 0.0;
  private double previous_error = 0.0;
  private double Kp = 0.0;
  private double Ki = 0.0;
  private double Kd = 0.0;
  private double integral = 0.0;
  private double distance = 0.0;
  private double tanActualAngle = 0.0;

  private final double COMPARE_ANGLE = 5.0;
  private final double ROTATION_SPEED = 0.1;

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
    double horizAngle = tx.getDouble(0.1234);
    double deltaAngle = ty.getDouble(0.1234);
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
    tanActualAngle = Math.tan(degreeToRadian(deltaAngle) + INITIAL_ANGLE);

    distance = HEIGHT / tanActualAngle;

    //if horizontal angle is positive, turn robot to negative and run curve code
    //if horizontal angle is negative, turn robot to positive and run curve code
    System.out.println(insideSpeedCalc(distance));
    System.out.println(outsideSpeedCalc(distance));
    System.out.println("DISTANCE: " + distance);
    System.out.println("Horizontal Angle:" + horizAngle);
    System.out.println("");

    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;

    if (stick.getRawButton(1)) //if the "A" button is held
    {
      if (distance >= DISTANCE_FROM_TARGET) //if the distance is greater than 40 inches
      { 
         if (horizAngle > COMPARE_ANGLE)
         {
          //runAt(-ROTATION_SPEED, speedCalc(distance));
          leftMotorSpeed = -outsideSpeedCalc(distance);
          rightMotorSpeed = ROTATION_SPEED;
         }
         else if (horizAngle < -COMPARE_ANGLE)
         {
          //runAt(-speedCalc(distance), ROTATION_SPEED);
          rightMotorSpeed = ROTATION_SPEED; // FIGURE OUT ROTATING FROM RIGHT
          leftMotorSpeed = -outsideSpeedCalc(distance);
         }
         else
         {
           runAt(0.0, 0.0);
         }
      }
    }
    System.out.println("Left Motor Speed: " + leftMotorSpeed);
    System.out.println("Right Motor Speed: " + rightMotorSpeed);
    runAt(leftMotorSpeed, rightMotorSpeed);
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


public static double insideSpeedCalc(double d)
{
      final double widthRobot = 25.5;
      double radian;
      double orad, irad;

      radian = d / Math.sqrt(2);        
      orad = radian + (0.5 * widthRobot);
      irad = radian - (0.5 * widthRobot);

      return 0.1 * (irad / orad);
}

public static double outsideSpeedCalc(double d)
{
      final double widthRobot = 25.5;
      double radian;
      double orad, irad;

      radian = d / Math.sqrt(2);        
      orad = radian + (0.5 * widthRobot);
      irad = radian - (0.5 * widthRobot);

      return 0.1 * (orad / irad);
  }
}