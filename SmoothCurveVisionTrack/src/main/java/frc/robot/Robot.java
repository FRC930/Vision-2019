package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  /*
  private final double opposite = 12.5;
  private double derivative = 0.0;
  private double previous_error = 0.0;
  private double Kp = 0.0;
  private double Ki = 0.0;
  private double Kd = 0.0;
  private double integral = 0.0;
  private double distance = 0.0;
  */
  private double tanActualAngle = 0.0;
  
  private static final double COMPARE_ANGLE = 0.5;
  private static final double ROTATION_SPEED = 0.1;

  //Andrew's Code
  private static final double DISTANCE_FROM_TARGET = 15.25;
  private static final double CAM_HEIGHT = 27.5;
  private static final double TARGET_HEIGHT = 31.375;
  private static final double HEIGHT = TARGET_HEIGHT - CAM_HEIGHT;
  private static final double INITIAL_ANGLE = Math.atan(HEIGHT / DISTANCE_FROM_TARGET);
  private static final double ANGLE_ERROR = 0.45;
  private static final double HALF_ROBOT_WIDTH = 25.5 / 2;

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

    System.out.println("Press A");

    if (stick.getRawButton(1)) //if the "A" button is held
    {
      System.out.println("A is held.");
      if (distance >= DISTANCE_FROM_TARGET) //if the distance is greater than 40 inches
      { 
        System.out.println("DISTANCE >= DISTANCE_FROM_TARGET");
         if (horizAngle > COMPARE_ANGLE)
         {
           System.out.println("HorizAngle > CompareAngle");
          //runAt(-ROTATION_SPEED, speedCalc(distance));
          leftMotorSpeed = -outsideSpeedCalc(distance);
          rightMotorSpeed = ROTATION_SPEED;
         }
         else if (horizAngle < -COMPARE_ANGLE)
         {
          System.out.println("HorizAngle < -CompareAngle");
          //runAt(-speedCalc(distance), ROTATION_SPEED);
          rightMotorSpeed = ROTATION_SPEED; // FIGURE OUT ROTATING FROM RIGHT
          leftMotorSpeed = -outsideSpeedCalc(distance);
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

  private static double degreeToRadian(double degree) {
    return Math.toRadians(degree);
  } 

  private static double distanceCalc(double radian) {
    double opposite = 12.5;
    double final1 = opposite / Math.tan(radian);
    return final1;
  }

  public void runAt(double leftSpeed, double rightSpeed) {        
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

   //Calculate inside wheel speed with an input of distance
  public static double insideSpeedCalc(double d) {
    double radian; // Radian from the middle of the robot
    double orad, irad; // Outide radian, inside radian

    radian = d / Math.sqrt(2); // Cacl for radian from middle of the robot      
    orad = radian + (HALF_ROBOT_WIDTH); // Calc forutside radian
    irad = radian - (HALF_ROBOT_WIDTH); // Calc for inside wheel radian

    return (irad / orad);//0.6 * (irad / orad);
  }

  //Calculate outside wheel speed with an input of distance
  public static double outsideSpeedCalc(double d) {
    double radian; // Radian from the middle of the robot
    double orad, irad; // Outide radian, inside radian

    radian = d / Math.sqrt(2); //Calc for radian from middle of the robot 
    orad = radian + (HALF_ROBOT_WIDTH); // Calc for outside wheel radian
    irad = radian - (HALF_ROBOT_WIDTH); // Calc for inside wheel radian

    return (orad / irad);//0.6 * (orad / irad);
  }

} //end of class 


//meow