package frc.robot;

import javax.lang.model.util.ElementScanner6;

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

  final double defaultHorizontalSpeed = -0.01;

  Joystick stick = new Joystick(0);
  double horizontalSpeed = 0;
  double minCommand = 0.05;

  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    /*//////////////////////////////////////////
    // Drive code for testing
    leftStick = stick.getRawAxis(1);
    rightStick = stick.getRawAxis(5);

    if (Math.abs(leftStick) < 0.05) {
      leftMotor.set(0);
    }
    if (Math.abs(rightStick) < 0.05) {
      rightMotor.set(0);
    }

    leftMotor.set(leftStick);
    rightMotor.set(rightStick);
    //////////////////////////////////////////*/

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry tx0 = table.getEntry("tx0");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ty0 = table.getEntry("ty0");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");


    //leftMotor.configSetCustomParam(newValue, paramIndex, timeoutMs);

    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    //NewworkTableEntry ledMode = table.getEntry("ledMode");

    //read values periodically
    // 0.1234 is the default value of the angle returned
    final double defaultDistanceSpeed = -0.1;
    double x = tx.getDouble(0.1234);
    double y = ty.getDouble(0.1234);
    double x0 = tx0.getDouble(0.1234);
    double y0 = ty0.getDouble(0.1234);
    double s = ts.getDouble(0.1234);
    double area = ta.getDouble(0.1234);
    
    int tshortVal = (int)tshort.getDouble(0.1234);
    int tlongVal = (int)tlong.getDouble(0.1234);
    int thorVal = (int)thor.getDouble(0.1234);
    int tvertVal = (int)tvert.getDouble(0.1234);

    double leftMovement = 0.0; //stick.getRawAxis(1) * 0.5;
    double rightMovement = 0.0; //stick.getRawAxis(5) * 0.5;
    //double horizontalSpeedLeft = 0;
    //double horizontalSpeedRight = 0;
    double distance = distanceCalc(degreeToRadian(y));
    //double distanceSpeedLeft = 0;
    //double distanceSpeedRight = 0;
    double distanceSpeed = 0;
    double automatedBaseSpeed = 0.05;

    final double horizontalFOV = degreeToRadian(54);
    final double verticalFOV = degreeToRadian(41);
    double viewPlaneWidth = 0;
    double viewPlaneHeight = 0;
    double viewPlaneX = 0;
    double viewPlaneY = 0;
    double viewPlanePitch = 0;
    double viewPlaneYaw = 0;
   
   // System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
   // System.out.println("Height(in): 13 " + "Distance: " + distance + "Y limelight angle: " + y);



	  if(stick.getRawButton(1) /*&& x != 0*/) {
      horizontalSpeed = 0;
      distanceSpeed = 0;

      // rotate the robot if horizontal angle is greater than 8
      if(Math.abs(x) > 8.0) {
        distanceSpeed = 0;
        rotate(x);
      }
      // drive straight if not rotating
      else {
        horizontalSpeed = 0;
        distanceSpeed = y / 30.0;
      }

      // left and right speeds
      leftMovement = distanceSpeed + horizontalSpeed;
      rightMovement = distanceSpeed - horizontalSpeed;

      //leftMovement -= automatedBaseSpeed + horizontalSpeed + distanceSpeed;
      //rightMovement += automatedBaseSpeed + horizontalSpeed + distanceSpeed;
    }


    // calculations for finding raw angles
    viewPlaneWidth = 2.0 * Math.tan(horizontalFOV / 2);
    viewPlaneHeight = 2.0 * Math.tan(verticalFOV / 2);

    viewPlaneX = viewPlaneWidth / 2.0 * x0;
    viewPlaneY = viewPlaneHeight / 2.0 * y0;

    viewPlanePitch = Math.atan2(1, viewPlaneY); //- (Math.PI / 2.0);
    viewPlaneYaw = Math.atan2(1, viewPlaneX); //- (Math.PI / 2.0);


    // run the robot
    runAt(leftMovement, -rightMovement);

    System.out.println("********************************************************");
    //System.out.println("tx: " + x);
    //System.out.println("ty: " + y);
    System.out.println("view plane pitch: " + viewPlanePitch);
    System.out.println("view plane yaw: " + viewPlaneYaw);
    //System.out.println("ts: " + s);
    //System.out.println("thor: " + thorVal);
    //System.out.println("tvert: " + tvertVal);
    /*System.out.println("left horizontal speed:" + horizontalSpeed);
    System.out.println("right horizontal speed:" + horizontalSpeed);
    System.out.println("left distance speed:" + distanceSpeed);
    System.out.println("right distance speed:" + distanceSpeed);
    System.out.println("left movement:" + leftMovement);
    System.out.println("right movement:" + rightMovement);*/
    System.out.println("********************************************************");
  }



  /**
   * This function is called periodically during test mode.
   */
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

   // drive the robot
   public void runAt(double leftSpeed, double rightSpeed) {
        
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
   }

   // rotate the robot based on horizontal angle offset
   public void rotate(double horizontalAngle) {
    if(horizontalAngle > 5.0) {
      horizontalSpeed = defaultHorizontalSpeed * (horizontalAngle / 2) - minCommand;
    }
    else if(horizontalAngle < -5.0) {
      horizontalSpeed = defaultHorizontalSpeed * (horizontalAngle / 2) + minCommand;
    }
   }
}