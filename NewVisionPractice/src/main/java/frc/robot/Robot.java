package frc.robot;

import javax.lang.model.util.ElementScanner6;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final double opposite = 12.5;
  private double derivative = 0;
  private double previous_error = 0;
  private double Kp = 0;
  private double Ki = 0;
  private double Kd = 0;
  private double integral = 0;
  private double distance;
  private double errorCorrection = 0;

  final double defaultHorizontalSpeed = -0.01;

  Joystick stick = new Joystick(0);
  double horizontalSpeed = 0;
  double minCommand = 0.05;

  // real-world measurements
  public final double DISTANCE_FROM_TARGET = 40.0;
  public final double CAM_HEIGHT = 14.375;
  public final double TARGET_HEIGHT = 27.5;
  public final double HEIGHT = TARGET_HEIGHT - CAM_HEIGHT;
  public final double INITIAL_ANGLE = Math.atan(HEIGHT/DISTANCE_FROM_TARGET);
  public final double ANGLE_ERROR = 0.45;

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

    Shuffleboard.startRecording();

    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		//camera.setResolution(320, 240);
		//camera.setFPS(10);

    CameraServer.getInstance().startAutomaticCapture();

    Shuffleboard.getTab("LiveWindow").add("Video Stream", SendableCameraWrapper.wrap(CameraServer.getInstance().putVideo("Cam", 6000, 6000)));

    //startCapture();
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
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    //NewworkTableEntry ledMode = table.getEntry("ledMode");

    //read values periodically
    // 0.1234 is the default value of the angle returned
    final double defaultDistanceSpeed = -0.1;
    double horizontalAngle = tx.getDouble(0.1234);
    double verticalAngle = ty.getDouble(0.1234);
    double s = ts.getDouble(0.1234);
    double area = ta.getDouble(0.1234);
    
    int tshortVal = (int)tshort.getDouble(0.1234);
    int tlongVal = (int)tlong.getDouble(0.1234);
    int thorVal = (int)thor.getDouble(0.1234);
    int tvertVal = (int)tvert.getDouble(0.1234);

    double leftMovement = 0.0;
    double rightMovement = 0.0;
    double distance = 0;
    double distanceSpeed = 0;
    double automatedBaseSpeed = 0.05;

    horizontalSpeed = 0;
    distanceSpeed = 0;

    // checks if the A button is currently being pressed
    // getRawButton returns a boolean
	  if(stick.getRawButton(1)) {
      // reset the horizontal and distance speeds every time the code runs
      // this will prevent previous leftover values from moving the motors
      horizontalSpeed = 0;
      distanceSpeed = 0;

      // rotate the robot towards the target if horizontal angle is greater than 8 degrees on either side of the target
      if(Math.abs(horizontalAngle) > 8.0) {
        distanceSpeed = 0;
        rotate(horizontalAngle);
      }
      // drive straight if not rotating
      else {
        horizontalSpeed = 0;
        distanceSpeed = verticalAngle / 30.0;
      }

      // left and right speeds
      leftMovement = distanceSpeed + horizontalSpeed;
      rightMovement = distanceSpeed - horizontalSpeed;

      //leftMovement -= automatedBaseSpeed + horizontalSpeed + distanceSpeed;
      //rightMovement += automatedBaseSpeed + horizontalSpeed + distanceSpeed;
    }

    // run the robot
    //runAt(leftMovement, -rightMovement);

    // 
    distance = HEIGHT / Math.tan(degreeToRadian(verticalAngle) + INITIAL_ANGLE);

    List<ShuffleboardComponent<?>> components =  Shuffleboard.getTab("SmartDashboard").getComponents();
    System.out.println(components);

    //System.out.println("********************************************************");
    //System.out.println("Distance: " + distance);
    //System.out.println("ty: " + verticalAngle);
    //System.out.println("Initial Angle: " + INITIAL_ANGLE);
    //System.out.println("distance = " + HEIGHT + "/" + Math.tan(degreeToRadian(verticalAngle) + INITIAL_ANGLE));
    //System.out.println("thor: " + thorVal);
    //System.out.println("tvert: " + tvertVal);
    //System.out.println("tvert / thor" + (tvertVal / thorVal));
    //System.out.println("********************************************************");
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

   // drive the robot
   public void runAt(double leftSpeed, double rightSpeed) {
        
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
   }


   // rotate the robot based on horizontal angle offset
   public void rotate(double xAngle) {
      if(xAngle > 5.0) {
        horizontalSpeed = defaultHorizontalSpeed * (xAngle / 2) - minCommand;
      }
      else if(xAngle < -5.0) {
        horizontalSpeed = defaultHorizontalSpeed * (xAngle / 2) + minCommand;
      }
   }

   public static void startCapture() {
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(320, 240);
			camera.setFPS(10);
		}).start();
	}
}