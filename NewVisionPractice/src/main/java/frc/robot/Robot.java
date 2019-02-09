package frc.robot;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
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

  //----Object Declarations----\\
  Joystick stick = new Joystick(0);

  //----Variable Declarations----\\
  double horizontalSpeed = 0;
  double stickX = 0;
  double stickY = 0;

  //----Constants----\\
  public final double DEFAULT_HORIZONTAL_SPEED = -0.01;
  public final double DEFAULT_LIMELIGHT_VALUE = 0.1234;

  // Controller constants
  public final int A_BUTTON = 1;
  public final int Y_BUTTON = 4;
  public final int LEFT_Y_AXIS = 1;
  public final int RIGHT_X_AXIS = 4;

  // real-world measurements
  public final double DISTANCE_FROM_TARGET = 40.0;
  public final double CAM_HEIGHT = 14.375;
  public final double TARGET_HEIGHT = 27.5;
  public final double HEIGHT = TARGET_HEIGHT - CAM_HEIGHT;
  public final double INITIAL_ANGLE = Math.atan(HEIGHT/DISTANCE_FROM_TARGET);
  public final double ANGLE_ERROR = 0.45;

  // Spark motor controllers
  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);

  @Override
  public void robotInit() {
    /* set the limelight and USB camera to picture-in-picture mode,
       which means the limelight's camera feed is shown in the
       bottom right corner of the USB camera's feed
    */
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);

    // configure 2 motors on each side of the robot to follow the main motor on each side
    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);

    Shuffleboard.startRecording();

    /*
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(800, 600);
		camera.setFPS(30);
    */

    //CameraServer.getInstance().startAutomaticCapture();

    //Shuffleboard.getTab("LiveWindow").add("Video Stream", SendableCameraWrapper.wrap(CameraServer.getInstance().putVideo("Cam", 6000, 6000)));

    startCapture();
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

    // NetworkTable object declaration used to get values from the limelight network table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");


    //read values periodically, return 0.1234 as the default value if nothing is found
    double horizontalAngle = tx.getDouble(DEFAULT_LIMELIGHT_VALUE);
    double verticalAngle = ty.getDouble(DEFAULT_LIMELIGHT_VALUE);

    double leftMovement = 0.0;
    double rightMovement = 0.0;
    double distance = 0;
    double distanceSpeed = 0;

    horizontalSpeed = 0;
    distanceSpeed = 0;

    // checks if the A button is currently being pressed, returns a boolean
	  if(stick.getRawButton(A_BUTTON)) {
      // reset the horizontal and distance speeds every time the code runs
      // this will prevent previous leftover values from moving the motors
      horizontalSpeed = 0;
      distanceSpeed = 0;

      // if the A button is currently pressed, 
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1 + (2 * (stick.getRawButton(A_BUTTON) ? 1 : 0)));

      // rotate the robot towards the target if horizontal angle is greater than 8 degrees on either side of the target
      rotate(horizontalAngle, 3.0);
    }

    // Cubing values to create smoother function
    stickX = -Math.pow(stick.getRawAxis(LEFT_Y_AXIS), 3);
    stickY = Math.pow(stick.getRawAxis(RIGHT_X_AXIS), 3);

    // Joystick deadband
    if(Math.abs(stickX) > 0.000124) {
      distanceSpeed = stickX;
    }
    if(Math.abs(stickY) > 0.000124) {
      horizontalSpeed = stickY;
    }

    // left and right speeds
    leftMovement = distanceSpeed + horizontalSpeed;
    rightMovement = distanceSpeed - horizontalSpeed;

    // run the robot
    runAt(-leftMovement, rightMovement);
  }

  @Override
  public void testPeriodic() {
  }

  private static double degreeToRadian(double degree)
  {
      return Math.toRadians(degree);
  } 

   // drive the robot
   public void runAt(double leftSpeed, double rightSpeed) {
        
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
   }

   // rotate the robot based on horizontal angle offset
   public void rotate(double xAngle, double angleThreshold) {
      if(Math.abs(xAngle) > angleThreshold) 
        horizontalSpeed = DEFAULT_HORIZONTAL_SPEED * (-xAngle / 1.5);
   }

   public static void startCapture() {
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
			camera.setResolution(320, 240);
      camera.setFPS(30);
      UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
			camera2.setResolution(160, 120);
      camera2.setFPS(30);
      //camera.setVideoMode(VideoMode.PixelFormat.kRGB565, 320, 240, 30);
		}).start();
	}
}