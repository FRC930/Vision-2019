/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  UsbCamera camera; 
  UsbCamera camera2;

  double stickX;
  double stickY;

  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);

  Joystick stick = new Joystick(0);

  @Override
  public void robotInit() {


    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);


    new Thread(() -> {
      // Instantiate the USB cameras and begin capturing their video streams
      camera = CameraServer.getInstance().startAutomaticCapture(0);
      camera2 = CameraServer.getInstance().startAutomaticCapture(1);
      // set the cameras' reolutions and FPS

      SmartDashboard.putNumber("Width Camera 1", 320);
      SmartDashboard.putNumber("Height Camera 1", 320);
      SmartDashboard.putNumber("Width Camera 2", 120);
      SmartDashboard.putNumber("Height Camera 2", 120);

			camera.setResolution(160, 120);
      camera.setFPS(30);

			camera2.setResolution(160, 120);
      camera2.setFPS(30);

    }).start();
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    //camera.setResolution((int)SmartDashboard.getNumber("Width Camera 1", 480),(int) SmartDashboard.getNumber("Height Camera 1", 320));
    //camera2.setResolution((int)SmartDashboard.getNumber("Width Camera 2", 480),(int) SmartDashboard.getNumber("Height Camera 2", 320));
    
    

    stickX = -Math.pow(stick.getRawAxis(1), 3);
    stickY = Math.pow(stick.getRawAxis(4), 3);



    // manual drive controls

    if(Math.abs(stickX) < 0.000124) {

      stickX =0;

    }

    if(Math.abs(stickY) < 0.000124) {

      stickY = 0;

    }



    // left and right speeds of the drivetrain

    stickX = stickX + stickY;

    stickY = stickX - stickY;



    // run the robot based on the left and right speeds of the drive train

    leftMotor.set(-stickX);
    rightMotor.set(stickY);

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
