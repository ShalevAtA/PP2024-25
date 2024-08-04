package frc.robot;

import Subsystem.Swerve;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Robot extends TimedRobot {
    Field2d field2d;
    Swerve swerve;
    XboxController xboxController;

    @Override
    public void robotInit() {
        field2d = new Field2d();
        Pose2d pose2d = new Pose2d(); // a 0,0 position
        field2d.setRobotPose(pose2d); // reset the field
        SmartDashboard.putData("field", field2d);

        swerve = SystemFactory.createSwerve();
        xboxController = new XboxController(0);

        JoystickButton A = new JoystickButton(xboxController, XboxController.Button.kA.value);
        A.whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        JoystickButton B = new JoystickButton(xboxController, XboxController.Button.kB.value);
        B.whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        JoystickButton X = new JoystickButton(xboxController, XboxController.Button.kX.value);
        X.whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        JoystickButton Y = new JoystickButton(xboxController, XboxController.Button.kY.value);
        Y.whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        PathPlannerLogging.setLogActivePathCallback((poses)-> {
            field2d.getObject("path").setPoses(poses);
        });
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("Angle", 0);
        //swerve.setSteerPosition(0);
    }
 
    @Override
    public void teleopPeriodic() {
        //double angle = SmartDashboard.getNumber("Angle", 0);
        //swerve.setSteerPosition(angle);
        //swerve.drive(0,5000,0);
        swerve.setDriveVelocity(3);
    }

    @Override
    public void teleopExit() {
        swerve.stop();
    }

    @Override
    public void autonomousInit() {
        ReplanningConfig replanningConfig = new ReplanningConfig(
                false,
                false);
        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(0.5,0,0.00007),
                new PIDConstants(0.5,0,0.00007),
                4.4169,
                RobotMap.CHASSIS_RADIUS,
                replanningConfig
        );
        PathPlannerPath pathS = PathPlannerPath.fromPathFile("New Path");
        FollowPathHolonomic pathHolonomic = new FollowPathHolonomic(
                pathS,
                swerve::getPose,
                swerve::getSpeeds,
                swerve::drive,
                holonomicPathFollowerConfig,
                ()-> {return false;},
                swerve);
        pathHolonomic.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        field2d.setRobotPose(swerve.getPose());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {
    }
}
