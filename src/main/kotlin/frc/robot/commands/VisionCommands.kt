package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import frc.robot.VisionUtils
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.VisionSubsystem


fun SetPipeline(pipeline: VisionConstants.Pipelines): Command {
    return InstantCommand({
        VisionUtils.setPipelineIndex("", pipeline.ordinal)
    })
}

class RumbleCheck(val controller: XboxController, val check: () -> Boolean) : CommandBase() {
    override fun execute() {
        controller.setRumble(RumbleType.kBothRumble, (if (check()) 1.0 else 0.0));
    }

    override fun end(int2errupted: Boolean) {
        controller.setRumble(RumbleType.kBothRumble, 0.0)
    }

    override fun isFinished(): Boolean {
        return !check()
    }
}

class ZAlign(val driveSubsystem: SwerveSubsystem, val visionSubsystem: VisionSubsystem, val targetZ: Double) : CommandBase() {


    val CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false)

    init {
        addRequirements(driveSubsystem, visionSubsystem)
        CommandRunning.set(true)
        visionSubsystem.lineupZPID.setTolerance(0.1, 0.1)
        visionSubsystem.lineupZPID.reset(visionSubsystem.mostRecentZ)
        visionSubsystem.lineupZPID.setGoal(targetZ)
    }

    override fun execute() {
        val zOutput = visionSubsystem.lineupZPID.calculate(visionSubsystem.mostRecentZ)
        driveSubsystem.drive(zOutput/2, 0.0, 0.0, true, false)

        visionSubsystem.ZOutput.set(-zOutput)
    }

    override fun isFinished(): Boolean {
        return visionSubsystem.lineupZPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        CommandRunning.set(false)
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}
/* 
class ZOdometryAlign(val driveSubsystem: SwerveSubsystem, val targetDistance: Double) : CommandBase() {
    val lineupZPID = ProfiledPIDController(
        VisionConstants.lineupZP,
        VisionConstants.lineupZI,
        VisionConstants.lineupZD,
        VisionConstants.lineupZConstraints
    )
    val lineupRotPID = ProfiledPIDController(
        VisionConstants.lineupRotP,
        VisionConstants.lineupRotI,
        VisionConstants.lineupRotD,
        VisionConstants.lineupRotConstraints
    )

    val ZSetpoint = NT("ZSetpoint")
    val ZMeasurement = NT("ZMeasurement")
    val ZOutput = NT("ZOutput")

    val rotSetpoint = NT("rotSetpoint")
    val rotMeasurement = NT("rotMeasurement")
    val rotOutput = NT("rotOutput")

    val CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false)

    //non-null asserted, but might be null; if limelight doesn't see apriltag for a frame
    val initialDistance =  VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Fiducials[0]!!.robotPose_TargetSpace[2] // get from the limelight
    var currentPose = driveSubsystem.pose
    val desiredPose = currentPose.minus(Pose2d(Translation2d(currentPose.x, targetDistance - initialDistance),Rotation2d(0.0)))
    override fun initialize() {
        lineupZPID.setTolerance(0.1, 0.1)
        lineupZPID.reset(currentPose.y)
        lineupZPID.setGoal(desiredPose.y)
        lineupRotPID.setTolerance(0.1, 0.1)
        lineupRotPID.reset(currentPose.rotation.radians)
        lineupRotPID.setGoal(desiredPose.rotation.radians)
    }

    override fun execute() {
        currentPose = driveSubsystem.pose
        val zOutput = lineupZPID.calculate(desiredPose.y)
        val rotOutput = lineupZPID.calculate(desiredPose.rotation.radians)
        driveSubsystem.drive(-zOutput, 0.0, rotOutput, true, false)

        ZMeasurement.set(currentPose.y)
        ZOutput.set(-zOutput)
        ZSetpoint.set(lineupZPID.goal.position)

        rotMeasurement.set(currentPose.y)
        //rotOutput.set(rotOutput)
        rotSetpoint.set(lineupZPID.goal.position)
    }

    override fun isFinished(): Boolean {
        return lineupZPID.atGoal() && lineupRotPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        CommandRunning.set(false)
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}*/


class RetroreflectiveAlign(val driveSubsystem: SwerveSubsystem, val visionSubsystem: VisionSubsystem, val xTarget: Double) : CommandBase() {
    val lineupXPID = PIDController(
        VisionConstants.retroreflectiveP,
        VisionConstants.retroreflectiveI,
        VisionConstants.retroreflectiveD
    )


    val CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false)

    init {
        addRequirements(driveSubsystem, visionSubsystem)
    }

    // moving this into init may help
    override fun initialize() {
        CommandRunning.set(true)
        lineupXPID.setTolerance(0.5, 0.1)
    }



    override fun execute() {

        val xOutput = -lineupXPID.calculate(visionSubsystem.mostRecentX, xTarget)
        driveSubsystem.drive(0.0, xOutput, 0.0, true, false)

        visionSubsystem.XSetpoint.set(xTarget)
        visionSubsystem.XOutput.set(xOutput)

    }

    override fun isFinished(): Boolean {
        return lineupXPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        CommandRunning.set(false)
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}


// CCW is positive
class TurnToAngle(val driveSubsystem: SwerveSubsystem, val visionSubsystem: VisionSubsystem, val targetAngleRadians: Double) : CommandBase() {
    val ttaPID = ProfiledPIDController(VisionConstants.ttaP, VisionConstants.ttaI, VisionConstants.ttaD, VisionConstants.ttaConstraints)

    val heading get() = Rotation2d.fromRadians(driveSubsystem.heading)

    init {
        addRequirements(driveSubsystem)
    }

    override fun initialize() {
        ttaPID.setTolerance(0.05, 0.1)
        ttaPID.reset(heading.radians)
        ttaPID.enableContinuousInput(-Math.PI, Math.PI)
        ttaPID.setGoal(targetAngleRadians)
    }

    override fun execute() {
        val out = ttaPID.calculate(heading.radians)
        driveSubsystem.drive(0.0, 0.0, out, true, false)

        visionSubsystem.TTASetpoint.set(targetAngleRadians)
        visionSubsystem.TTAMeasurement.set(heading.radians)
        visionSubsystem.TTAOutput.set(out)
    }

    override fun isFinished(): Boolean {
        return ttaPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}

fun ChuteVision(driveSubsystem: SwerveSubsystem, visionSubsystem: VisionSubsystem, controller: XboxController): Command {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        return SequentialCommandGroup(
            SetPipeline(VisionConstants.Pipelines.APRILTAG),
            RumbleCheck(controller) { !VisionUtils.getTV("") },
            ZAlign(driveSubsystem, visionSubsystem,-2.39),
            TurnToAngle(driveSubsystem,visionSubsystem, -Math.PI / 2.0)
        )
    } else {
        return SequentialCommandGroup(
            SetPipeline(VisionConstants.Pipelines.APRILTAG),
            RumbleCheck(controller) { !VisionUtils.getTV("") },
            ZAlign(driveSubsystem,visionSubsystem, -2.39),
            TurnToAngle(driveSubsystem,visionSubsystem, Math.PI / 2.0)
        )
    }
}


fun CubeVision(driveSubsystem: SwerveSubsystem, visionSubsystem: VisionSubsystem, controller:XboxController): SequentialCommandGroup {
    return(SequentialCommandGroup(
        SetPipeline(VisionConstants.Pipelines.APRILTAG),
        RumbleCheck(controller) { !VisionUtils.getTV("") },
        ZAlign(driveSubsystem, visionSubsystem,-2.9)
    ))
}

fun RetroreflectiveVision(driveSubsystem: SwerveSubsystem, visionSubsystem: VisionSubsystem, controller: XboxController): SequentialCommandGroup {
    return (SequentialCommandGroup(
        SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
        RumbleCheck(controller) { !VisionUtils.getTV("") },
        TurnToAngle(driveSubsystem, visionSubsystem, Math.PI),
        RetroreflectiveAlign(driveSubsystem, visionSubsystem, 6.7)
    ))
}