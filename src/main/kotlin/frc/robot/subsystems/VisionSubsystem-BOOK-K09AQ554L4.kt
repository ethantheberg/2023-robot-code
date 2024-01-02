package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.DoubleEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.interfaces.Gyro
import frc.robot.VisionUtils
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem

class VisionSubsystem(swerve: SwerveSubsystem) : SubsystemBase() {
    var cyclesVisible = 0;

    val swerveSubsystem : SwerveSubsystem;

    val zFilter = LinearFilter.singlePoleIIR(0.1, 0.02)
    val xFilter = LinearFilter.singlePoleIIR(0.1, 0.02)
    //val zFilter = LinearFilter.movingAverage(5)

    var mostRecentZ = 0.0
    var mostRecentX = 0.0

    val ZSetpoint = NT("ZSetpoint")
    val ZOutput = NT("ZOutput")
    val ZMeasurement = NT("ZMeasurement")
    val ZUnfiltered = NT("ZUnfiltered")

    val ZP = NT("ZP");
    val ZI = NT("ZI");
    val ZD = NT("ZD");

    val lineupZPID = ProfiledPIDController(
        VisionConstants.lineupZP,
        VisionConstants.lineupZI,
        VisionConstants.lineupZD,
        VisionConstants.lineupZConstraints
    )


    val XSetpoint = NT("XSetpoint")
    val XMeasurement = NT("XMeasurement")
    val XOutput = NT("XOutput")

    val TTASetpoint = NT("TTASetpoint")
    val TTAMeasurement = NT("TTAMeasurement")
    val TTAOutput = NT("TTAOutput")

    fun NT(x: String): DoubleEntry {
        return NetworkTableInstance.getDefault().getTable("Vision").getDoubleTopic(x).getEntry(0.0)
    }

    fun calculateAprilTagTargets() {
        val targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Fiducials

        /*if (targets.size > 0) {
            mostRecentX = xFilter.calculate(targets[0]!!.robotPose_FieldSpace[0])
            ZUnfiltered.set(targets[0]!!.robotPose_FieldSpace[0])
            mostRecentZ = zFilter.calculate(targets[0]!!.robotPose_FieldSpace[2])
            ++cyclesVisible
        } else {
            cyclesVisible = 0
        }*/
    }
    init {
        swerveSubsystem = swerve;
        ZP.set(VisionConstants.lineupZP)
        ZI.set(VisionConstants.lineupZI)
        ZD.set(VisionConstants.lineupZD)
    }

    /*fun calculateXTargets() {
        val targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Retro
        val latestTarget = if (targets.isNotEmpty()) {
            targets[0]
        } else {
            null
        }

        mostRecentX = xFilter.calculate(
            if (latestTarget == null) {
                mostRecentX
            } else {
                latestTarget.tx
            }
        )
        println("Ending calculate targets...")
    }*/
    override fun periodic() {
        if(VisionUtils.getTV("")){

            ++cyclesVisible
        } else {
            cyclesVisible = 0;
        }
        if(cyclesVisible >= 20){
            //val targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Fiducials[0]?.robotPose_FieldSpace

            swerveSubsystem.odometry.addVisionMeasurement(VisionUtils.getBotPose2d(""), Timer.getFPGATimestamp())
        }
        calculateAprilTagTargets()

        lineupZPID.p = ZP.get();
        lineupZPID.i = ZI.get();
        lineupZPID.d = ZD.get();

        ZMeasurement.set(mostRecentZ)
        ZSetpoint.set(lineupZPID.goal.position)

        XMeasurement.set(mostRecentX)

    }
}
