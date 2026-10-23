package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm

import com.pedropathing.math.Pose
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class SectorPrism(

    pose: Pose,
    private val xMin: Double,
    private val xMax: Double,
    private val pivotY: Double,
    private val pivotZ: Double,
    private val length: Double,
    private val angleLo: Double,
    angleHi: Double,
    private val clearance: Double,
) : ConvexShape {
    private val px = pose.x()
    private val py = pose.y()
    private val left = Vec3(-sin(pose.heading()), cos(pose.heading()), 0.0)
    private val forward = Vec3(cos(pose.heading()), sin(pose.heading()), 0.0)

    private val span = angleHi - angleLo
    private val startF = length * cos(angleLo)
    private val startZ = length * sin(angleLo)
    private val endF = length * cos(angleHi)
    private val endZ = length * sin(angleHi)

    override fun support(d: Vec3): Vec3 {
        val dl = d dot left
        val df = d dot forward
        val dz = d.z

        val x = if (dl >= 0.0) xMax else xMin

        var bestF = 0.0
        var bestZ = 0.0
        var bestScore = 0.0
        val n = hypot(df, dz)
        if (n > 1e-12) {
            val scoreStart = df * startF + dz * startZ
            if (scoreStart > bestScore) {
                bestScore = scoreStart
                bestF = startF
                bestZ = startZ
            }
            val scoreEnd = df * endF + dz * endZ
            if (scoreEnd > bestScore) {
                bestScore = scoreEnd
                bestF = endF
                bestZ = endZ
            }
            var rel = (atan2(dz, df) - angleLo) % (2 * PI)
            if (rel < 0.0) rel += 2 * PI
            if (rel <= span) {
                val scoreFacing = length * n
                if (scoreFacing > bestScore) {
                    bestF = length * df / n
                    bestZ = length * dz / n
                }
            }
        }

        val lf = pivotY + bestF
        val lz = pivotZ + bestZ
        val point = Vec3(
            px + x * left.x + lf * forward.x,
            py + x * left.y + lf * forward.y,
            lz,
        )

        if (clearance > 0.0) {
            val len = d.length()
            if (len > 1e-12) return point + d * (clearance / len)
        }
        return point
    }
}
