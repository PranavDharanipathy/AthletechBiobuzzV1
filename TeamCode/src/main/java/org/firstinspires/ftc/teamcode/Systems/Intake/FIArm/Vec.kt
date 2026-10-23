package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm

import kotlin.math.sqrt

data class Vec2(val x: Double, val y: Double)

data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun unaryMinus() = Vec3(-x, -y, -z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)

    infix fun dot(o: Vec3): Double = x * o.x + y * o.y + z * o.z
    infix fun cross(o: Vec3) = Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x)

    fun lengthSq(): Double = this dot this
    fun length(): Double = sqrt(lengthSq())
}