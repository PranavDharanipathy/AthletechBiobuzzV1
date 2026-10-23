package org.firstinspires.ftc.teamcode.Systems.Field

import org.firstinspires.ftc.teamcode.Systems.Intake.FIArm.Obstacle
import org.firstinspires.ftc.teamcode.Systems.Intake.FIArm.Vec2

class FieldCollisionMap {

    private val border = ArrayList<Obstacle>()
    private val items = ArrayList<Obstacle>()

    val obstacles: List<Obstacle> get() = items.toList()

    fun setBorder(c1: Vec2, c2: Vec2, c3: Vec2, c4: Vec2): FieldCollisionMap {
        border.clear()
        border.addAll(Obstacle.fieldBorder(c1, c2, c3, c4))
        return this
    }

    fun setBorder(minX: Double, minY: Double, maxX: Double, maxY: Double): FieldCollisionMap =
        setBorder(Vec2(minX, minY), Vec2(maxX, minY), Vec2(maxX, maxY), Vec2(minX, maxY))

    fun add(obstacle: Obstacle): FieldCollisionMap {
        items.add(obstacle)
        return this
    }

    fun addCircle(x: Double, y: Double, radius: Double): FieldCollisionMap =
        add(Obstacle.circle(x, y, radius))

    fun addRectangle(a: Vec2, b: Vec2, c: Vec2, d: Vec2): FieldCollisionMap =
        add(Obstacle.rectangle(a, b, c, d))

    fun addWall(a: Vec2, b: Vec2): FieldCollisionMap = add(Obstacle.wall(a, b))

    fun remove(obstacle: Obstacle): Boolean = items.remove(obstacle)

    fun clearObstacles() {
        items.clear()
    }

    fun clearBorder() {
        border.clear()
    }

    fun all(): List<Obstacle> = border + items
}
