package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm

interface ConvexShape {
    fun support(d: Vec3): Vec3
}

// GJK (Gilbert-Johnson-Keerthi) intersection test for two convex shapes.
// Touching counts as intersecting, and hitting the iteration cap (shapes separated
// by a vanishingly small gap) also counts as intersecting, so every failure mode
// errs toward "blocked".
object GJK {
    private const val EPS = 1e-12
    private const val MAX_ITERATIONS = 100

    @JvmStatic
    fun intersects(a: ConvexShape, b: ConvexShape): Boolean {
        var p = minkowski(a, b, Vec3(1.0, 0.0, 0.0))
        var dir = -p
        if (dir.lengthSq() < EPS) return true

        val simplex = ArrayList<Vec3>(4)
        simplex.add(p)

        var iterations = 0
        while (iterations < MAX_ITERATIONS) {
            iterations++
            p = minkowski(a, b, dir)
            //new point never reaches the origin, so the shapes are separated
            if ((p dot dir) < 0.0) return false

            simplex.add(0, p)
            val next = when (simplex.size) {
                2 -> line(simplex)
                3 -> triangle(simplex)
                else -> tetrahedron(simplex)
            }
            //null means the origin is enclosed (or sits on the boundary)
            if (next == null) return true
            dir = next
        }
        return true
    }

    //support point of the Minkowski difference A - B.
    private fun minkowski(a: ConvexShape, b: ConvexShape, d: Vec3): Vec3 =
        a.support(d) - b.support(-d)

    private fun nonZero(v: Vec3): Vec3? = if (v.lengthSq() < EPS) null else v

    private fun MutableList<Vec3>.keep(vararg points: Vec3) {
        clear()
        for (p in points) add(p)
    }

    //tuff stuff

    private fun line(s: MutableList<Vec3>): Vec3? {

        val a = s[0]
        val b = s[1]
        val ab = b - a
        val ao = -a
        if ((ab dot ao) > 0.0) {
            return nonZero((ab cross ao) cross ab)
        }
        s.keep(a)
        return nonZero(ao)
    }

    private fun triangle(s: MutableList<Vec3>): Vec3? {

        val a = s[0]
        val b = s[1]
        val c = s[2]
        val ab = b - a
        val ac = c - a
        val ao = -a
        val abc = ab cross ac
        // Degenerate triangle: treat as touching (conservative).
        if (abc.lengthSq() < EPS) return null

        if (((abc cross ac) dot ao) > 0.0) {
            if ((ac dot ao) > 0.0) {
                s.keep(a, c)
                return nonZero((ac cross ao) cross ac)
            }
            s.keep(a, b)
            return line(s)
        }
        if (((ab cross abc) dot ao) > 0.0) {
            s.keep(a, b)
            return line(s)
        }
        if ((abc dot ao) > 0.0) {
            return nonZero(abc)
        }
        s.keep(a, c, b)
        return nonZero(-abc)
    }

    private fun tetrahedron(s: MutableList<Vec3>): Vec3? {

        val a = s[0]
        val b = s[1]
        val c = s[2]
        val d = s[3]
        val ab = b - a
        val ac = c - a
        val ad = d - a
        val ao = -a
        val abc = ab cross ac
        val acd = ac cross ad
        val adb = ad cross ab

        if ((abc dot ao) > 0.0) {
            s.keep(a, b, c)
            return triangle(s)
        }
        if ((acd dot ao) > 0.0) {
            s.keep(a, c, d)
            return triangle(s)
        }
        if ((adb dot ao) > 0.0) {
            s.keep(a, d, b)
            return triangle(s)
        }
        return null
    }
}