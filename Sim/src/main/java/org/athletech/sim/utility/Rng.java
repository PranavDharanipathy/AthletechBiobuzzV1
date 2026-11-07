package org.athletech.sim.utility;

import java.util.SplittableRandom;

public final class Rng {
    private final SplittableRandom random;

    public Rng(long seed) {
        this.random = new SplittableRandom(seed);
    }

    public double uniform() { return random.nextDouble(); }
    public double uniform(double lo, double hi) { return lo + (hi - lo) * random.nextDouble(); }
    private double spare;
    private boolean hasSpare;

    public double gaussian() {

        if (hasSpare) {
            hasSpare = false;
            return spare;
        }
        double u, v, s;
        do {
            u = 2 * random.nextDouble() - 1;
            v = 2 * random.nextDouble() - 1;
            s = u * u + v * v;
        } while (s >= 1 || s == 0);
        double k = Math.sqrt(-2 * Math.log(s) / s);
        spare = v * k;
        hasSpare = true;
        return u * k;
    }

    public double gaussian(double mean, double std) { return mean + std * gaussian(); }
    public int nextInt(int bound) { return random.nextInt(bound); }

    public Rng split() { return new Rng(random.nextLong()); }
}
