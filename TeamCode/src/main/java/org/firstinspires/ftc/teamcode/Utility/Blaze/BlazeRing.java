package org.firstinspires.ftc.teamcode.Utility.Blaze;

public final class BlazeRing {

    private static final int SIZE = 4096, MASK = SIZE - 1;
    private final long[] time = new long[SIZE];
    private final double[] a = new double[SIZE], b = new double[SIZE], c = new double[SIZE], d = new double[SIZE], e = new double[SIZE], f = new double[SIZE];
    private volatile long head, tail;
    public volatile long dropped, total;

    public void push(long t, double a0, double b0, double c0, double d0, double e0, double f0) {

        long h = head;
        if (h - tail >= SIZE) {
            dropped++;
            return;
        }
        int k = (int) (h & MASK);
        time[k] = t;
        a[k] = a0;
        b[k] = b0;
        c[k] = c0;
        d[k] = d0;
        e[k] = e0;
        f[k] = f0;
        total++;
        head = h + 1;
    }

    public interface Sink {
        void take(long t, double a, double b, double c, double d, double e, double f);
    }

    public void drain(Sink sink) {

        long t0 = tail, h = head;
        for (long i = t0; i < h; i++) {
            int k = (int) (i & MASK);
            sink.take(time[k], a[k], b[k], c[k], d[k], e[k], f[k]);
        }
        tail = h;
    }

    public void clear() {
        tail = head;
    }
}
