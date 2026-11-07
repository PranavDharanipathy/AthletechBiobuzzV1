package org.firstinspires.ftc.teamcode.Utility.TemporalLog;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.IdentityHashMap;

/// every variable of an object graph, logged when it changes. Built once by reflection (and grown when a field that was null
/// gets an object), then each sample re-reads the chain of fields from the root, so new objects put in a field (a Pose made
/// each loop) are followed. Primitives, enums, boxed numbers and small primitive arrays are logged; objects of our code,
/// Pedro and NextFTC are walked into; the SDK, Java and the logging/monitoring internals aren't. Allocates only when it grows
final class VariableWatcher {

    private static final int MAX_DEPTH = 7, MAX_ARRAY = 64, MAX_LEAVES = 12000;
    private static final int D = 0, F = 1, I = 2, J = 3, S = 4, B = 5, Z = 6, C = 7, ENUM = 8, BOXED = 9, ARR_D = 10, ARR_F = 11, ARR_I = 12, ARR_J = 13, ARR_Z = 14;

    //nodes: an object found by following field from the parent node's object (static field when parent < 0)
    private final ArrayList<Field> nodeField = new ArrayList<>();
    private final ArrayList<Class<?>> nodeClass = new ArrayList<>();
    private int[] nodeParent = new int[256], nodeDepth = new int[256];
    private boolean[] nodePending = new boolean[256];
    private String[] nodePath = new String[256];
    private Object[] cur = new Object[256];
    private int nodes;

    //leaves: a value read from a node's object
    private final ArrayList<Field> leafField = new ArrayList<>();
    private int[] leafNode = new int[1024], leafType = new int[1024], leafIndex = new int[1024], leafKey = new int[1024];
    private long[] leafLast = new long[1024];
    private int leaves;

    private final IdentityHashMap<Object, Boolean> seen = new IdentityHashMap<>();

    int leafCount() {
        return leaves;
    }

    void addRoot(String name, Object root) {

        int n = node(-1, null, root.getClass(), name, 0);
        cur[n] = root;
        seen.put(root, Boolean.TRUE);
        expand(n, root.getClass(), false);
    }

    void addStatics(Class<?> c) {

        int n = node(-1, null, c, c.getSimpleName(), 0);
        cur[n] = c;
        expand(n, c, true);
    }

    private int node(int parent, Field f, Class<?> cls, String path, int depth) {

        if (nodes == cur.length) {
            int size = nodes * 2;
            nodeParent = java.util.Arrays.copyOf(nodeParent, size);
            nodeDepth = java.util.Arrays.copyOf(nodeDepth, size);
            nodePending = java.util.Arrays.copyOf(nodePending, size);
            nodePath = java.util.Arrays.copyOf(nodePath, size);
            cur = java.util.Arrays.copyOf(cur, size);
        }
        nodeField.add(f);
        nodeClass.add(cls);
        nodeParent[nodes] = parent;
        nodeDepth[nodes] = depth;
        nodePath[nodes] = path;
        return nodes++;
    }

    private void leaf(int node, Field f, int type, int index, String name, String labels) {

        if (leaves >= MAX_LEAVES) return;
        if (leaves == leafNode.length) {
            int size = leaves * 2;
            leafNode = java.util.Arrays.copyOf(leafNode, size);
            leafType = java.util.Arrays.copyOf(leafType, size);
            leafIndex = java.util.Arrays.copyOf(leafIndex, size);
            leafKey = java.util.Arrays.copyOf(leafKey, size);
            leafLast = java.util.Arrays.copyOf(leafLast, size);
        }
        leafField.add(f);
        leafNode[leaves] = node;
        leafType[leaves] = type;
        leafIndex[leaves] = index;
        leafKey[leaves] = TemporalLog.key("var", name, labels);
        leafLast[leaves] = 0x7ff8dead0000beefL;
        leaves++;
    }

    //fields of an object (or the static fields of a class) into leaves and child nodes
    private void expand(int n, Class<?> cls, boolean statics) {

        String path = nodePath[n];
        int depth = nodeDepth[n];
        if (cls.isArray()) {
            arrayLeaves(n, cls, path);
            return;
        }
        java.util.HashSet<String> used = new java.util.HashSet<>();
        for (Class<?> c = cls; c != null && walkable(c, statics); c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                boolean isStatic = Modifier.isStatic(f.getModifiers());
                if (isStatic != statics || f.isSynthetic() || f.getName().startsWith("$")) continue;
                try {
                    f.setAccessible(true);
                } catch (Throwable t) {
                    continue;
                }
                //a superclass field hidden by a subclass one of the same name gets its class in the name
                String name = path + "." + (used.add(f.getName()) ? f.getName() : f.getName() + "@" + c.getSimpleName());
                Class<?> t = f.getType();
                int type = primitive(t);
                if (type >= 0) leaf(n, f, type, 0, name, t == boolean.class ? "0=false|1=true" : "");
                else if (t.isEnum()) leaf(n, f, ENUM, 0, name, TemporalLog.labels(t));
                else if (t == Double.class || t == Float.class || t == Integer.class || t == Long.class || t == Short.class || t == Byte.class) leaf(n, f, BOXED, 0, name, "");
                else if (t == Boolean.class) leaf(n, f, BOXED, 0, name, "0=false|1=true");
                else if (depth < MAX_DEPTH && (t.isArray() ? arrayType(t) >= 0 : !t.isPrimitive())) {
                    int child = node(n, f, null, name, depth + 1);
                    nodePending[child] = true;
                }
            }
            if (statics) break;
        }
    }

    private void arrayLeaves(int n, Class<?> cls, String path) {

        Object a = cur[n];
        int type = arrayType(cls);
        int len = a == null ? 0 : Math.min(MAX_ARRAY, Array.getLength(a));
        for (int k = 0; k < len; k++) leaf(n, null, type, k, path + "[" + k + "]", type == ARR_Z ? "0=false|1=true" : "");
    }

    //our code, Pedro and NextFTC; not the SDK, Java, Kotlin, or the logging/monitoring internals (huge, and logged elsewhere)
    private static boolean walkable(Class<?> c, boolean statics) {

        if (statics) return true;
        String name = c.getName();
        if (name.startsWith("org.firstinspires.ftc.teamcode.")) {
            return !name.contains(".Utility.Monitoring.") && !name.contains(".Utility.TemporalLog.") && !name.contains(".Utility.Blaze.BlazeRing") && !name.endsWith("DeviceStats") && !name.endsWith("DeviceTracker") && !name.endsWith("CacheSlot") && !name.endsWith("CsvLogger") && !name.endsWith("RobotLogger");
        }
        return name.startsWith("com.pedropathing.") || name.startsWith("dev.nextftc.");
    }

    private static int primitive(Class<?> t) {

        if (t == double.class) return D;
        if (t == float.class) return F;
        if (t == int.class) return I;
        if (t == long.class) return J;
        if (t == short.class || t == byte.class) return S;
        if (t == boolean.class) return Z;
        if (t == char.class) return C;
        return -1;
    }

    private static int arrayType(Class<?> t) {

        Class<?> e = t.getComponentType();
        if (e == double.class) return ARR_D;
        if (e == float.class) return ARR_F;
        if (e == int.class || e == short.class || e == byte.class) return ARR_I;
        if (e == long.class) return ARR_J;
        if (e == boolean.class) return ARR_Z;
        return e != null && !e.isPrimitive() ? -2 : -1;
    }

    void sample(long now) {

        //resolve every node's object for this sample (parents come before children)
        for (int n = 0; n < nodes; n++) {
            int p = nodeParent[n];
            if (p < 0) continue;
            Object parent = cur[p];
            Object o = null;
            if (parent != null && nodeClass.get(p) != null) {
                try {
                    o = nodeField.get(n).get(parent instanceof Class && nodeParent[p] < 0 && nodeField.get(p) == null ? null : parent);
                } catch (Throwable t) {
                    o = null;
                }
            }
            if (o != null && nodePending[n]) grow(n, o);
            else if (o != null && o.getClass() != nodeClass.get(n)) o = null;
            cur[n] = o;
        }
        for (int k = 0; k < leaves; k++) {
            double v = read(k);
            long bits = Double.doubleToRawLongBits(v);
            if (bits == leafLast[k]) continue;
            leafLast[k] = bits;
            TemporalLog.record(now, leafKey[k], 0, v);
        }
    }

    //a field that was null (or not looked at) now holds an object: walk it the first time
    private void grow(int n, Object o) {

        nodePending[n] = false;
        Class<?> cls = o.getClass();
        nodeClass.set(n, cls);
        boolean array = cls.isArray();
        if (array ? arrayType(cls) < 0 : !walkable(cls, false)) {
            //not ours to walk: boxed numbers in an Object field still count
            if (o instanceof Number || o instanceof Boolean) leaf(n, null, BOXED, -1, nodePath[n], o instanceof Boolean ? "0=false|1=true" : "");
            return;
        }
        if (!array && seen.put(o, Boolean.TRUE) != null) return;
        cur[n] = o;
        expand(n, cls, false);
    }

    private double read(int k) {

        Object o = cur[leafNode[k]];
        if (o == null) return Double.NaN;
        boolean statics = o instanceof Class && nodeParent[leafNode[k]] < 0;
        Object target = statics ? null : o;
        Field f = leafField.get(k);
        try {
            switch (leafType[k]) {
                case D:
                    return f.getDouble(target);
                case F:
                    return f.getFloat(target);
                case I:
                    return f.getInt(target);
                case J:
                    return f.getLong(target);
                case S:
                    return f.getInt(target);
                case C:
                    return f.getChar(target);
                case Z:
                    return f.getBoolean(target) ? 1 : 0;
                case ENUM: {
                    Object e = f.get(target);
                    return e == null ? Double.NaN : ((Enum<?>) e).ordinal();
                }
                case BOXED: {
                    Object b = leafIndex[k] < 0 ? o : f.get(target);
                    if (b instanceof Number) return ((Number) b).doubleValue();
                    if (b instanceof Boolean) return (Boolean) b ? 1 : 0;
                    return Double.NaN;
                }
                case ARR_D:
                    return leafIndex[k] < Array.getLength(o) ? Array.getDouble(o, leafIndex[k]) : Double.NaN;
                case ARR_F:
                    return leafIndex[k] < Array.getLength(o) ? Array.getFloat(o, leafIndex[k]) : Double.NaN;
                case ARR_I:
                    return leafIndex[k] < Array.getLength(o) ? Array.getInt(o, leafIndex[k]) : Double.NaN;
                case ARR_J:
                    return leafIndex[k] < Array.getLength(o) ? Array.getLong(o, leafIndex[k]) : Double.NaN;
                case ARR_Z:
                    return leafIndex[k] < Array.getLength(o) ? (Array.getBoolean(o, leafIndex[k]) ? 1 : 0) : Double.NaN;
                default:
                    return Double.NaN;
            }
        } catch (Throwable t) {
            return Double.NaN;
        }
    }
}
