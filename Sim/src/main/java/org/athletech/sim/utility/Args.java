package org.athletech.sim.utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Command-line parsing: positional words plus {@code --name value} and {@code --flag} options. */
public final class Args {
    private final List<String> positional = new ArrayList<>();
    private final Map<String, String> options = new HashMap<>();
    private final Set<String> used = new HashSet<>();

    public Args(String[] argv) {
        for (int i = 0; i < argv.length; i++) {
            String a = argv[i];
            if (a.startsWith("--") && a.length() > 2) {
                String name = a.substring(2);
                int eq = name.indexOf('=');
                if (eq >= 0) {
                    options.put(name.substring(0, eq), name.substring(eq + 1));
                } else if (i + 1 < argv.length && !looksLikeOption(argv[i + 1])) {
                    options.put(name, argv[++i]);
                } else {
                    options.put(name, "true");
                }
            } else {
                positional.add(a);
            }
        }
    }

    private static boolean looksLikeOption(String s) {
        return s.startsWith("--") && s.length() > 2 && !Character.isDigit(s.charAt(2));
    }

    public List<String> positional() {
        return positional;
    }

    public String pos(int i) {
        return i < positional.size() ? positional.get(i) : null;
    }

    public boolean flag(String name) {
        used.add(name);
        String v = options.get(name);
        return v != null && !v.equalsIgnoreCase("false") && !v.equals("0");
    }

    public String str(String name, String fallback) {
        used.add(name);
        return options.getOrDefault(name, fallback);
    }

    public double num(String name, double fallback) {
        used.add(name);
        String v = options.get(name);
        if (v == null) return fallback;
        try {
            return Double.parseDouble(v);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("--" + name + " needs a number, got '" + v + "'");
        }
    }

    public int integer(String name, int fallback) {
        return (int) Math.round(num(name, fallback));
    }

    public boolean has(String name) {
        used.add(name);
        return options.containsKey(name);
    }

    /** Options given on the command line that no code asked for (likely typos). */
    public List<String> unused() {
        List<String> out = new ArrayList<>();
        for (String k : options.keySet()) if (!used.contains(k)) out.add("--" + k);
        return out;
    }
}
