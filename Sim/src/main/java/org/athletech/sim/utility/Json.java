package org.athletech.sim.utility;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Tiny JSON reader/writer. Objects become {@code LinkedHashMap<String,Object>}, arrays
 * {@code List<Object>}, numbers {@code Double}. Enough for HardwareMonitor's summary.json and the
 * profile export; not a general-purpose library.
 */
public final class Json {
    private Json() {
    }

    // ------------------------------------------------------------------ writing

    public static String write(Object v) {
        StringBuilder sb = new StringBuilder();
        write(sb, v, 0, true);
        return sb.toString();
    }

    @SuppressWarnings("unchecked")
    private static void write(StringBuilder sb, Object v, int indent, boolean pretty) {
        if (v == null) {
            sb.append("null");
        } else if (v instanceof String) {
            quote(sb, (String) v);
        } else if (v instanceof Number) {
            double d = ((Number) v).doubleValue();
            sb.append(Double.isFinite(d) ? Csv.fmt(d) : "null");
        } else if (v instanceof Boolean) {
            sb.append(v);
        } else if (v instanceof Map<?, ?>) {
            Map<?, ?> m = (Map<?, ?>) v;
            if (m.isEmpty()) {
                sb.append("{}");
                return;
            }
            sb.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> e : m.entrySet()) {
                if (!first) sb.append(',');
                first = false;
                if (pretty) newline(sb, indent + 1);
                quote(sb, String.valueOf(e.getKey()));
                sb.append(pretty ? ": " : ":");
                write(sb, e.getValue(), indent + 1, pretty);
            }
            if (pretty) newline(sb, indent);
            sb.append('}');
        } else if (v instanceof List<?>) {
            List<?> l = (List<?>) v;
            boolean flat = l.stream().allMatch(x -> x == null || x instanceof Number || x instanceof String || x instanceof Boolean);
            sb.append('[');
            for (int i = 0; i < l.size(); i++) {
                if (i > 0) sb.append(flat ? ", " : ",");
                if (pretty && !flat) newline(sb, indent + 1);
                write(sb, l.get(i), indent + 1, pretty && !flat);
            }
            if (pretty && !flat && !l.isEmpty()) newline(sb, indent);
            sb.append(']');
        } else if (v instanceof double[]) {
            List<Object> l = new ArrayList<>();
            for (double d : (double[]) v) l.add(d);
            write(sb, l, indent, pretty);
        } else {
            quote(sb, v.toString());
        }
    }

    private static void newline(StringBuilder sb, int indent) {
        sb.append('\n');
        for (int k = 0; k < indent; k++) sb.append("  ");
    }

    private static void quote(StringBuilder sb, String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"':
                    sb.append("\\\"");
                    break;
                case '\\':
                    sb.append("\\\\");
                    break;
                case '\n':
                    sb.append("\\n");
                    break;
                case '\r':
                    sb.append("\\r");
                    break;
                case '\t':
                    sb.append("\\t");
                    break;
                default:
                    if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
                    else sb.append(c);
            }
        }
        sb.append('"');
    }

    // ------------------------------------------------------------------ reading

    public static Object parse(String text) {
        Parser p = new Parser(text);
        p.ws();
        Object v = p.value();
        p.ws();
        if (p.i != text.length()) throw p.error("trailing characters");
        return v;
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> obj(Object o, String key) {
        Object v = get(o, key);
        return v instanceof Map<?, ?> ? (Map<String, Object>) v : new LinkedHashMap<String, Object>();
    }

    @SuppressWarnings("unchecked")
    public static List<Object> arr(Object o, String key) {
        Object v = get(o, key);
        return v instanceof List<?> ? (List<Object>) v : new ArrayList<Object>();
    }

    public static double num(Object o, String key) {
        Object v = get(o, key);
        return v instanceof Number ? ((Number) v).doubleValue() : Double.NaN;
    }

    public static String str(Object o, String key) {
        Object v = get(o, key);
        return v == null ? null : v.toString();
    }

    private static Object get(Object o, String key) {
        return o instanceof Map<?, ?> ? ((Map<?, ?>) o).get(key) : null;
    }

    private static final class Parser {
        final String s;
        int i;

        Parser(String s) {
            this.s = s;
        }

        IllegalArgumentException error(String what) {
            return new IllegalArgumentException("JSON " + what + " at character " + i);
        }

        void ws() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
        }

        Object value() {
            if (i >= s.length()) throw error("unexpected end");
            char c = s.charAt(i);
            switch (c) {
                case '{': return object();
                case '[': return array();
                case '"': return string();
                case 't': expect("true"); return Boolean.TRUE;
                case 'f': expect("false"); return Boolean.FALSE;
                case 'n': expect("null"); return null;
                default: return number();
            }
        }

        void expect(String word) {
            if (!s.startsWith(word, i)) throw error("expected " + word);
            i += word.length();
        }

        Map<String, Object> object() {
            Map<String, Object> m = new LinkedHashMap<>();
            i++;
            ws();
            if (s.charAt(i) == '}') {
                i++;
                return m;
            }
            while (true) {
                ws();
                String k = string();
                ws();
                if (s.charAt(i++) != ':') throw error("expected ':'");
                ws();
                m.put(k, value());
                ws();
                char c = s.charAt(i++);
                if (c == '}') return m;
                if (c != ',') throw error("expected ',' or '}'");
            }
        }

        List<Object> array() {
            List<Object> l = new ArrayList<>();
            i++;
            ws();
            if (s.charAt(i) == ']') {
                i++;
                return l;
            }
            while (true) {
                ws();
                l.add(value());
                ws();
                char c = s.charAt(i++);
                if (c == ']') return l;
                if (c != ',') throw error("expected ',' or ']'");
            }
        }

        String string() {
            if (s.charAt(i) != '"') throw error("expected string");
            i++;
            StringBuilder sb = new StringBuilder();
            while (true) {
                char c = s.charAt(i++);
                if (c == '"') return sb.toString();
                if (c == '\\') {
                    char e = s.charAt(i++);
                    switch (e) {
                        case 'n':
                            sb.append('\n');
                            break;
                        case 't':
                            sb.append('\t');
                            break;
                        case 'r':
                            sb.append('\r');
                            break;
                        case 'b':
                            sb.append('\b');
                            break;
                        case 'f':
                            sb.append('\f');
                            break;
                        case 'u': {
                            sb.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                            i += 4;
                            break;
                        }
                        default:
                            sb.append(e);
                            break;
                    }
                } else {
                    sb.append(c);
                }
            }
        }

        Double number() {
            int start = i;
            while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) i++;
            if (start == i) throw error("unexpected character '" + s.charAt(i) + "'");
            return Double.parseDouble(s.substring(start, i));
        }
    }
}
