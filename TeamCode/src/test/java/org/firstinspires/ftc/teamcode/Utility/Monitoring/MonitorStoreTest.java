package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import java.io.File;
import java.io.Writer;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Queue;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import static org.junit.Assert.*;

public class MonitorStoreTest {
    @Rule public final TemporaryFolder temporary = new TemporaryFolder();

    private MonitorStore store(File dir) {
        return store(dir, null);
    }

    private MonitorStore store(File dir, List<Runnable> finalTasks) {
        ScheduledExecutorService scheduler = (ScheduledExecutorService) Proxy.newProxyInstance(
                ScheduledExecutorService.class.getClassLoader(), new Class<?>[] {ScheduledExecutorService.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("scheduleWithFixedDelay")) return null; // manually driven
                    if (method.getName().equals("execute")) {
                        if (finalTasks == null) ((Runnable) args[0]).run();
                        else finalTasks.add((Runnable) args[0]);
                        return null;
                    }
                    if (method.getName().equals("shutdown")) return null;
                    throw new AssertionError(method);
                });
        return new MonitorStore(dir, null, 100000, 1000, new MonitorStore.Provider() {
            public String summaryJson(boolean complete) { return "{\"complete\":" + complete + "}"; }
            public String indexRow() { return null; } // this test covers the session writer, not global index
        }, scheduler);
    }

    @Test public void partialOpenFailureCanBeRetried() throws Exception {
        File dir = temporary.newFolder();
        File blocked = new File(dir, "system.csv");
        assertTrue(blocked.mkdir());
        MonitorStore store = store(dir);
        try {
            try {
                open(store);
                fail("system.csv directory must cause an open failure");
            } catch (java.io.IOException expected) {
                assertNull("failed open must not publish a partially initialized writer", field(store, "devicesOut"));
            }
            assertTrue(blocked.delete());
            open(store);
            assertNotNull(field(store, "systemOut"));
            assertNotNull(field(store, "motorsOut"));
        } finally {
            closeWriters(store);
        }
    }

    @Test public void stalledWriterBacklogIsBoundedAndLateRowsAreRejected() throws Exception {
        File dir = temporary.newFolder();
        MonitorStore store = store(dir);
        for (int i = 0; i < 20000; i++) {
            store.addSystemRow("row" + i);
            store.addMotorRow("motor" + i);
            store.addDevice(0, "motor", "device" + i);
        }
        Queue<?> rows = (Queue<?>) field(store, "systemRows");
        assertEquals(MonitorStore.MAX_PENDING_ROWS, rows.size());
        assertEquals(MonitorStore.MAX_PENDING_ROWS, ((Queue<?>) field(store, "motorRows")).size());
        assertEquals(MonitorStore.MAX_PENDING_ROWS, ((Queue<?>) field(store, "deviceRows")).size());
        assertEquals(3L * (20000 - MonitorStore.MAX_PENDING_ROWS), store.droppedRows());
        store.setSystemHeader("sample");
        store.finish();
        byte[] before = Files.readAllBytes(new File(dir, "system.csv").toPath());
        store.addSystemRow("late");
        store.addMotorRow("late");
        store.finish();
        assertTrue(rows.isEmpty());
        assertArrayEquals(before, Files.readAllBytes(new File(dir, "system.csv").toPath()));
        assertTrue(new String(before, StandardCharsets.UTF_8).startsWith("sample\nrow0\n"));
    }

    @Test public void finishingStopsAdmissionBeforeTheWriterActuallyCloses() throws Exception {
        List<Runnable> tasks = new ArrayList<>();
        File dir = temporary.newFolder();
        MonitorStore store = store(dir, tasks);
        store.setSystemHeader("sample");
        store.addSystemRow("before");
        store.finish();
        store.addSystemRow("after");
        store.addMotorRow("after");
        store.addDevice(0, "after", "after");
        store.finish();
        assertEquals(1, tasks.size());
        assertEquals(1, ((Queue<?>) field(store, "systemRows")).size());
        assertTrue(((Queue<?>) field(store, "motorRows")).isEmpty());
        assertTrue(((Queue<?>) field(store, "deviceRows")).isEmpty());
        tasks.get(0).run();
        assertEquals("sample\nbefore\n", new String(Files.readAllBytes(
                new File(dir, "system.csv").toPath()), StandardCharsets.UTF_8));
    }

    private static Object field(Object object, String name) throws Exception {
        Field f = MonitorStore.class.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(object);
    }

    private static void open(MonitorStore store) throws Exception {
        Method method = MonitorStore.class.getDeclaredMethod("openIfNeeded");
        method.setAccessible(true);
        try { method.invoke(store); }
        catch (InvocationTargetException e) { throw (Exception) e.getCause(); }
    }

    private static void closeWriters(MonitorStore store) throws Exception {
        for (String name : new String[] {"devicesOut", "systemOut", "motorsOut", "eventsOut"}) {
            Writer writer = (Writer) field(store, name);
            if (writer != null) writer.close();
        }
    }
}
