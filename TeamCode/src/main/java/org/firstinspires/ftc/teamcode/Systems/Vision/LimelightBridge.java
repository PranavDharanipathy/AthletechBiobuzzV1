package org.firstinspires.ftc.teamcode.Systems.Vision;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.Channel;
import java.nio.channels.ClosedSelectorException;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public final class LimelightBridge {

    private static final int BUFFER_BYTES = 64 * 1024;

    private final String host;
    private final int firstListenPort, firstTargetPort, portCount, maxConnections;
    private final List<ServerSocketChannel> servers = new ArrayList<>();
    private Selector selector;
    private Thread thread;
    private volatile boolean running;
    private volatile int openPorts, connections;
    private volatile long bytes;
    private volatile String lastError = "";

    public LimelightBridge(String host, int firstListenPort, int firstTargetPort, int portCount, int maxConnections) {

        this.host = host;
        this.firstListenPort = firstListenPort;
        this.firstTargetPort = firstTargetPort;
        this.portCount = portCount;
        this.maxConnections = maxConnections;
    }

    public synchronized void start() {

        if (running) return;
        try {
            selector = Selector.open();
        }
        catch (IOException e) {
            lastError = "selector: " + e.getMessage();
            return;
        }
        int opened = 0;
        for (int i = 0; i < portCount; i++) {
            try {
                ServerSocketChannel server = ServerSocketChannel.open();
                servers.add(server);
                server.socket().setReuseAddress(true);
                server.socket().bind(new InetSocketAddress(firstListenPort + i));
                server.configureBlocking(false);
                server.register(selector, SelectionKey.OP_ACCEPT, firstTargetPort + i);
                opened++;
            }
            catch (IOException e) {
                lastError = "port " + (firstListenPort + i) + ": " + e.getMessage();
            }
        }
        openPorts = opened;
        running = true;
        thread = new Thread(this::loop, "LimelightBridge");
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
    }

    public void stop() {

        Thread t;
        Selector s;
        synchronized (this) {
            if (!running) return;
            running = false;
            t = thread;
            s = selector;
            thread = null;
        }
        s.wakeup();
        try {
            t.join(1000);
        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean isRunning() {
        return running;
    }

    public String host() {
        return host;
    }

    public int openPorts() {
        return openPorts;
    }

    public int connections() {
        return connections;
    }

    public long bytes() {
        return bytes;
    }

    public String lastError() {
        return lastError;
    }

    private void loop() {

        try {
            while (running) {
                selector.select();
                Iterator<SelectionKey> keys = selector.selectedKeys().iterator();
                while (keys.hasNext()) {
                    SelectionKey key = keys.next();
                    keys.remove();
                    handle(key);
                }
            }
        }
        catch (IOException | ClosedSelectorException e) {
            lastError = "bridge: " + e.getMessage();
        }
        finally {
            closeAll();
        }
    }

    private void handle(SelectionKey key) {

        if (!key.isValid()) return;
        Object attachment = key.attachment();
        if (attachment instanceof Integer) {
            accept((ServerSocketChannel) key.channel(), (Integer) attachment);
            return;
        }
        End end = (End) attachment;
        try {
            if (key.isConnectable()) end.connect();
            if (key.isValid() && key.isReadable()) end.read();
            if (key.isValid() && key.isWritable()) end.write();
            end.link.refresh();
        }
        catch (IOException e) {
            end.link.close();
        }
    }

    private void accept(ServerSocketChannel server, int targetPort) {

        SocketChannel client = null;
        SocketChannel upstream = null;
        try {
            client = server.accept();
            if (client == null) return;
            if (connections >= maxConnections) {
                closeQuietly(client);
                return;
            }
            client.configureBlocking(false);
            client.socket().setTcpNoDelay(true);
            upstream = SocketChannel.open();
            upstream.configureBlocking(false);
            upstream.socket().setTcpNoDelay(true);
            Link link = new Link(client, upstream);
            link.connected = upstream.connect(new InetSocketAddress(host, targetPort));
            link.client.key = client.register(selector, 0, link.client);
            link.upstream.key = upstream.register(selector, 0, link.upstream);
            connections++;
            link.refresh();
        }
        catch (IOException e) {
            lastError = "connect " + targetPort + ": " + e.getMessage();
            closeQuietly(client);
            closeQuietly(upstream);
        }
    }

    private void closeAll() {

        List<SelectionKey> keys = new ArrayList<>(selector.keys());
        for (SelectionKey key : keys) {
            if (key.attachment() instanceof End) ((End) key.attachment()).link.close();
        }
        for (ServerSocketChannel server : servers) closeQuietly(server);
        servers.clear();
        closeQuietly(selector);
        openPorts = 0;
        connections = 0;
    }

    private static void closeQuietly(Channel channel) {

        if (channel == null) return;
        try {
            channel.close();
        }
        catch (IOException ignored) {
        }
    }

    private static void closeQuietly(Selector selector) {

        try {
            selector.close();
        }
        catch (IOException ignored) {
        }
    }

    private final class Link {

        final End client, upstream;
        boolean connected, closed;

        Link(SocketChannel clientChannel, SocketChannel upstreamChannel) {

            client = new End(this, clientChannel);
            upstream = new End(this, upstreamChannel);
            client.peer = upstream;
            upstream.peer = client;
        }

        void refresh() throws IOException {

            if (closed) return;
            client.shutPeerIfDone();
            upstream.shutPeerIfDone();
            if (client.outputShut && upstream.outputShut) {
                close();
                return;
            }
            client.key.interestOps(client.interest());
            upstream.key.interestOps(upstream.interest());
        }

        void close() {

            if (closed) return;
            closed = true;
            connections--;
            closeQuietly(client.channel);
            closeQuietly(upstream.channel);
        }
    }

    private final class End {

        final Link link;
        final SocketChannel channel;
        final ByteBuffer in = ByteBuffer.allocate(BUFFER_BYTES);
        End peer;
        SelectionKey key;
        boolean eof, outputShut;

        End(Link link, SocketChannel channel) {

            this.link = link;
            this.channel = channel;
        }

        void connect() throws IOException {
            if (channel.finishConnect()) link.connected = true;
        }

        void read() throws IOException {

            int n = channel.read(in);
            if (n < 0) eof = true;
            else bytes += n;
        }

        void write() throws IOException {

            ByteBuffer pending = peer.in;
            pending.flip();
            channel.write(pending);
            pending.compact();
        }

        void shutPeerIfDone() throws IOException {

            if (!eof || in.position() > 0 || peer.outputShut) return;
            peer.channel.socket().shutdownOutput();
            peer.outputShut = true;
        }

        int interest() {

            if (!link.connected) return this == link.upstream ? SelectionKey.OP_CONNECT : 0;
            int ops = 0;
            if (!eof && in.hasRemaining()) ops |= SelectionKey.OP_READ;
            if (peer.in.position() > 0 && !outputShut) ops |= SelectionKey.OP_WRITE;
            return ops;
        }
    }
}
