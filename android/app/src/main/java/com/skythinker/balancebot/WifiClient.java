package com.skythinker.balancebot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public class WifiClient {

    public interface OnRecvListener {
        void onRecv(byte[] data, int length);
    } //UDP接收事件回调接口

    private int selfPort = 15000; //本地UDP端口
    private InetAddress serverAddress = null; //目标服务器地址
    private int serverPort = 5000; //目标服务器端口
    DatagramSocket socket = null;
    Thread recvThread = null; //轮询接收线程
    boolean threadStopFlag = false;

    public WifiClient(int port, OnRecvListener listener) {
        selfPort = port;
        try {
            socket = new DatagramSocket(selfPort); //创建UDP套接字
        } catch (SocketException ignored) { }

        recvThread = new Thread(new Runnable() { //数据接收线程
            @Override
            public void run() {
                while (true) {
                    if(socket != null) {
                        byte[] recvData = new byte[1024];
                        DatagramPacket recvPacket = new DatagramPacket(recvData, recvData.length);
                        try {
                            socket.receive(recvPacket); //阻塞接收
                            if(listener != null)
                                listener.onRecv(recvData, recvPacket.getLength()); //接收完成，触发回调函数
                        } catch (Exception ignored) { }
                    }
                    if(threadStopFlag)
                        break;
                }
            }
        });
        recvThread.start();
    }

    public WifiClient(int port) {
        this(port, null);
    }

    //设置目标服务器地址
    public void setServerAddress(String ip, int port) {
        if(ip == null || ip.isEmpty() || port <= 0 || port > 65535){
            serverAddress = null;
            serverPort = 0;
            return;
        }
        try {
            serverAddress = InetAddress.getByName(ip);
            serverPort = port;
        } catch (UnknownHostException ignored) { }
    }

    //通过UDP发送数据
    public void sendBytes(byte[] data) {
        if(socket != null && serverAddress != null) {
            try {
                socket.send(new DatagramPacket(data, data.length, serverAddress, serverPort));
            } catch (Exception ignored) { }
        }
    }

    public void stop() {
        if(socket != null) {
            socket.close();
            socket = null;
        }
        threadStopFlag = true;
    }
}
