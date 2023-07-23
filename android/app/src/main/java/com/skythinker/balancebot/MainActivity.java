package com.skythinker.balancebot;

import androidx.appcompat.app.AppCompatActivity;

import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;
import android.widget.EditText;

import java.util.Arrays;

public class MainActivity extends AppCompatActivity {

    final byte BT_FRAME_HEADER = (byte) 0xAA; //遥控数据帧头

    private CtrlView ctrlView = null; //主界面视图
    Bluetooth btCtrl = null, btUpper = null; //连接主控模块和图传模块的蓝牙控制器
    WifiClient wifiClient = null; //连接主控模块的WiFi控制器
    Thread thread = null; //数据处理线程，主要用于发送遥控数据
    boolean threadStopFlag = false; //线程停止标志，线程检测到此标志后会退出
    Handler handler = new Handler();
    SharedPreferences sharedPreferences = null;
    boolean isPairing = false; //是否正在进行连接配置

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //设置窗口全屏
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getSupportActionBar().setElevation(0);
        if(getSupportActionBar() != null)
            getSupportActionBar().hide();

        sharedPreferences = getSharedPreferences("settings",MODE_PRIVATE);

        ctrlView = new CtrlView(this); //创建主界面视图

        //实现用户界面输入事件处理
        ctrlView.setCallback(new CtrlView.Callback() {
            @Override
            public void onUserInput(String type, Object value) {
                //用户点击了设置按钮
                if(type.equals("click") && value.toString().equals("config")) {
                    //弹出列表选择对话框
                    showListDialog("请选择操作", new String[]{"切换为蓝牙模式", "切换为图传模式","进行连接配置","发送站立指令"}, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) { //用户选择了列表中的某一项
                            if(which == 3) { //用户选择了发送站立指令
                                if(ctrlView.isVideoMode) {
                                    wifiClient.sendBytes(new byte[]{(byte)0xAB});
                                }else{
                                    btCtrl.sendBytes(new byte[]{(byte)0xAB});
                                }
                                return;
                            }
                            //断开所有连接
                            if(btCtrl.isConnected)
                                btCtrl.disconnect();
                            if(btUpper.isConnected)
                                btUpper.disconnect();
                            wifiClient.setServerAddress(null,0);

                            if(which == 0) { //设置为蓝牙模式
                                ctrlView.isVideoMode = false;
                            }else if(which == 1){ //设置为图传模式
                                ctrlView.isVideoMode = true;
                            }else if(which == 2){ //触发连接配置
                                isPairing = true;
                                if(ctrlView.isVideoMode)
                                    btUpper.clearMacInfo();
                                else
                                    btCtrl.clearMacInfo();
                            }

                            //根据所选模式，重新连接至图传模块或主控模块
                            if(ctrlView.isVideoMode) {
                                handler.postDelayed(new Runnable() {
                                    @Override
                                    public void run() {
                                        btUpper.connectDevice(MainActivity.this, "NanoPI");
                                    }
                                }, 1000);
                            }else{
                                handler.postDelayed(new Runnable() {
                                    @Override
                                    public void run() {
                                        btCtrl.connectDevice(MainActivity.this, "BalanceBot");
                                    }
                                }, 1000);
                            }

                            sharedPreferences.edit().putBoolean("useVideoMode", which==1).apply(); //保存连接方式设置

                            ctrlView.setMessageText("正在扫描设备...");
                        }
                    });
                }
            }
        });

        //创建连接到图传模块的蓝牙控制器
        btUpper = new Bluetooth(this);
        btUpper.setUUID("4c9a0011-fb2e-432e-92ec-c6f5316b4689",
            "4c9a0013-fb2e-432e-92ec-c6f5316b4689",
            "4c9a0012-fb2e-432e-92ec-c6f5316b4689");
        btUpper.setCallback(new Bluetooth.Callback() {
            @Override
            public void onConnectStateChange(boolean isConnected) {
                Log.d("btUpper", "onConnectStateChange: " + isConnected);
                if(isConnected && btCtrl.isConnected) {
                    btCtrl.disconnect();
                }
                if(isConnected) {
                    if(isPairing) { //如果在配置模式，连接蓝牙后请求扫描WiFi
                        ctrlView.setMessageText("蓝牙已连接，正在扫描可以连接的WiFi...");
                        btUpper.sendBytes("SCAN_WIFI".getBytes());
                    }else { //否则连接蓝牙后请求当前图传IP地址
                        ctrlView.setMessageText("蓝牙已连接，正在获取IP地址...");
                        btUpper.sendBytes("GET_IP".getBytes());
                    }
                }
            }

            @Override
            public void onReceive(byte[] data) { //收到来自图传的蓝牙数据
                Log.d("btUpper", "onReceive: " + data.length);
                String cmdStr = new String(data);
                if(cmdStr.startsWith("WIFI_LIST:")) { //收到扫描到的WiFi列表
                    String[] wifiList = cmdStr.substring(10).split("\t");
                    Log.d("btUpper", "wifiList: " + Arrays.toString(wifiList));
                    showListDialog("请选择要连接的WiFi", wifiList, new DialogInterface.OnClickListener() { //请求用户选择要连接的WiFi
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            String wifiName = wifiList[which];
                            showInputDialog("请输入WiFi密码","无需密码请留空", new DialogInterface.OnClickListener() { //请求用户输入WiFi密码
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    EditText editText = ((AlertDialog)dialog).findViewById(android.R.id.edit);
                                    String password = editText.getText().toString();
                                    btUpper.sendBytes(("CONN_WIFI:" + wifiName + "\t" + password).getBytes()); //发送连接WiFi的指令
                                    Log.d("btUpper", "wifiList[which]: " + wifiName);
                                    ctrlView.setMessageText("正在连接到WiFi: " + wifiName + "...");
                                }
                            });
                        }
                    });
                }else if(cmdStr.startsWith("CONN_WIFI_OK")) { //图传模块报告WiFi连接成功
                    ctrlView.setMessageText("WiFi连接成功，正在获取IP地址...");
                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            btUpper.sendBytes("GET_IP".getBytes()); //延时1秒后请求图传IP地址
                        }
                    }, 1000);
                }else if(cmdStr.startsWith("CONN_WIFI_FAIL")) { //图传模块报告WiFi连接失败
                    ctrlView.setMessageText("WiFi连接失败，请重试");
                    btUpper.sendBytes("SCAN_WIFI".getBytes()); //重新扫描WiFi列表
                }else if(cmdStr.startsWith("SELF_IP:")) { //收到图传模块的IP地址
                    String ip = cmdStr.substring(8);
                    Log.d("btUpper", "ip: " + ip);
                    wifiClient.setServerAddress(ip, 5000); //设置图传UDP服务器地址
                    ctrlView.setServerIP(ip); //设置主视图获取的视频流目标IP地址
                    btUpper.disconnect(); //断开到图传模块的蓝牙连接
                    ctrlView.setMessageText("已找到设备，IP地址: " + ip);
                    if(isPairing) { //如果在配置模式，标记配置模式结束
                        isPairing = false;
                        showConfirmDialog("配网完成","请确保网络环境可达目标IP", null, null);
                    }
                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            ctrlView.setMessageText("");
                        }
                    }, 4000);
                }
            }
        });

        //创建连接到主控模块的蓝牙控制器
        btCtrl = new Bluetooth(this);
        btCtrl.setUUID("4c9a0001-fb2e-432e-92ec-c6f5316b4689",
                "4c9a0002-fb2e-432e-92ec-c6f5316b4689",
                "4c9a0003-fb2e-432e-92ec-c6f5316b4689");
        btCtrl.setCallback(new Bluetooth.Callback() {
            @Override
            public void onConnectStateChange(boolean isConnected) {
                Log.d("btCtrl", "onConnectStateChange: " + isConnected);
                if(isConnected && btUpper.isConnected) { //如果已连接到图传系统，则断开与主控模块的连接
                    btCtrl.disconnect();
                }
                if(isConnected && isPairing){ //在配置模式中，连接到主控模块后标记配对完成
                    isPairing = false;
                    showConfirmDialog("配对完成","已连接到设备", null, null);
                }
            }

            @Override
            public void onReceive(byte[] data) {
                Log.d("btCtrl", "onReceive: " + data.length);
            }
        });

        wifiClient = new WifiClient(15000); //创建连接到图传系统的UDP客户端

		//请求权限用于扫描蓝牙设备
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                requestPermissions(new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT}, 123);
            } else {
                requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 123);
            }
        } else {
            requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, 123);
        }

        //创建处理线程
        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    if(btCtrl.isConnected != ctrlView.bluetoothConnected) { //同步主视图的蓝牙连接状态
                        ctrlView.bluetoothConnected = btCtrl.isConnected;
                        ctrlView.invalidate();
                    }

                    //发送遥控数据
                    byte[] btData = new byte[]{
                            BT_FRAME_HEADER,
                            (byte) ((ctrlView.rockerValueX + 1) * 100),
                            (byte) ((ctrlView.rockerValueY + 1) * 100),
                            (byte) ((ctrlView.sliderValue + 1) * 100)
                    };
                    if(btCtrl.isConnected) { //如果已连接到主控模块，则发送数据到主控模块
                        btCtrl.sendBytes(btData);
                    }
                    else { //如果未连接到主控模块，则发送数据到图传系统
                        wifiClient.sendBytes(btData);
                    }

                    try {
                        Thread.sleep(100); //发送频率10Hz
                    } catch (InterruptedException ignored) { }
                    if(threadStopFlag)
                        break;
                }
            }
        });
        thread.start();

        setContentView(ctrlView);

        if(!sharedPreferences.getBoolean("useVideoMode", false)){ //根据配置文件决定使用蓝牙模式还是图传模式
            ctrlView.isVideoMode = false;
            btCtrl.connectDevice(MainActivity.this, "BalanceBot");
        }else{
            ctrlView.isVideoMode = true;
            btUpper.connectDevice(MainActivity.this, "NanoPI");
        }
        ctrlView.setMessageText("正在扫描设备...");
    }

    //弹出一个列表选择对话框
    private void showListDialog(String title, String[] items, DialogInterface.OnClickListener listener)
    {
        handler.post(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                builder.setTitle(title);
                builder.setItems(items, listener);
                builder.show();
            }
        });
    }

    //弹出一个输入对话框
    private void showInputDialog(String title, String hint, DialogInterface.OnClickListener listener)
    {
        handler.post(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                builder.setTitle(title);
                final EditText editText = new EditText(MainActivity.this);
                editText.setId(android.R.id.edit);
                editText.setHint(hint);
                builder.setView(editText);
                builder.setPositiveButton("确定", listener);
                builder.setNegativeButton("取消", null);
                builder.show();
            }
        });
    }

    //弹出一个确认对话框
    private void showConfirmDialog(String title, String message, DialogInterface.OnClickListener posListener, DialogInterface.OnClickListener negListener)
    {
        handler.post(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                builder.setTitle(title);
                builder.setMessage(message);
                builder.setPositiveButton("确定", posListener);
                builder.setNegativeButton("取消", negListener);
                builder.show();
            }
        });
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        ctrlView.setSize(ctrlView.getWidth(), ctrlView.getHeight());
        ctrlView.invalidate();
    }

    @Override
    protected void onDestroy() {
        btCtrl.disconnect();
        btUpper.disconnect();
        wifiClient.stop();
        threadStopFlag = true;
        super.onDestroy();
    }
}