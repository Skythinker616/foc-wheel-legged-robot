package com.skythinker.balancebot;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.ScanCallback;
import android.content.Context;
import android.content.SharedPreferences;
import android.os.Handler;
import android.util.Log;

import java.util.List;
import java.util.Set;
import java.util.UUID;

public class Bluetooth {

    public interface Callback {
        void onConnectStateChange(boolean isConnected);
        void onReceive(byte[] data);
    } //蓝牙事件回调接口

    //默认UUID，可通过setUUID方法修改
    String uuidService = "4c9a0001-fb2e-432e-92ec-c6f5316b4689";
    String uuidRxChara = "4c9a0002-fb2e-432e-92ec-c6f5316b4689";
    String uuidTxChara = "4c9a0003-fb2e-432e-92ec-c6f5316b4689";

    String btDeviceName = ""; //目标蓝牙设备名称
    BluetoothAdapter btAdapter = null;
    BluetoothGattCallback btCallback = null;
    BluetoothDevice btDevice = null;
    ScanCallback btScanCallback = null;
    BluetoothGattService btService = null;
    BluetoothGattCharacteristic btCharaRx = null, btCharaTx = null;
    BluetoothGatt btGatt = null;
    Handler handler = new Handler();

    Callback callback = null;

    boolean isConnected = false; //标记当前是否已连接

    SharedPreferences sharedPreferences = null;

    @SuppressLint("MissingPermission")
    public Bluetooth(Context context) {
        sharedPreferences = context.getSharedPreferences("bluetooth",Context.MODE_PRIVATE);

        btAdapter = ((BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE)).getAdapter();
        if(!btAdapter.isEnabled())
            btAdapter.enable();

        btCallback = new BluetoothGattCallback() { //设置GATT回调
            @Override
            public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) { //连接状态改变
                Log.d("gatt","state change status="+status+" newState="+newState);
                if(status == BluetoothGatt.GATT_SUCCESS && newState == BluetoothGatt.STATE_CONNECTED) { //连接成功
                    btGatt = gatt;
                    gatt.setPreferredPhy(BluetoothDevice.PHY_LE_2M,BluetoothDevice.PHY_LE_2M,BluetoothDevice.PHY_OPTION_NO_PREFERRED);
                    gatt.discoverServices(); //开始搜索服务
                    Log.d("gatt","connected");
                }else if(newState == BluetoothGatt.STATE_DISCONNECTED) { //连接断开
                    isConnected = false;
                    gatt.disconnect();
                    gatt.close();
                    if(callback != null)
                        callback.onConnectStateChange(false); //发送连接断开事件
                    Log.d("gatt","connect disconnect");

                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            connectDevice(context, btDeviceName);
                        } //1s后重新连接
                    },1000);
                }
                super.onConnectionStateChange(gatt, status, newState);
            }

            @Override
            public void onServicesDiscovered(BluetoothGatt gatt, int status) { //搜索服务完成
                Log.d("gatt","service discover status="+status);
                if(status == BluetoothGatt.GATT_SUCCESS) {
                    Log.d("gatt","service discover success");
                    List<BluetoothGattService> services = gatt.getServices();
                    btService = gatt.getService(UUID.fromString(uuidService)); //获取目标服务
                    btCharaRx = btService.getCharacteristic(UUID.fromString(uuidRxChara)); //获取接收特征
                    btCharaTx = btService.getCharacteristic(UUID.fromString(uuidTxChara)); //获取发送特征
                    BluetoothGattDescriptor descriptor = btCharaTx.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"));
                    descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    gatt.writeDescriptor(descriptor);
                    gatt.setCharacteristicNotification(btCharaTx,true);

                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            isConnected = true;
                            if(callback != null)
                                callback.onConnectStateChange(true); //发送连接成功事件
                            Log.d("gatt","isConnected = true");
                        }
                    },500);
                }
                super.onServicesDiscovered(gatt, status);
            }

            @Override
            public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
                super.onCharacteristicWrite(gatt, characteristic, status);
            }

            @Override
            public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) { //接收到数据
                if(callback != null)
                    callback.onReceive(characteristic.getValue()); //发送接收到数据事件
                super.onCharacteristicChanged(gatt, characteristic);
            }
        };

        btScanCallback = new ScanCallback() {
            @Override
            public void onScanResult(int callbackType, android.bluetooth.le.ScanResult result) { //搜索到设备
                super.onScanResult(callbackType, result);
                if(result.getDevice().getName() != null && result.getDevice().getName().equals(btDeviceName)) { //与目标设备名称匹配，开始连接
                    btDevice = result.getDevice();
                    btDevice.connectGatt(context,false,btCallback,BluetoothDevice.TRANSPORT_LE);
                    btAdapter.getBluetoothLeScanner().stopScan(btScanCallback);
                    SharedPreferences.Editor editor = sharedPreferences.edit();
                    editor.putString("MAC " + btDeviceName,result.getDevice().getAddress());
                    editor.apply();
                }
            }
        };
    }

    public void setCallback(Callback callback) {
        this.callback = callback;
    } //设置事件回调

    //设置UUID
    public void setUUID(String service, String rxChara, String txChara) {
        uuidService = service;
        uuidRxChara = rxChara;
        uuidTxChara = txChara;
    }

    @SuppressLint("MissingPermission")
    public void connectDevice(Context context, String deviceName) { //连接到指定名称的设备
        btDeviceName = deviceName;
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if(sharedPreferences.contains("MAC " + btDeviceName)) { //如果已经保存了MAC地址，直接连接
                    btDevice = btAdapter.getRemoteDevice(sharedPreferences.getString("MAC " + btDeviceName, ""));
                    btDevice.connectGatt(context, false, btCallback, BluetoothDevice.TRANSPORT_LE);
                }else { //否则开始扫描附近设备
                    btAdapter.getBluetoothLeScanner().startScan(btScanCallback);
                }
            }
        },1000);
    }

    @SuppressLint("MissingPermission")
    public void sendBytes(byte[] bytes) { //向目标设备发送数据
        if(isConnected && btGatt != null && btCharaRx != null) {
            btCharaRx.setValue(bytes);
            btGatt.writeCharacteristic(btCharaRx);
        }
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        if(btGatt != null) {
            btGatt.disconnect();
            btGatt.close();
            isConnected = false;
        }
    }

    public void clearMacInfo() { //清除保存的MAC地址，下次连接时重新扫描
        SharedPreferences.Editor editor = sharedPreferences.edit();
        editor.remove("MAC " + btDeviceName);
        editor.apply();
    }
}
