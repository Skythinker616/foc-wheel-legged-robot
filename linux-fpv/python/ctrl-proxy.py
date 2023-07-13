# Desc: 机器人图传控制和遥控数据转发程序
# by L.B.W 2023

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
import time
import threading
import socket, fcntl, struct
from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions
import signal
import subprocess

# 下位机器人的蓝牙名称和UUID
CTRL_DEVICE_NAME = "BalanceBot"
CTRL_UUID_SERVICE = "4c9a0001-fb2e-432e-92ec-c6f5316b4689"
CTRL_UUID_TX_CHARA = "4c9a0002-fb2e-432e-92ec-c6f5316b4689"
CTRL_UUID_RX_CHARA = "4c9a0003-fb2e-432e-92ec-c6f5316b4689"

# 自身蓝牙的名称和UUID
SELF_DEVICE_NAME = "NanoPI"
SELF_UUID_SERVICE = "4c9a0011-fb2e-432e-92ec-c6f5316b4689"
SELF_UUID_TX_CHARA = "4c9a0012-fb2e-432e-92ec-c6f5316b4689"
SELF_UUID_RX_CHARA = "4c9a0013-fb2e-432e-92ec-c6f5316b4689"

# 自身UDP服务器绑定的IP和端口号
UDP_IP = "0.0.0.0"
UDP_PORT = 5000

# 创建UDP服务器
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(2)

udp_client_addr = None # UDP客户端(APP)的地址
udp_data_list = [] # 来自UDP客户端的消息队列

is_proxy_mode = False # 当前是否处于遥控数据转发模式(即APP已连接)

process_exit_flag = False # 标记程序结束，各线程检测到此标记后退出

# 运行Linux shell命令
# 返回值ret.returncode==0表示命令执行成功
# 返回值ret.stdout为命令执行结果的标准输出
def run_shell(cmd:str):
	return subprocess.run(cmd, shell = True, capture_output = True, text = True)

# 扫描周围WiFi热点
def scan_wifi()->list:
	cmd_res = run_shell("nmcli -f SSID dev wifi list")
	if cmd_res.returncode != 0:
		return []
	wifi_list = cmd_res.stdout.split('\n')
	del wifi_list[0]
	for i in range(len(wifi_list)):
		wifi_list[i] = wifi_list[i].strip()
		if wifi_list[i] == "":
			del wifi_list[i]
	return list(set(wifi_list))

# 连接到指定WiFi热点
def connect_wifi(ssid, passwd)->bool:
	print("connect to wifi:", ssid, passwd)
	cmd = ""
	if passwd == "":
		cmd = "nmcli dev wifi connect \"" + ssid + "\" ifname wlan0"
	else:
		cmd = "nmcli dev wifi connect \"" + ssid + "\" password \"" + passwd + "\" ifname wlan0"
	return run_shell(cmd).returncode == 0

# 判断当前是否处于AP模式
# 返回True：AP模式, False：STA模式
def check_is_ap_mode()->bool:
	cmd_res = run_shell("iwconfig wlan0").stdout
	if "Mode:Master" in cmd_res:
		return True
	return False

# 设置WiFi AP状态和参数
# ap_enable: True:AP模式, False:STA模式
# ssid, passwd: AP模式下的SSID和密码
def set_wifi_ap(ap_enable:bool, ssid = "", passwd = "")->bool:
	if not ap_enable:
		return run_shell("turn-wifi-into-apmode no").returncode == 0
	else:
		if ssid == "" or len(passwd) < 8:
			return False
		else:
			return run_shell("echo \"" + ssid +  "\n" + passwd + "\n" + passwd + "\n\" | turn-wifi-into-apmode yes").returncode == 0

# 判断视频推流是否正在运行
def set_streamer(enable:bool)->bool:
	if enable:
		return run_shell("service mjpg-ffserver start").returncode == 0
	else:
		return run_shell("service mjpg-ffserver stop").returncode == 0

# UDP数据接收线程函数
def udp_recv_process():
	global is_proxy_mode, sock, udp_client_addr, udp_data_list, process_exit_flag
	prev_udp_recv_time = 0
	while True:
		try:
			if process_exit_flag: # 检测到程序退出标记，退出线程
				set_streamer(False)
				print("UDP thread exit.")
				return
			data, addr = sock.recvfrom(1024) # 等待并接收一条UDP消息
			# print("recv from udp:", str(data), "addr:", addr)
			udp_client_addr = addr # 保存UDP客户端的地址
			udp_data_list.append(data) # 将消息加入消息队列
			if not is_proxy_mode: # 如果当前不处于遥控数据转发模式，切换到转发模式
				is_proxy_mode = True
				set_streamer(True)
			prev_udp_recv_time = time.time()
		except Exception as e:
			pass
		if time.time() - prev_udp_recv_time > 5: # 超过5秒未收到UDP消息，退出转发模式
			if is_proxy_mode:
				is_proxy_mode = False
				set_streamer(False)

# 下位机蓝牙数据接收回调
def notification_handler(sender, data):
	# print("Recv: " + str(data))
	if not udp_client_addr is None: # 将下位机传来的蓝牙数据从UDP转发给上位APP
		sock.sendto(data, udp_client_addr)

# 连接下位机蓝牙的协程处理函数
async def ctrl_proxy_process():
	global process_exit_flag

	if process_exit_flag:
		return

	await asyncio.sleep(3)

	# 作为主机扫描周边蓝牙设备
	print("Scanning for devices...")
	device = None
	while not device:
		device = await BleakScanner.find_device_by_filter(
			lambda d, ad: d.name and d.name.lower() == CTRL_DEVICE_NAME.lower(),
			timeout=3.0,
		)
		if process_exit_flag:
			print("Client task exit.")
			return
		if not is_proxy_mode:
			print("Exit proxy mode.")
			return

	print("Device found.")
	print(device)

	while True:
		print("Connecting to device...")
		try:
			client = BleakClient(device) # 尝试连接到下位机蓝牙
			await client.connect()
			print("Connected to device.")
			await client.start_notify(CTRL_UUID_RX_CHARA, notification_handler) # 注册数据接收回调
			while True:
				if not client.is_connected: # 检查蓝牙连接状态，如果断开，重新连接
					print("Device is disconnected. Reconecting...")
					await client.stop_notify(CTRL_UUID_RX_CHARA)
					await asyncio.sleep(2)
					break
				while len(udp_data_list) > 0: # 如果有UDP消息则转发给下位机
					await client.write_gatt_char(CTRL_UUID_TX_CHARA, udp_data_list[0])
					del udp_data_list[0]
				await asyncio.sleep(0.01)
				if process_exit_flag:
					await client.disconnect()
					print("Client task exit.")
					return
				if not is_proxy_mode: # 如果数据转发模式被关闭，断开下位机蓝牙
					await client.disconnect()
					print("Exit proxy mode.")
					return
		except Exception as e:
			if process_exit_flag:
				print("Client task exit.")
				return
			if not is_proxy_mode:
				print("Exit proxy mode.")
				return
			print("Connection failed. Retrying...")
			await asyncio.sleep(2)

# 获取本机IP地址
def get_ip_address(ifname):
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	return socket.inet_ntoa(fcntl.ioctl(
		s.fileno(),
		0x8915,
		struct.pack('256s', ifname[:15].encode('utf-8'))
	)[20:24])

# 本机蓝牙数据协程处理函数(处理来自上位APP的蓝牙配网消息)
async def self_ble_process():
	global process_exit_flag

	await asyncio.sleep(3)

	# 要创建的GATT服务和特征
	gatt={
		SELF_UUID_SERVICE:{
			SELF_UUID_TX_CHARA:{
				"Properties":GATTCharacteristicProperties.notify,
				"Permissions":GATTAttributePermissions.readable
			},
			SELF_UUID_RX_CHARA:{
				"Properties":GATTCharacteristicProperties.write,
				"Permissions":GATTAttributePermissions.writeable
			}
		}
	}

	server = BlessServer(name = "NanoPI") # 创建BLE服务端

	# 通过notify方式发送数据给上位APP
	def tx_notify(value):
		server.get_characteristic(SELF_UUID_TX_CHARA).value = value
		server.update_value(SELF_UUID_SERVICE, SELF_UUID_TX_CHARA)

	# 上位机写入特征值的回调函数
	def on_self_write(chara, value, **kwargs):
		print("on self write:", value)
		chara.value = value
		cmd = value.decode("utf8")
		if cmd.startswith("GET_IP"): # 上位机请求获取本机IP地址
			tx_notify(("SELF_IP:"+get_ip_address("wlan0")).encode())
		elif cmd.startswith("SCAN_WIFI"): # 上位机请求扫描周边WIFI
			if check_is_ap_mode():
				set_wifi_ap(False)
			wifi_list = scan_wifi()
			res = "WIFI_LIST:"
			for wifi_name in wifi_list:
				res = res + wifi_name + "\t"
			tx_notify(res.encode())
		elif cmd.startswith("CONN_WIFI:"): # 上位机请求连接WIFI
			if check_is_ap_mode():
				set_wifi_ap(False)
			wifi_info = cmd.replace("CONN_WIFI:","").split("\t")
			if connect_wifi(wifi_info[0], wifi_info[1]):
				tx_notify("CONN_WIFI_OK".encode())
			else:
				tx_notify("CONN_WIFI_ERR".encode())
		elif cmd.startswith("SET_AP"): # 上位机请求设置AP模式SSID和密码
			ap_info = cmd.replace("SET_AP:","").split("\t")
			if set_wifi_ap(True, ap_info[0], ap_info[1]):
				tx_notify("SET_AP_OK".encode())
			else:
				tx_notify("SET_AP_ERR".encode())


	server.write_request_func = on_self_write # 注册写入回调函数
	await server.add_gatt(gatt) # 设置GATT服务和特征
	await server.start() # 启动BLE服务端
	print("Adv started.")
	while True:
		if process_exit_flag: # 程序退出标志被置位，退出该协程
			await server.stop()
			print("Server task exit.")
			return
		if is_proxy_mode: # 切换为数据转发模式(收到了UDP消息)，退出该协程
			await server.stop()
			print("Adv stop for proxy mode.")
			return
		await asyncio.sleep(0.1)

# 系统SIGNAL信号处理函数，拦截SIGINT信号(CTRL+C)并触发程序退出标志
def signal_handler(sig, frame):
	global process_exit_flag
	print('\nSIGINT detected. Exiting...')
	process_exit_flag = True

# 主程序入口
if __name__ == "__main__":
	signal.signal(signal.SIGINT, signal_handler) # 注册SIGNAL信号处理函数
	threading.Thread(target = udp_recv_process).start() # 启动UDP消息接收线程
	# 两个蓝牙协程轮流运行
	while not process_exit_flag:
		asyncio.run(self_ble_process()) # 本机蓝牙协程，接受上位APP消息，收到UDP消息后退出
		asyncio.run(ctrl_proxy_process()) # 下位机蓝牙协程，进行上位APP到下位机的数据转发，UDP消息断流后退出
