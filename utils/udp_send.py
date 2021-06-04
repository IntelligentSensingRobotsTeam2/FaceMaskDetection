# -*- coding: utf-8 -*-

import time
import socket, fcntl, struct


#client 发送端
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = '192.168.50.135'
PORT = 8000  ## voice udp server port

def send_data(data):
    # ip = get_ip('ens33')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # for data in [b'mask', b'start spray']:
    # 发送数据:
    enc_data = data.encode('utf-8')
    s.sendto(enc_data, (ip, PORT))
    print('udp send:',data)
    # 接收数据:
    # receive_data, _ =s.recvfrom(1024)
    # print(receive_data)
    s.close()


if __name__ == '__main__':
    info = 'angle:90'
    send_data(info)
