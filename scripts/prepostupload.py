from time import sleep
import socket
Import("env")

UDP_IP = "127.0.0.1"

UDP_PORT = 9876
MESSAGE_BEFORE = b"up begin;"
MESSAGE_AFTER = b"up end;"

def send_udp(message):
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    print("message: %s" % message)

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(message, (UDP_IP, UDP_PORT))

def before_upload(source, target, env):
    print("before_upload")
    # do some actions
    send_udp(MESSAGE_BEFORE)
    sleep(0.5)


def after_upload(source, target, env):
    print("after_upload")
    # do some actions
    send_udp(MESSAGE_AFTER)

env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)