from btpycom import *

def onStateChanged(state,msg):
    if state == "LISTENING":
        print("Server is Listening")
    elif state == "CONNECTED":
        print('Connection Established to',msg)
    elif state == "MESSAGE":
        print("Got Message",msg)
        server.sendMessage(msg)


serviceName = "EchoServer"
server = BTServer(serviceName,stateChanged=onStateChanged)
