import socket
import pygame
import os
import time
import threading
from PyQt5.QtGui import*
from PyQt5.QtWidgets import*
from PyQt5.QtCore import*
from PyQt5 import uic
import sys
import cv2


HEADER =100
# PORT =5050
# Below is port for LAN to UART
PORT=5005
FORMAT = 'utf-8'
DISCONNECT_MESSAGE ="!DISCONNECT"
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 3
AXIS_R2 = 45
AXIS_L2 = 5
# Labels for DS4 controller buttons
# # Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_SQUARE = 2
BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 3
BUTTON_L1 = 9
BUTTON_R1 = 10
BUTTON_L2 = 7
BUTTON_R2 = 8
BUTTON_SHARE = 8
BUTTON_OPTIONS = 6

BUTTON_LEFT_STICK = 10
BUTTON_RIGHT_STICK = 11

UP_ARROW=11
DOWN_ARROW=12
LEFT_ARROW=13
RIGHT_ARROW=14
BUTTON_PS = 5
BUTTON_PAD = 15
axis = {}
button = {}

ARM_FLAG=0


class CameraThread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self,name):
        # super(CameraThread,self).__init__()
        QThread.__init__(self, parent=None)
        
        self.name=name

    def TakeScreenshot(self):
        ps4.logic = 1

    # def setFullScreen(self):
    #     if ps4.full_screen == 1:
    #         ps4.full_screen = 0
    #     else:
    #         ps4.full_screen = 1

    def run(self):
        cap = cv2.VideoCapture(self.name)
        
        value = 0
        while True:
            ret, frame = cap.read()
            if ret:

                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(
                    rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                
                # if ps4.full_screen == 0:
                #     p = convertToQtFormat.scaled(720,1280, Qt.KeepAspectRatio)
                # else:
                #     p = convertToQtFormat.scaled(620, 480, Qt.KeepAspectRatio)
                p = convertToQtFormat.scaled(620, 480, Qt.KeepAspectRatio)
                

                self.changePixmap.emit(p)
                if ps4.logic == 1:
                    value += 1
                    cv2.imwrite(
                        'C:/Users/Devansh/Python/PyQt5 Codes/PyQt5 Designer/Screenshot/CamPhotoNo.%s.png' % (value), frame)
                    Camera.logic = 0
                    print("SS taken")
    
class ps4(QMainWindow):
    speak=pyqtSignal(str)
    
    logic = 0
    full_screen = 0
    def __init__(self):      
        super().__init__()
        pygame.init()
        pygame.joystick.init()
        comm = Comm()
        self.speak.connect(self.setTerminal)
        global gear
        gear=0
        
        global ARM_FLAG
        ARM_FLAG=0
        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        global a
        a=uic.loadUi(r"C:/GUI-2022-main/GUI-2022-main/MRM.ui", self)
        
    def setup(self):
        # Three types of controls: axis, button, and hat
        
        for i in range(self.controller.get_numaxes()):
            axis[i] = 0.0
            # Buttons are initialized to False
        for i in range(self.controller.get_numbuttons()):
            button[i] = False
            
        self.control()

    # Main loop, one can press the PS button to break
    def control(self):
        self.quit = False
        while self.quit == False:

            # Get events
            for event in pygame.event.get():

                if event.type == pygame.JOYAXISMOTION:
                    axis[event.axis] = round(event.value,3)
                elif event.type == pygame.JOYBUTTONDOWN:
                    button[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    button[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    hat[event.hat] = event.value

            self.quit = button[BUTTON_PS]
            # if(self.quit==True):
            #     self.SendMsg("m"+'0'+"s"+"08000"+"f"+"08000"+"n")
            self.display()

    def debounce(self,b):
        x=0
        while(x<1000000/2):
            if(button[b] != True):
                return False
            x+=1
        
        return True
    
    def getGear(self):
        global gear
        if(self.debounce(BUTTON_R1)==True and gear<9):
                gear += 1
            
        if(self.debounce(BUTTON_L1)==True and gear>0): 
                gear -= 1
    
    def getValueXJoy(self,stick):
        return  str(int(axis[stick]*8000+8000))
    
    def getValueYJoy(self,stick):
        return  str(int(-1*axis[stick]*8000+8000))
    
    
    def ArmMode(self):
        
        x_r=self.getValueXJoy(AXIS_RIGHT_STICK_X)
        y_r=self.getValueYJoy(AXIS_RIGHT_STICK_Y)
            
        x_l=self.getValueXJoy(AXIS_LEFT_STICK_X)
        y_l=self.getValueYJoy(AXIS_LEFT_STICK_Y)
        
        x_r=x_r.zfill(5)
        y_r=y_r.zfill(5)
            
        x_l=x_l.zfill(5)
        y_l=y_l.zfill(5)
        
        roll="00000"
        pitch="00000"
        swivel="00000"
        gripper="00000"


            # print(x)
        # self.getGear()

            # For Swivel 
        
        if(button[BUTTON_L1]==True):
            swivel="16000"
                
        elif(button[BUTTON_R1]==True):
            swivel="16001"
            
        else:
            swivel="00000"
            
        if(button[BUTTON_L2] == True): # THIS IS ACTUALLY LEFT JOYSTICK BUTTON
            
            exec_time=0.00
            start_time = time.time()

            while(exec_time<=2):
                #gripper="16000" # OPEN
                print("Button L2")
                exec_time = time.time() - start_time 
                
        elif(button[BUTTON_R2]==True): # THIS IS ACTUALLY RIGHT JOYSTICK BUTTON
            print("Button R2")
            gripper="16001" # CLOSE
            
        else:
            print("NONE")
            
                
            # For Roll
        if(button[LEFT_ARROW]==True):
            
            pitch= "16000"
            roll= "16000"

        elif(button[RIGHT_ARROW]==True):
            
            pitch="16001"
            roll="16001"

        # For Pitch
        elif(button[UP_ARROW]==True):
           
            pitch="16000"
            roll="16001"
        elif(button[DOWN_ARROW]==True):
            
            pitch="16001"
            roll="16000"
        
        else:
            
            pitch="00000"
            roll="00000"
                
        if(button[BUTTON_SQUARE]==True):
            gripper="16000"
        elif(button[BUTTON_CROSS]==True):
            gripper="16001"
        else:
            gripper="00000"
        
            
        self.SendMsg("as"+str(swivel)+'o'+y_l+'t'+y_r+'r'+str(roll)+'p'+str(pitch)+'g'+str(gripper))
        
        
        
    def MotorMode(self):
        
        global gear
        x_r=self.getValueXJoy(AXIS_RIGHT_STICK_X)
        y_r=self.getValueYJoy(AXIS_RIGHT_STICK_Y)
        x_r=x_r.zfill(5)
        y_r=y_r.zfill(5)
        temp_y = y_r

        self.getGear()
        self.SendMsg("m"+str(gear)+"s"+x_r+"f"+y_r+"n")

                        
            
    def SendMsg(self, msg):
        comm.send(msg)
        print(msg)
        
    # def DigitalTrain(self,b):
    #     while(button[b]==True):
            
        
    def display(self):
        os.system('cls')
            #gear=0
            # -1 cause DS4 values  are inverted
        x_r=self.getValueXJoy(AXIS_RIGHT_STICK_X)
        y_r=self.getValueYJoy(AXIS_RIGHT_STICK_Y)
            
        x_l=self.getValueXJoy(AXIS_LEFT_STICK_X)
        y_l=self.getValueYJoy(AXIS_LEFT_STICK_Y)
            
            
            # print(y_l)
        global ARM_FLAG
        if(self.debounce(BUTTON_CIRCLE)==True):
            ARM_FLAG=ARM_FLAG^1
            self.debounce(BUTTON_CIRCLE)
        
        
        
        if(ARM_FLAG==1):
            print("ARM MODE...")
            self.ArmMode()
        elif(ARM_FLAG==0):
            print("MOTOR MODE...")
            self.MotorMode()
        
        

        global c
        # c=self.camera_box.currentIndex()
            
        # if(button[BUTTON_CIRCLE]==True):
        #     self.speak.emit("Current FUcked")   
                
        # elif(button[BUTTON_CIRCLE]==False):
        #     self.speak.emit("Current Bueno") 
            
        self.x_axis.setText(x_r)
        self.y_axis.setText(y_r)
            
        # self.x_axis_2.setText(x_l)
        # self.y_axis_2.setText(y_l)
        # self.gear_label.setText(str(gear))
            

        # Limited to 30 frames per second to make the display not so flashy                                                                                                                                                                                                   
        clock = pygame.time.Clock()
        clock.tick(30) 
    
    
    def image(self):     
        
        # self.th1=CameraThread('http://root:mrm@192.168.1.90/axis-cgi/mjpg/video.cgi?camera=4')
        # self.th1.changePixmap.connect(self.setImage1)
        # self.th1.start()

        self.th2=CameraThread(0)
        self.th2.changePixmap.connect(self.setImage2)
        self.th2.start()
        
        
        
        # th2=CameraThread()
        # a.screenshot.pressed.connect(CameraThread.TakeScreenshot)
        
    pyqtSlot(QImage)
    def setImage1(self,image):
        global c
        c=1
        if(c == 1):
            a.main_camera.setPixmap(QPixmap.fromImage(image))
        print(self)
        a.camera_2.setPixmap(QPixmap.fromImage(image))
        
    pyqtSlot(QImage)
    def setImage2(self,image):
        global c
        # c=1
        # if(c == 1):
        #     a.main_camera.setPixmap(QPixmap.fromImage(image))
        # print(self)
        a.camera_1.setPixmap(QPixmap.fromImage(image))
    
    pyqtSlot(str)
    def setTerminal(self,s):
        a.terminal.setPlainText(s)
        
class Comm():
    
    def __init__(self):
        
        # self.SERVER = socket.gethostbyname(socket.gethostname())

        # Below is IP for LAN to UART
        self.SERVER="192.168.1.7"
        self.ADDR=(self.SERVER,PORT)
        self.client =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(self.ADDR)
        # ps4.a.camera_2.setText("Hello")

    def send(self,msg):
        self.message=msg.encode(FORMAT)
        # self.msg_length=len(self.message)
        # self.send_length=str(self.msg_length).encode(FORMAT)
        
        #We are padding spaces to make message 64 bytes
        # self.send_length +=b' '*(HEADER-len(self.send_length))
        # self.client.send(self.send_length)
        self.client.sendall(self.message)


app = QApplication(sys.argv)
mainWindow = ps4()


mainWindow.image()


widget = QStackedWidget()
widget.addWidget(mainWindow)
widget.show()


comm=Comm()
# comm.send("Hello World")
# comm.send("Hello Apoogggrv")
# comm.send("Hello Bitchhjytr!")  818uj                                                                        

t1=threading.Thread(target=mainWindow.setup, args=())
t1.start()
# comm.send("lfdgon")
app.exec_()
comm.send(DISCONNECT_MESSAGE)
