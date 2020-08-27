from djitellopy import Tello
import cv2
import numpy as np

def initializeTello():
    '''this funcion initilizes communication witht eh tello Drone'''
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity=0
    myDrone.left_right_velocity=0
    myDrone.up_down_velocity=0
    myDrone.yaw_velocity=0
    myDrone.speed=0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone

def telloGetFrame(myDrone,w=360,h=420):
    '''usig the drone object we created this function will get the image frame from the drone'''
    myFrame= myDrone.get_frame_read()
    myFrame=myFrame.frame
    img=cv2.resize(myFrame,(w,h))
    return img

def findFace(img):
    '''uses Open cv's Viola Jone's algorithm to detect face i a frame'''

    faceCascade = cv2.CascadeClassifier(r'haarcascade_frontalface_default.xml')
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    faces=faceCascade.detectMultiScale(imgGray,1.2,4)

    myFaceList=[] #list of the centre of the
    myFaceListArea=[]


    for(x,y,w,h) in faces:
        '''This for loop goes through '''
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
        cx = x+w/2
        cy= y+h/2
        area=w*h
        myFaceListArea.append(area)
        myFaceList.append([cx,cy])

    if len(myFaceListArea)!=0:
        i=myFaceListArea.index(max(myFaceListArea))
        return img,[myFaceList[i],myFaceListArea[i]]
    else:
        return img,[[0,0],0]

def trackFace(myDrone,info,w,h,pid,pError):

    '''
    Given the centre of the frame size is 640, te midde is 640/2 which is our desired position
    from our info if cx is greater than 620 it will be a negetive value and will move to the right
    if cx is positive than we will be moving left similar concept in th vertical direction
    '''

    #PID contorller
    errorx = info[0][0] - w / 2
    errory = info[0][1] - h / 2
    errorz = info[1] - 25000
    error=[errorx,errory,errorz]

    # x-axis (left right)
    speedx = pid[0][0]*error+pid[0][1]*(error[0]-pError[0])
    speedx=int(np.clip(speedx,-100,100))# putting boundaries in the speed input

    # y-axis (up down)
    speedy = pid[0][0]*error+pid[0][1]*(error[1]-pError[1])
    speedy=int(np.clip(speedy,-100,100))# putting boundaries in the speed input


    # z axis (forward back)

    speedz = pid[0][0]*error+pid[0][1]*(error[2]-pError[2])
    speedz=int(np.clip(speedz,-100,100))# putting boundaries in the speed input

    speed=[speedx,speedy,speedz]




    if info[0][0] !=0:
        myDrone.for_back_velocity = speed[2]
        myDrone.left_right_velocity = speed[0]
        myDrone.up_down_velocity = speed[1]
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error=0

    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.for_back_velocity,
                                myDrone.left_right_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity )

    return error



def testTrackFace(myDrone,info,w,pid,pError):

    '''
    Given the centre of the frame size is 640, te midde is 640/2 which is our desired position
    from our info if cx is greater than 620 it will be a negetive value and will move to the right
    if cx is positive than we will be moving left similar concept in th vertical direction
    '''

    ## PID
    error = info[0][0] - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    print(speed)
    if info[0][0] != 0:
        myDrone.yaw_velocity = speed
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error = 0
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return error


