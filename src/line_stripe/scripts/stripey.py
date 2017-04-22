#!/usr/bin/env python

## Laser Line Scanning Obstacle Detection Code
# Joshua Jenkins
# Spring 2017
# josh824@vt.edu

################################################################################################
## Edit Log
################################################################################################
#
#   DATE              USER           Update Description
# 4/19/2017         jenkinsj         Original file production. Open sourcing and commenting for
#                                    other users to operate and modify
#
#
#
#
#


###### GENERAL OPERATION INSTRUCTIONS FOR PYTHON CODE ######
# F5 - To compile and run the code
#  q - Pressed when the video feed is selected to quit code


################################################################################################
## User Inputs
################################################################################################

## Import appropriate libraries
import numpy as np
import cv2
import rospy
from std_msgs.msg import Int8MultiArray
################################################################################################
# Beany Additions

import signal
import sys

pub = rospy.Publisher('stripe_obs_array', Int8MultiArray, queue_size=1)
rospy.init_node('pubby', anonymous=True)
rate = rospy.Rate(10)
quit = False

def sigint_handle(sig, frame):
    global quit
    quit = True

def pubby(outlist):
    # while not rospy.is_shutdown():
    stripe_obs_data = Int8MultiArray()
    #Need Function that runs algorithm and returns outlist
    stripe_obs_data.data = outlist
    #rospy.loginfo(stripe_obs_data)
    pub.publish(stripe_obs_data)
    # rate.sleep()

signal.signal(signal.SIGINT, sigint_handle)

################################################################################################

## Webcam
webcam = '/dev/video-cam-obs' #------------------------# Change me to toggle camera options (0,1,2,etc)

##Computer vision logic
cv=1;                 #------------------------# Change me to turn the computer vision overlay
                                               # on or off in the final video feed

## Abs percent error within the true y location on camera for the obsticle to not be an issue  (KEEP THESE CALIBRATED WELL)
laserThresh = 0.05    #------------------------# I represent the %error in the average lines
                                               # (the green dotted) with the true laser
                                               # laser location. If the error is bigger than
                                               # me, then we will assume an obstacle is present
laserTrue = 240       #------------------------# I represent the true y pixel location of the
                                               # laser. It's best to calibrate me often.

##Define hard brickwall filter (red lines)
upperlimit = laserTrue+100 #-------------------# I'm the y location of the upper red line filter
lowerlimit = laserTrue-100 #-------------------# I'm the y location of the lower red line filter

## Initialize laser color  (KEEP THESE CALIBRATED WELL)
R = 63                #------------------------# I'm the targeted Red RGB value for the computer
G = 209               #------------------------# I'm the targeted Green RGB value for the computer
B = 101               #------------------------# I'm the targeted Blue RGB value for the computer

## Initialize tolerance of laser color (KEEP THESE CALIBRATED WELL)
TolRangeRUp = 150     #------------------------# I'm the upper range of the red values. Add me to
TolRangeRDn = 20      #-------                 # the variable 'R' to get the max red RBG value the
                            #-                 # computer will consider. Note that I cannot be
                            #-                 # greater than 255. Doing so will cause an error.
                            #-
                            #------------------# Same as above, only I am the lower range. I cannot
                                               # be a negative number or else it will cause an error.
                                               
TolRangeGUp = 46      #------------------------# I'm the upper range of the red values. Add me to
TolRangeGDn = 50      #-------                 # the variable 'R' to get the max red RBG value the
                            #-                 # computer will consider. Note that I cannot be
                            #-                 # greater than 255. Doing so will cause an error.
                            #-
                            #------------------# Same as above, only I am the lower range. I cannot
                                               # be a negative number or else it will cause an error.
                                               
TolRangeBUp = 150     #------------------------# I'm the upper range of the red values. Add me to
TolRangeBDn = 40      #-------                 # the variable 'R' to get the max red RBG value the
                            #-                 # computer will consider. Note that I cannot be
                            #-                 # greater than 255. Doing so will cause an error.
                            #-
                            #------------------# Same as above, only I am the lower range. I cannot
                                               # be a negative number or else it will cause an error.


## Camera parameters
camera_change = 1     #------------------------# I'm a logical value that when 0 doesn't change the
                                               # camera parameters. When I'm 1, I do.
camera_print = 0      #------------------------# I'm a logical value that when 0, I don't print the
                                               # camera parameters. When I'm 1, I do.

## Number of points in CV lines
NumPointsInLine_filt = 50 #--------------------# My value represents the number of dots in the red filter
NumPointsInLine_avg = 5  #--------------------# My value represents the number of dots in the average green lines

## OpenCV command to turn on the webcam and initialize camera parameters
cap = cv2.VideoCapture(webcam)

## Camera parameters to set. The only important ones are the smiley face ones.
#  Note that different cameras may have different properties and responses.
if(camera_change==1):
    #current_pos = -1
    #based_index=-1
    #rel_pos=-1
    #width=640
    #height=480
    #frame_rate=0                # Can change :)
    #charcode=844715353.0                    # Can't change :(
    #frames_vid=-1                           # Can't change :(
    #format_mat=-1                           # Can't change :(
    #cap_mode=-1                             # Can't change :(
    brightness=100               # Can change :)
    contrast=100                # Can change :)
    saturation=60              # Can Change :)
    hue=0                       # Can Change :)
    #gain=14                                 # Can't change :(
    #exposure=15                             # Can't change :(
    #RGB=-1                                  # Can't change :(
    #prop_WB=17                              # Can't change :(
    #rectification=-1                        # Can't change :(


## Print camera parameters before running. This may screw up higher level code as it may
#  be monitoring outputs for just the 1x10 array of logical osbticle information
if(camera_print==1):
    # Exploring features of camera and video feed
    print(cap.get(0))         #0. CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
    print(cap.get(1))         #1. CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
    print(cap.get(2))         #2. CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file
    print(cap.get(3))         #3. CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
    print(cap.get(4))         #4. CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.  
    print(cap.get(5))         #5. CV_CAP_PROP_FPS Frame rate.  
    print(cap.get(6))         #6. CV_CAP_PROP_FOURCC 4-character code of codec.   
    print(cap.get(7))         #7. CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
    print(cap.get(8))         #8. CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
    print(cap.get(9))         #9. CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
    print(cap.get(10))        #10. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras). 
    print(cap.get(11))        #11. CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
    print(cap.get(12))        #12. CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
    print(cap.get(13))        #13. CV_CAP_PROP_HUE Hue of the image (only for cameras).
    print(cap.get(14))        #14. CV_CAP_PROP_GAIN Gain of the image (only for cameras).
    print(cap.get(15))        #15. CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
    print(cap.get(16))        #16. CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
    print(cap.get(17))        #17. CV_CAP_PROP_WHITE_BALANCE Currently unsupported
    print(cap.get(18))        #18. CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently






################################################################################################
## Function Definitions
################################################################################################

## Sector Analysis
#  This function takes the x location of a pixel as an input and returns the sector that pixel
#  belongs to. The sectors are defined as n/10ths of the total x view of the camera. So if you
#  input a value of 501 pixels in a 1000 pixel x frame, you'd be in sector 5.

def sectorAnalysis(x): ## Sector sorting algorithm. Returns the sector the detected keypoint is in

    # Inputs:
    #    x - A value representing the x coordinate of a pixel
    #
    # Outputs/returns:
    #   sector - A value corresponding to the n/10th spanwise sector of the frame
    #

    # Compare the input with each combination  with elseif structures    
    if(x>=0 and x<width/10*1):
        sector=1
    elif(x>=width/10*1 and x<width/10*2):
        sector=2
    elif(x>=width/10*2 and x<width/10*3):
        sector=3
    elif(x>=width/10*3 and x<width/10*4):
        sector=4
    elif(x>=width/10*4 and x<width/10*5):
        sector=5
    elif(x>=width/10*5 and x<width/10*6):
        sector=6
    elif(x>=width/10*6 and x<width/10*7):
        sector=7
    elif(x>=width/10*7 and x<width/10*8):
        sector=8
    elif(x>=width/10*8 and x<width/10*9):
        sector=9
    elif(x>=width/10*9 and x<=width/10*10):
        sector=10
        
    return sector;



## Concatenate Average Data
#  This function builds vectors and appends data to the appropriate variable.
#  All potential variables are passed through for one simple call. It'll
#  know which variable to append the data to based on the value of sector

def concat_avdata(y,sector,av1,av2,av3,av4,av5,av6,av7,av8,av9,av10):

    # Inputs:
    #    y   - A value representing the y coordinate of a pixel
    # sector - The n/10th location of the x coordinate of the pixel
    #  av1   - Array of sector 1 values to average later, note these aren't averages
    #  av2   - Array of sector 2 values to average later, note these aren't averages
    #  av3   - Array of sector 3 values to average later, note these aren't averages
    #  av4   - Array of sector 4 values to average later, note these aren't averages
    #  av5   - Array of sector 5 values to average later, note these aren't averages
    #  av6   - Array of sector 6 values to average later, note these aren't averages
    #  av7   - Array of sector 7 values to average later, note these aren't averages
    #  av8   - Array of sector 8 values to average later, note these aren't averages
    #  av9   - Array of sector 9 values to average later, note these aren't averages
    #  av10   - Array of sector 10 values to average later, note these aren't averages
    #
    # Outputs/returns:
    #   av1-10 - Returns all average arrays back into code local variable memory
    #
    
    if(sector==1):
        av1[len(av1):] = [y]
    elif(sector==2):
        av2[len(av2):] = [y]
    elif(sector==3):
        av3[len(av3):] = [y]
    elif(sector==4):
        av4[len(av4):] = [y]
    elif(sector==5):
        av5[len(av5):] = [y]
    elif(sector==6):
        av6[len(av6):] = [y]
    elif(sector==7):
        av7[len(av7):] = [y]
    elif(sector==8):
        av8[len(av8):] = [y]
    elif(sector==9):
        av9[len(av9):] = [y]
    elif(sector==10):
        av10[len(av10):] = [y]
        
    return av1,av2,av3,av4,av5,av6,av7,av8,av9,av10;



## Sector Validation
#  av1-10 has now been consolidated to a single value. This is the average of all
#  detected keypoints for each sector. This function now cycles through each sector
#  and takes the averages to compare to the true value. The output is the array
#  that a code processing this one will interpret to be suggestions for navigation

def sectorValidation(laserThresh,laserTrue,av1,av2,av3,av4,av5,av6,av7,av8,av9,av10):
    # Inputs:
    #laserThresh- The acceptable percent error between true
    # laserTrue - The true y pixel coordinates of the laser for no obsticle
    #   av1     - Average of sector 1 values to average later, note these ARE averages
    #   av2     - Average of sector 2 values to average later, note these ARE averages
    #   av3     - Average of sector 3 values to average later, note these ARE averages
    #   av4     - Average of sector 4 values to average later, note these ARE averages
    #   av5     - Average of sector 5 values to average later, note these ARE averages
    #   av6     - Average of sector 6 values to average later, note these ARE averages
    #   av7     - Average of sector 7 values to average later, note these ARE averages
    #   av8     - Average of sector 8 values to average later, note these ARE averages
    #   av9     - Average of sector 9 values to average later, note these ARE averages
    #   av10    - Average of sector 10 values to average later, note these aren't averages
    #
    # Outputs/returns:
    #   outlist - Returns 1x10 outlist where 1 represents obsticle present and 0
    #             represents clear way. The array's index corresponds to the sector
    #             of the cameras field of view.
    #

    # Initialize null outlist
    outlist=[0,0,0,0,0,0,0,0,0,0]

    # Process through each sector and average, assign logical variables as such
    if(abs(av1-laserTrue)/laserTrue<laserThresh):
        outlist[1-1]=0
    else:
        outlist[1-1]=1
        
    if(abs(av2-laserTrue)/laserTrue<laserThresh):
        outlist[2-1]=0
    else:
        outlist[2-1]=1
        
    if(abs(av3-laserTrue)/laserTrue<laserThresh):
        outlist[3-1]=0
    else:
        outlist[3-1]=1
        
    if(abs(av4-laserTrue)/laserTrue<laserThresh):
        outlist[4-1]=0
    else:
        outlist[4-1]=1
        
    if(abs(av5-laserTrue)/laserTrue<laserThresh):
        outlist[5-1]=0
    else:
        outlist[5-1]=1
        
    if(abs(av6-laserTrue)/laserTrue<laserThresh):
        outlist[6-1]=0
    else:
        outlist[6-1]=1
        
    if(abs(av7-laserTrue)/laserTrue<laserThresh):
        outlist[7-1]=0
    else:
        outlist[7-1]=1
        
    if(abs(av8-laserTrue)/laserTrue<laserThresh):
        outlist[8-1]=0
    else:
        outlist[8-1]=1
        
    if(abs(av9-laserTrue)/laserTrue<laserThresh):
        outlist[9-1]=0
    else:
        outlist[9-1]=1
        
    if(abs(av10-laserTrue)/laserTrue<laserThresh):
        outlist[10-1]=0
    else:
        outlist[10-1]=1

    return outlist;



## Computer Vision Lines
#  This function builds horizontal numpy contour variable arrays that will present dashed lines over
#  the area that is being scanned. Note that this is only optimized for completely horizontal and
#  completely vertical lines. Diagonal lines is something that can be improved.

def CVLines(x1,x2,y1,y2,NumPointsInLine,orient): # Last argument corresponds to horiz (1) or vert (2) lines

    # Inputs:
    #      x1       - The x location of your starting point of the line you want CV'd onto the video stream
    #      x2       - The x location of your ending point of the line you want CV'd onto the video stream
    #      y1       - The y location of your starting point of the line you want CV'd onto the video stream
    #      y2       - The y location of your ending point of the line you want CV'd onto the video stream
    #NumPointsInLine- The number of points to be included in the line
    #   orient      - This value is 1 for a horizontal line or 2 for a vertical line
    #
    # Outputs/returns:
    #contours_output- Contour array optimized for cv2.drawcontours
    #
    
    # Cycle through mod operator to make sure span of line is an integer
    if(orient==1):
        while((x2-x1)%NumPointsInLine!=0):
            NumPointsInLine=NumPointsInLine+1
    else:
        while((y2-y1)%NumPointsInLine!=0):
            NumPointsInLine=NumPointsInLine+1

    # Initialize string to concatenate with command and execute        
    mainStringRef = "contours_line=["

    # Iterate through each potential point in line                    
    for Ref_Line_Count in range(0,NumPointsInLine+1):

        if (Ref_Line_Count==0):
            xpixel=x1
            ypixel=y1
        else:
            if(orient==1):
                xpixel=x1+Ref_Line_Count*int(round((x2-x1)/NumPointsInLine))
            else:
                ypixel=y1+Ref_Line_Count*int(round((y2-y1)/NumPointsInLine))
                        
        string_element ="line_contour_%s = np.array([[%s, %s]], dtype=np.int32)"%(str(Ref_Line_Count),str(xpixel),str(ypixel))

        string_iter ="line_contour_%s"%(str(Ref_Line_Count))

        # Execute string command                
        exec(string_element)

        # Work more on main string, this logic solves issues of making sure commas are right
        if (Ref_Line_Count==0):
            mainStringRef = mainStringRef + string_iter# + "," + string_lower_iter
        else:
            mainStringRef = mainStringRef + "," + string_iter# + "," + string_lower_iter

    # Cap main string with bracket
    mainStringRef = mainStringRef + "]"

    # Execute string command that we just built up
    exec(mainStringRef)

    # Reassign variable names
    contours_output=contours_line

    return contours_output



## Combine Computer Vision Lines: 2 Inputs
#  This function just appends arrays together. Super simple.
def combineCVLines2(line1,line2):

    # Inputs:
    #   line1 - 1st array to be appended to
    #   line2 - 2nd array to append to line1
    #
    # Outputs/returns:
    #   line1 - [line1,line2] appended to each other
    #
    
    line1[len(line1):] = line2

    return line1;






################################################################################################
## Setup
################################################################################################
    
## Outlist initialization
outlist=[0,0,0,0,0,0,0,0,0,0]

## Apply tolerances
Rmin = R - TolRangeRDn
Rmax = R + TolRangeRUp
Gmin = G - TolRangeGDn
Gmax = G + TolRangeGUp
Bmin = B - TolRangeBDn
Bmax = B + TolRangeBUp

## Create NumPy arrays from the boundaries
lower = np.uint8([Bmin, Gmin, Rmin])
upper = np.uint8([Bmax, Gmax, Rmax])

# Capture frame-by-frame
ret, frame = cap.read()

## Get image size and height
height, width, channels = frame.shape

## Define contour lines for final figure

# Manual hard filter definitions
contours_red_line_lower=CVLines(0,width,lowerlimit,lowerlimit,NumPointsInLine_filt,1)
contours_red_line_upper=CVLines(0,width,upperlimit,upperlimit,NumPointsInLine_filt,1)
contours_red_line=combineCVLines2(contours_red_line_lower,contours_red_line_upper)

# Sectors 1-10 vertical line definitions
NumPointsInLine_sec=30
sector1start=CVLines(0,0,lowerlimit,upperlimit,NumPointsInLine_sec,2)
sector1end=CVLines(int(width*1/10),int(width*1/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sector1start,sector1end)
sector2start=CVLines(int(width*2/10),int(width*2/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector2start)
sector3start=CVLines(int(width*3/10),int(width*3/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector3start)
sector4start=CVLines(int(width*4/10),int(width*4/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector4start)
sector5start=CVLines(int(width*5/10),int(width*5/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector5start)
sector6start=CVLines(int(width*6/10),int(width*6/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector6start)
sector7start=CVLines(int(width*7/10),int(width*7/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector7start)
sector8start=CVLines(int(width*8/10),int(width*8/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector8start)
sector9start=CVLines(int(width*9/10),int(width*9/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector9start)
sector10start=CVLines(int(width*10/10),int(width*10/10),lowerlimit,upperlimit,NumPointsInLine_sec,2)
sectorLines=combineCVLines2(sectorLines,sector10start)



################################################################################################
## Main Loop
################################################################################################

while(True):

    if(camera_change==1):
	## Set camera parameters to those set above
        #cap.set(0,current_pos)
        #cap.set(1,based_index)
        #cap.set(2,rel_pos)
        #cap.set(3,width)
        #cap.set(4,height)
        #cap.set(5,frame_rate)
        #cap.set(6,charcode)
        #cap.set(7,frames_vid)
        #cap.set(8,format_mat)
        #cap.set(9,cap_mode)
        cap.set(10,brightness)
        cap.set(11,contrast)
        cap.set(12,saturation)
        cap.set(13,hue)
        #cap.set(14,gain)
        #cap.set(15,exposure)
        #cap.set(16,RGB)
        #cap.set(17,prop_WB)
        #cap.set(18,rectification)

    # ReZero intialization of running averages
    av1=[]
    av2=[]
    av3=[]
    av4=[]
    av5=[]
    av6=[]
    av7=[]
    av8=[]
    av9=[]
    av10=[]

    # Capture frame-by-frame 
    ret, frame = cap.read()

    # Get image size and height
    height, width, channels = frame.shape

    # Find the colors within the  specific boundaries and apply the mask
    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask = mask)

    # Hold frame before processing
    frame2=output
    
    # Apply Gaussian Blur to image
    frame2 = cv2.GaussianBlur(frame2, (5, 5), 0)

    # Gray image
    imgray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    # Apply threshold http://docs.opencv.org/3.2.0/d4/d73/tutorial_py_contours_begin.html
    ret,thresh = cv2.threshold(imgray,127,255,0)

    # Contour code
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # Detect number of contour bodies
    cnt1 = len(contours)

    # Initialize flag variables that control the logic. These were implemented to make sure that
    #  the final command is executed correctly s.t. the array of data can be fed into the openCV plotter
    contour_start=1
    contour_cont=0

    # First for loop to execute the number of times there are shapes detected
    for x1 in range(0, cnt1):

        # Parse out the current nth iteration object detected to a variable for processing
        object1 = contours[x1]

        # Count the number of detected keypoints  in the object detected
        cnt2 = len(object1)

        # Second for loop to execute the number of elements that are in a shape    
        for x2 in range(0,cnt2):

            # Parse out the y and x pixels
            ypixel = object1[x2,0,1]
            xpixel = object1[x2,0,0]

            # Arrange the pixels in a numpy array for processing
            pixelnumpy = np.array([[xpixel, ypixel]], dtype=np.int32)

            # First logic test to compare the pixel location to the y coordinates specified to look at for the FIRST ITERATION
            if (ypixel < upperlimit and ypixel > lowerlimit and x2==0):

                # Assign the current pixel being analyzed as a successful candidate that meets the bound requirements
                iterated_contours = [pixelnumpy]

                # Determine sector of x pixel
                sectorTemp = sectorAnalysis(xpixel)

                # Add y pixel of point being considered to running arrays to be averaged later
                av1,av2,av3,av4,av5,av6,av7,av8,av9,av10=concat_avdata(ypixel,sectorTemp,av1,av2,av3,av4,av5,av6,av7,av8,av9,av10)

            # Second logic test to compare the pixel location to the y coordinates specified to look at for SUBSEQUENT ITERATIONS   
            if (ypixel < upperlimit and ypixel > lowerlimit and x2 != 0):

                # Assign the current pixel being analyzed as a successful candidate that meets the bound requirements
                iterated_contours = np.concatenate((iterated_contours, [pixelnumpy]))

                # Determine sector of x pixel
                sectorTemp = sectorAnalysis(xpixel)

                # Add y pixel of point being considered to running arrays to be averaged later
                av1,av2,av3,av4,av5,av6,av7,av8,av9,av10=concat_avdata(ypixel,sectorTemp,av1,av2,av3,av4,av5,av6,av7,av8,av9,av10)              

        # Logic to build the command to execute that will put together a numpy array of arrays to feed into contour plot.
        if 'iterated_contours' in locals():

            ## Place all the contour points for one object back into a contour numpy array
            iterated_contours=np.array(iterated_contours, dtype=np.int32)

            ## Assign a variable name that corresponds to the iteration of x1
            string_iter="filtered_contours_%s = iterated_contours" %(str(x1))
            exec(string_iter)

            ## Builds the string to format the contour points back into a contourplot format for the first iteration
            if (contour_start==1):
                string = "filtered_contours = [filtered_contours_%s"%(str(x1))

                #Flag variables to ensure correct building of string to execute
                contour_start=0
                contour_cont=1

            ## Builds the string to format the contour points back into a contourplot format for subsequent iterations
            if (x1!=0 and x1<=cnt1-1):
                string= string + ", filtered_contours_%s" %(str(x1))

        ## Logical structure to execute the string that has been built to put the code into the correct format. If no objects are detected, it skips this section entirely.   
    if (cnt1!=0):

        if (contour_cont==1):
            string = string + "]"

            # Execute string
            exec(string)

            # Compute CV lines to represent average on HUD
            if(len(av1)!=0):
                av1=sum(av1)/float(len(av1))
            else:
                av1=1
            if(len(av2)!=0):                    
                av2=sum(av2)/float(len(av2))
            else:
                av2=1
            if(len(av3)!=0):
                av3=sum(av3)/float(len(av3))
            else:
                av3=1
            if(len(av4)!=0):
                av4=sum(av4)/float(len(av4))
            else:
                av4=1
            if(len(av5)!=0):
                av5=sum(av5)/float(len(av5))
            else:
                av5=1
            if(len(av6)!=0):
                av6=sum(av6)/float(len(av6))
            else:
                av6=1
            if(len(av7)!=0):
                av7=sum(av7)/float(len(av7))
            else:
                av7=1
            if(len(av8)!=0):
                av8=sum(av8)/float(len(av8))
            else:
                av8=1
            if(len(av9)!=0):
                av9=sum(av9)/float(len(av9))
            else:
                av9=1
            if(len(av10)!=0):
                av10=sum(av10)/float(len(av10))
            else:
                av10=1

            # Build CV lines to plot on top of live video feed
            av1_CV=CVLines(width/10*0,width/10*1,av1,av1,NumPointsInLine_avg,1)
            av2_CV=CVLines(width/10*1,width/10*2,av2,av2,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av1_CV,av2_CV)
            av3_CV=CVLines(width/10*2,width/10*3,av3,av3,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av3_CV)
            av4_CV=CVLines(width/10*3,width/10*4,av4,av4,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av4_CV)
            av5_CV=CVLines(width/10*4,width/10*5,av5,av5,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av5_CV)
            av6_CV=CVLines(width/10*5,width/10*6,av6,av6,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av6_CV)
            av7_CV=CVLines(width/10*6,width/10*7,av7,av7,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av7_CV)
            av8_CV=CVLines(width/10*7,width/10*8,av8,av8,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av8_CV)
            av9_CV=CVLines(width/10*8,width/10*9,av9,av9,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av9_CV)
            av10_CV=CVLines(width/10*9,width/10*10,av10,av10,NumPointsInLine_avg,1)
            av_CV=combineCVLines2(av_CV,av10_CV)
            
            # Run final analysis on averages over the sectors
            outlist=sectorValidation(laserThresh,laserTrue,av1,av2,av3,av4,av5,av6,av7,av8,av9,av10)

            # PRINT OUTPUT (Probably the only thing that should be printing)
            # print(outlist)
            pubby(outlist)
        
            # Draw contours on original image
            if(cv==1):
                cv2.drawContours(frame,filtered_contours,-1,(255,0,0),3)
                cv2.drawContours(frame,contours_red_line,-1,(0,0,255),3)
                cv2.drawContours(frame,sectorLines,-1,(125,51,255),3)
                cv2.drawContours(frame,av_CV,-1,(0,225,0),3)

            # Display the resulting frame
            cv2.imshow('frame',frame)
        
        else:
            print([999,999,999,999,999,999,999,999,999,999])   

    else:

        if(cv==1):
            cv2.drawContours(frame,contours_red_line,-1,(0,0,255),3)
            cv2.drawContours(frame,sectorLines,-1,(125,51,255),3)
        
        # Display the resulting frame
        cv2.imshow('frame',frame)

    # Command waiting for user to press 'q' to stop         
    if (cv2.waitKey(1) & 0xFF == ord('q')) or quit:
        break

    # Take frame
    return_value, image1 = cap.read()
    # cv2.imwrite("frame.png",image1)


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# if __name__ == '__main__':
#     try:
#         pubby()
#     except rospy.ROSInterruptException:
#         pass
