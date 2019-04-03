#!/usr/bin/env python3
'''COM2009-3009 EV3DEV TEST PROGRAM'''

# Connect left motor to Output C and right motor to Output B
# Connect an ultrasonic sensor to Input 3

import os
import sys
import time
import ev3dev.ev3 as ev3

# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)

 #methods for the PID for movement and avoidance
def getAvoidanceError(t, usv2, usv3):
    e=0
    if(usv2<=t or usv3<=t): #if the values of us2 or three exceed the threashold, they are too close to an object/// 
        if(usv2<usv3):
            e =-usv2
        elif(usv3<=usv2):
            e =usv3
    #capping the error so the ki doesn't go out of control
    if(e>100): 
        e=100
    if(e<-100):
        e=-100
    return(e)

def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # display something on the screen of the device
    print('Hello World!')

    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    # announce program start
    #ev3.Sound.speak('Test program starting!').wait()

  # set the motor variables
   
     #PID controller for movement and avoidance. 
    mb = ev3.LargeMotor('outB')
    mc = ev3.LargeMotor('outC')
    us3 = ev3.UltrasonicSensor('in3') #set ultrasonic sensor var
    us2 =ev3.UltrasonicSensor('in2')
    ls1=ev3.ColorSensor("in1")
    ls1.mode="COL-AMBIENT"
    tp=50 #target power, which is 50% power on both motors. may just be reworded to tsp(target speed) if we can't do % power
    kp=4 #the constant for the proportional controller, or how fast it turns to corrects. 10 is just a wild guess. 
            # additionally kp=10(*10) because we divide p by 10 after the calculation to help the robot process ints. apparently...
            # it doesn't like floats, so we multiply a 10 to 99 kp value by 10, and a .1 to 1 kp value by 100
    ki=.05 # constant for the integral controller, or how fast it adds extra gentle turn. good for fixing small past errors. also divided by 10.
    kd=1 # constant for the derivitive controller, or how fast it preemptivly adds/ subtracts turn based on the integral. also divided by 10
    kpLight=5
    kiLight=.05
    kdLight=1
    target= 200  #set the target distance. This is also known as the "offset". used for noting how close the robot can get to bjects before it panics
    

   

    #setting containers for varibles to be used. Don't modify these, the code does that
    startTime=time.time()
    integral=0
    lastError=0
    derivitive=0
    mcMove=0
    mbMove=0
    lightValue=0
    lightError=0
    maxLightvalue=0
    currentLightValue=0
    lastLightError=0
    i=0
    romeObjDirection=1
    #else() #all other PIDs go above this one in an if statment of decending physical priority.
    while True:
        avoidError=getAvoidanceError(target,us2.value(),us3.value())
        debug_print(avoidError)
        if(avoidError!=0 and currentLightValue!=maxLightvalue):  #OBJECT AVOIDANCE PID
            i=0
            #if(avoidError>0):
            #    romeObjDirection=1
           # else:
            #    romeObjDirection=-1
           # debug_print(romeObjDirection)
            error=avoidError #calls the method which gets the error
            #debug_print(error)
            dt= time.time()-startTime # dt is the delta time since the program started running.
            #debug_print(us2.value())
            integral= ((1/100)*integral)+(error*dt) #the integral is the sum of all errors over time, reduced by 1/3rd every tick so it doesn't go out of control
            #debug_print(dt)
            derivitive= error-lastError # the derivitive is the estimated next error based on the last error and the current error.
            p= (kp*error)+(ki*integral)+(kd*derivitive) #if error is 0, do not turn, P is the total rate of turn
            p=p/10
            #debug_print('error is: '+str(error))
            #debug_print('integral: ' +str(integral))
            #debug_print('derivitive is: '+str(derivitive))
            #debug_print('toatl is: '+str(p))       
            mbMove=tp-p#-(abs(error)/10)
            mcMove=tp+p#-(abs(error)/10)

            if(mbMove>=100): #setting move caps (at 100% power)
                mbMove=100
            if(mbMove<=-100):
                mbMove=-100
            if(mcMove<=-100):
                mcMove=-100
            if(mcMove>=100):
                mcMove=100 
            mb.run_direct(duty_cycle_sp= -mbMove)
            mc.run_direct(duty_cycle_sp= -mcMove)
            # debug_print(mbMove)
            if(integral>1000): #capping integral
                integral=1000
            if(integral<-1000):
                integral=-1000
        elif(currentLightValue>=35):
             mb.run_direct(duty_cycle_sp= 0)
             mc.run_direct(duty_cycle_sp= 0)

        elif(lightValue>=21 or True):  #LIGHT SEEKING PID
            currentLightValue= ls1.value()
            if(currentLightValue>=maxLightvalue): 
                maxLightvalue=currentLightValue
            lightError= (maxLightvalue-currentLightValue)
            debug_print(lightError)
            dt= time.time()-startTime # dt is the delta time since the program started running.
            #debug_print(us2.value())
            integral= ((1/100)*integral)+(lightError*dt) #the integral is the sum of all errors over time, reduced by 1/3rd every tick so it doesn't go out of control
            #debug_print(dt)
            derivitive= lightError-lastLightError # the derivitive is the estimated next error based on the last error and the current error.
            p= (kpLight*lightError)+(kiLight*integral)+(kdLight*derivitive) #if error is 0, do not turn, P is the total rate of turn
            p=p/10
            #debug_print('error is: '+str(error))
            #debug_print('integral: ' +str(integral))
            #debug_print('derivitive is: '+str(derivitive))
            #debug_print('toatl is: '+str(p))       
            mbMove=tp+p#-(abs(error)/10)
            mcMove=tp-p#-(abs(error)/10)

            if(mbMove>=100): #setting move caps (at 100% power)
                mbMove=100
            if(mbMove<=-100):
                mbMove=-100
            if(mcMove<=-100):
                mcMove=-100
            if(mcMove>=100):
                mcMove=100 
            mb.run_direct(duty_cycle_sp= -mbMove)
            mc.run_direct(duty_cycle_sp= -mcMove)
            # debug_print(mbMove)
            if(integral>1000): #capping integral
                integral=1000
            if(integral<-1000):
                integral=-1000

        elif(avoidError==0):#Romeing DOES NOT WORK! BORKS AVOIDANCE
            mbMove=tp
            mcMove=tp
            while(i<600):
                mb.run_direct(duty_cycle_sp= -mbMove)
                mc.run_direct(duty_cycle_sp= -mcMove)
                i=i+1
            if(i>=600 and i<660):
                mbMove=tp*romeObjDirection
                mcMove=tp*romeObjDirection
                mb.run_direct(duty_cycle_sp= mbMove)
                mc.run_direct(duty_cycle_sp= -mcMove)
                i=i+1
            elif(i>=660):
                i=0
       

                
    lastError=error
    lastLightError=lightError
    

   # while True:
    #    time.process_time() 
     #   ds = us3.value()
      #  error = ds - offset 
       # integral = integral + error
   #     print(integral)
    #    derivative = error - lastError
     #   print(derivative)
      #  change = kp*error + ki*integral + kd*derivative
       # change = change/100
   #     delta1=sp+change
    #    delta2=sp-change
       
        #if(ds=offest):
       # mb.run_direct(duty_cycle_sp=delta1)
        #mc.run_direct(duty_cycle_sp=delta2)
      #  else:
          #  mb.run_direct(duty_cycle_sp=0)
         #   mc.run_direct(duty_cycle_sp=0)

if __name__ == '__main__':
    main()


  
    
    #getting values for the Ziegler–Nichols Method
    #kc= kp, when kp follows the line, but gives frequent non-crazy oscillation  (other values set to 0)
    #pc = the amount of time it takes for the robot to go from perigee-apogee-perigee (perigee to perigee) in the oscillation
    #dt=the amount of time it takes for the robot to cycle through the loop once (run the loop 10,000 times, have the bot count in realtime, and divide)
    
