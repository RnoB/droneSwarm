import time
import picamera
with picamera.PiCamera() as camera:
    camera.resolution = (1296,976)
    #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
    camera.framerate = 10
    camera.iso = 800
    camera.shutter_speed = 100000
    #camera.awb_mode = 'off'
    #camera.awb_gains=(1,1)
    #camera.exposure_speed = 100
    #camera.exposure_mode = 'night'
    camera.exposure_compensation = 25
    camera.start_preview()
    firstRound = True
    try:
        for i, filename in enumerate(camera.capture_continuous('./imTest/image{counter:02d}.jpg', use_video_port=True)):
            print(filename)
            print('analog gain : '+str(camera.analog_gain)+' digital_gain : '+str(camera.digital_gain))

            if camera.analog_gain >7 and camera.digital_gain > 1 and firstRound:
                
                camera.exposure_mode = 'off'
                camera.awb_mode = 'off'
                b=0
                r=0

                camera.awb_gains=(r,b)
                firstRound = False
            if not firstRound:
                r=r+1
                if r>8:
                    r=0
                    b=b+1
                if b>8:
                    break
                camera.awb_gains=(r,b)

            print('White balance gain : '+str(camera.awb_gains))
            #time.sleep(.1)
            if i == 400:
                break
    finally:
        camera.stop_preview()