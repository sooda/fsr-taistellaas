/**

This file is part of MaCI/GIMnet.

MaCI/GIMnet is free software: you can redistribute it and/or modify it 
under the terms of the GNU Lesser General Public License as published 
by the Free Software Foundation, either version 3 of the License, or 
(at your option) any later version.

MaCI/GIMnet is distributed in the hope that it will be useful, but WITHOUT 
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public 
License for more details.

You should have received a copy of the GNU Lesser General Public 
License along with GIMnet. (See COPYING.LESSER) If not, see 
<http://www.gnu.org/licenses/>.

**/
/**
 * \file
 * \brief J2B2 Control Demonstration using the J2B2ClientInterface.
 * \author Antti Maula <antti.maula@tkk.fi>
 */
#include "J2B2Demo.hpp"
#include "SLAM.hpp"
#include "owndebug.h"
#include <math.h>
#include <signal.h>


// SDL includes
#ifndef _DONT_USE_SDL_
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_gfxPrimitives.h>
#endif

//*****************************************************************************

CJ2B2Demo::CJ2B2Demo(CJ2B2Client &aInterface)
  : CSync(1,1), // One ConditionLock, one Mutex.
    CThread(5), // Maximum of 5 threads.
    iInterface(aInterface), // Pass in & store Interface reference
    iDemoActive(false), // Mark demo to be inactive at begin.
    iPTUDemoActive(false),
    iSensorsThreadActive(false),
    iInfoThreadActive(false),
    iMotionThreadActive(false),
    iCameraThreadActive(false),
    iSDLThreadActive(false),
    iCurrentSensorEvent(KSensorEventAllClear), // Sensors seem clear at begin.
    iSmallestDistanceToObject(),
    iLastCameraImage(),
    iLastLaserDistanceArray()
{
}
//*****************************************************************************

CJ2B2Demo::~CJ2B2Demo()
{
  Terminate();
}
//*****************************************************************************

void CJ2B2Demo::Execute()
{
  // Execute only if demo not active.
  if (!iDemoActive) {
    // Flag to mark 'Master' DemoActive (controls threads)
    iDemoActive = true;
    
    // Start execution of parallel thread(s) (Base class name is not
    // required, it is just added in the demo for clarification
    // purposes)
    dPrint(1,"Executing demo threads...");
    CThread::RunThread(KThreadSensorsDemo);
    CThread::RunThread(KThreadInfoDemo);
    CThread::RunThread(KThreadCameraDemo);
    CThread::RunThread(KThreadSDLDemo);
  }
}
//*****************************************************************************

void CJ2B2Demo::Terminate()
{
  if (iDemoActive) {
    iDemoActive = false;
    dPrint(1,"Terminating threads...");
    if (iCameraThreadActive)  CThread::WaitThread(KThreadCameraDemo);
    if (iMotionThreadActive)  CThread::WaitThread(KThreadMotionDemo);
    if (iInfoThreadActive)    CThread::WaitThread(KThreadInfoDemo);
    if (iSensorsThreadActive) CThread::WaitThread(KThreadSensorsDemo);
    if (iSDLThreadActive)     CThread::WaitThread(KThreadSDLDemo);
    dPrint(1,"All Threads terminated succesfully.");
  }
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

int CJ2B2Demo::RunInfoDemo(int aIterations)
{
  int iterations = 0;
  int posSeq = -1;
  ownTime_ms_t tbegin = ownTime_get_ms();
  using namespace MaCI::Position;
  using namespace MaCI;

  if (iInfoThreadActive) {
    dPrint(1,"InfoDemo already active! Will not start again.");
    return -1;

  } else {
    dPrint(3,"InfoDemoThread starting...");
    iInfoThreadActive = true;

  }

  // This prints Odometry output every 5 seconds.
  if (iInterface.iPositionOdometry) {
    while(iDemoActive && 
          iInfoThreadActive && 
          (aIterations == -1 || iterations < aIterations)) {
      CPositionData pd;
      if (iInterface.iPositionOdometry->GetPositionEvent(pd,
                                                     &posSeq,
                                                     1000)) {
        const TPose2D *pose = pd.GetPose2D();
        if (pose && ownTime_get_ms_since(tbegin) > 5000) {
          dPrint(1,"Odometry position now at: x:%f, y:%f, a: %frad",
                 pose->x, pose->y, pose->a);
          tbegin = ownTime_get_ms();
        }
      } else {
        dPrint(1,"No event available!");
        
      }
      
      ++iterations;
    }
  } else {
    dPrint(1,"No PositionOdometry available. Nothing to do. Abort Infothread.");

  }
  
  // Mark thread terminated.
  iInfoThreadActive = false;

  dPrint(3,"InfoDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunSDLDemo(int aIterations)
{ 
  if (iSDLThreadActive) {
    dPrint(1,"SDLDemo already active! Will not start again.");
    return -1;

  } else {
    dPrint(3,"SDLDemoThread starting...");
    iSDLThreadActive = true;

  }



#ifndef _DONT_USE_SDL_
  int iterations = 0;
  SDL_Surface *screen = NULL;
  const int window_width = 1100;
  const int window_height = 768;

  // Manual Speed control
  ownTime_ms_t speedcommand_last_sent = ownTime_get_ms();
  float r_speed = 0.00, r_wspeed = 0.00, r_acc = 0.20;

  // Manual PTU control
  ownTime_ms_t ptucommand_last_sent =  ownTime_get_ms();
  float ptu_pan = 0.00, ptu_tilt = 0.00;
  float ptu_pan_delta = 0.0, ptu_tilt_delta = 0.0;

  /////////////////////////////////////////////////////////////////////////////
  // Initialize SDL
  if(SDL_Init(SDL_INIT_VIDEO) == -1){
    dPrint(1,"Failed to initialize SDL Video: %s\n", SDL_GetError());
    exit(1);
  }
  
  // Register cleanup function for SDL
  atexit(SDL_Quit);
  
  // Initialize SDL-Video in mode WW*WH, 32bpp, & some flags.
  if ((screen = SDL_SetVideoMode(window_width, window_height, 32,
				 SDL_HWSURFACE|SDL_DOUBLEBUF|SDL_RESIZABLE)) == NULL) {
    dPrint(1, "Unable to set video mode for SDL Context: %s\n", SDL_GetError());
    abort();
  }
  
  // Clear the screen
  SDL_FillRect(screen, NULL, 0);

  // Set imporant info :)
  SDL_WM_SetCaption("Simple J2B2 Demo application","J2B2Demo");
  

  // Now, run the loop of processing as long as the demo is alive.
  while(iDemoActive && 
        iSDLThreadActive && 
        (aIterations == -1 || iterations < aIterations)) {
    SDL_Event event;
    
    // Wait for incoming event
    while (SDL_PollEvent(&event)) {
      
      // Switch based on event type.
      switch (event.type) {
        
        // QUIT ('Window close' clicked)
      case SDL_QUIT: {
        // close button clicked
        dPrint(1,"'Window close' clicked.");
        iDemoActive = false;
        ownSleep_ms(1000);
        raise(SIGINT);
        raise(SIGINT);
        raise(SIGTERM);
        ownSleep_ms(1000);
        raise(SIGKILL);
        break;
      }
        
        // Any key down
      case SDL_KEYDOWN: {
        // Switch based on the key that was pressed
        switch (event.key.keysym.sym) {
          
          /////////////////////////////////////////////////////////////
          // Perform simple robot control (just set the current values
          /////////////////////////////////////////////////////////////
        case SDLK_UP:
          // 'w' pressed - set speed to 'forward 0.3m/s'
          r_speed = 0.30;
          break;
        case SDLK_LEFT:
          // 'a' pressed - set speed 'turn left M_PI rad/s'
          r_wspeed = M_PI;
          break;
        case SDLK_DOWN:
          // 's' pressed - set speed to 'backwards 0.3m/s'
          r_speed = -0.30;
          break;
        case SDLK_RIGHT:
          // 'd' pressed - set speed 'turn right M_PI rad/s'
          r_wspeed = -M_PI;
          break;
          
          ///////////////////////////////////////////////////////////
          // Perform PTU control
          ///////////////////////////////////////////////////////////
        case SDLK_w:
          // 'w' pressed
          ptu_tilt_delta = 1.0;
          break;
        case SDLK_a:
          // 'a' pressed
          ptu_pan_delta = 1.0;
          break;
        case SDLK_s:
          // 's' pressed
          ptu_tilt_delta = -1.0;
          break;
        case SDLK_d:
          // 'd'-pressed
          ptu_pan_delta = -1.0;
          break;

         // Behaviour control example
        case SDLK_e:
          dPrint(1,"Enable Emergency behaviours");
          iInterface.iBehaviourCtrl->SetStart();
          break;
          
        case SDLK_r:
          dPrint(1,"Disable Emergency behaviours");
          iInterface.iBehaviourCtrl->SetStop();
          break;

          // Control Motion demo
        case SDLK_m:
          if (iMotionThreadActive) {
            dPrint(1,"Terminating MotionDemo...");
            iMotionThreadActive = false;
            CThread::WaitThread(KThreadMotionDemo);
            dPrint(1,"Terminated.");

          } else {
            dPrint(1,"Executing MotionDemo..");
            CThread::RunThread(KThreadMotionDemo);

          }
          break;

          // Control PTU demo.
        case SDLK_p:
          if (iPTUDemoActive) {
            dPrint(1,"Terminating PTUDemo...");
            iPTUDemoActive = false;
            dPrint(1,"Terminated.");
            
          } else if (iCameraThreadActive) {
            dPrint(1,"Executing PTUDemo..");
            iPTUDemoActive = true;

          } else {
            dPrint(1,"Executing CameraThread & PTUDemo..");
            iPTUDemoActive = true;
            CThread::RunThread(KThreadCameraDemo);
            
          }
          break;

        case SDLK_1: {
          // Set All outputs HIGH.
          // Calling SetPin for all pins SEPARATELY. If you need more
          // synchronous & faster control between different outputs,
          // use the 'SetDigitalMask()' function instead.
          if (iInterface.iIOBoardESC != NULL) {
            for(unsigned int pin = 8; pin < 16; ++pin) {
              iInterface.iIOBoardESC->SetDigitalPin(pin, true);
            }
          }
          dPrint(1,"Set all outputs on ESC to HIGH state");
          break;
        }

        case SDLK_0: {
          // Set all outputs LOW.
          // Calling SetPin for all pins SEPARATELY. If you need more
          // synchronous & faster control between different outputs,
          // use the 'SetDigitalMask()' function instead.
          if (iInterface.iIOBoardESC != NULL) {
            for(unsigned int pin = 8; pin < 16; ++pin) {
              iInterface.iIOBoardESC->SetDigitalPin(pin, false);
            }
          }
          dPrint(1,"Set all outputs on ESC to LOW state");
          break;
        }


          
          ///////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////

        default:
          // Default, print events
          dPrint(1,"SDL returned KEYDOWN event for '%s'" , SDL_GetKeyName(event.key.keysym.sym));
          break;
        }
        break; // SDL_KEYDOWN - end
      }
        
      case SDL_KEYUP: {
        switch (event.key.keysym.sym) {
        case SDLK_UP:
        case SDLK_DOWN:
          r_speed = 0.00;
          break;
        case SDLK_LEFT:
        case SDLK_RIGHT:
          // In every case of the controls, UP == SetSpeed(0.0, 0.0);
          r_wspeed = 0.00;
          break;
          
        case SDLK_w:
        case SDLK_s:
          ptu_tilt_delta = 0.00;
          break;

        case SDLK_a:
        case SDLK_d:
          ptu_pan_delta = 0.00;
          break;

        case SDLK_0:
        case SDLK_1:
          // No action on IO ups.
          break;

        case SDLK_e:
        case SDLK_r:
        case SDLK_m:
        case SDLK_p:
          // No action for Command keys on UP.
          break;

        default:
          dPrint(1,"SDL returned KEYUP event for '%s'" , SDL_GetKeyName(event.key.keysym.sym));
          break;
        }
        break; // SDL_KEYUP - end
      }
        
        // Default handler. Nothing done.
      default:
        break; // default - end
      }
    }
    

    // Then, send the current speed commands to Robot (it needs
    // refreshing). But ONLY; if the demo is not active, and last
    // command was sent more than 200 ms ago (too rapid sending rate
    // will clutter the connection and everything will start to go
    // wrong)
    if (iMotionThreadActive == false && 
        ownTime_get_ms_since(speedcommand_last_sent) > 200) {
      // Only send if the motion demo is not active (it is a
      // standalone demo controlling the robot, if it is active,
      // sending commands to robot would only mess up things)
      iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
      speedcommand_last_sent = ownTime_get_ms();
    }


    // Then, apply the PTU manual control parameters.
    if (iPTUDemoActive == false &&
        ownTime_get_ms_since(ptucommand_last_sent) > 100) {
      // Do movement
      ptu_pan += ptu_pan_delta * M_PI/20;
      ptu_tilt += ptu_tilt_delta * M_PI/20;
      
      // Do value checking and limiting.
      if (ptu_pan > M_PI/2) ptu_pan = M_PI/2;
      else if (ptu_pan < -M_PI/2) ptu_pan = -M_PI/2;
      if (ptu_tilt > M_PI/2) ptu_tilt = M_PI/2;
      else if (ptu_tilt < -M_PI/2) ptu_tilt = -M_PI/2;

      // Do control
      iInterface.iServoCtrl->SetPosition(ptu_pan, KServoCameraPTUPan);
      iInterface.iServoCtrl->SetPosition(ptu_tilt, KServoCameraPTUTilt);
      ptucommand_last_sent = ownTime_get_ms();
    }

    // Lock class mutex.
    Lock();


    // Clear the screen.
    SDL_FillRect(screen, NULL, 0);

    // Now, draw the last received Image.
    if (iLastCameraImage.GetImageDataType() == MaCI::Image::KImageDataJPEG &&
        iLastCameraImage.GetImageDataPtr() != NULL) {
      
      /////////////////////////////////////////////////////////////////////////////
      // Draw the last imageimage! (Don't care whether its new or not)
      /////////////////////////////////////////////////////////////////////////////
      SDL_Surface *image;
      SDL_RWops *rw;
      SDL_Rect rcDest = {0,0,0,0};
      
      rw = SDL_RWFromMem((void*)iLastCameraImage.GetImageDataPtr(), 
                         iLastCameraImage.GetImageDataSize());
      image = IMG_LoadJPG_RW(rw);
      SDL_BlitSurface(image, NULL, screen, &rcDest);
      SDL_FreeSurface(image);
    }
   
	// let's see if this works
	RobotLocation loc = RobotLocation(0,0,0);
	SLAM slam = SLAM(1,1,1,1,loc,iLastLaserDistanceArray);
	slam.drawLaserData(screen, window_width, window_height);

	/* 
    // Draw the LaserScan
    if (iLastLaserDistanceArray.size()) {
      float min_d = 1000;
      float scale = 160; ///< scales from meters to screen pixels
      int min_x_end = 0;
      int min_y_end = 0;
      const int x_origin = window_width/2; ///< origin of laser measurement
      const int y_origin = window_height; ///< origim of laser meas
      const int dasize = iLastLaserDistanceArray.size();
      for(int i = 0; i < dasize; ++i) {
        const MaCI::Ranging::TDistance &measurement = iLastLaserDistanceArray[i];
        int pix_x = x_origin - (int)(scale * measurement.distance * sin(measurement.angle)); 
        int pix_y = y_origin - (int)(scale * measurement.distance * cos(measurement.angle)); 
        if (pix_x >= 0 && pix_x < window_width && 
            pix_y >= 0 && pix_y < window_height) {
          pixelRGBA(screen, pix_x, pix_y, 255, 0, 0, 150);
          if (measurement.distance < min_d) {
            min_d = measurement.distance;
            min_x_end = pix_x;
            min_y_end = pix_y;
          }
        }
      }
      filledCircleRGBA(screen, x_origin, y_origin,
                       (int)(0.2 * scale), 0, 255, 255, 255);
      
      //Example for drawing a line using SDL: (The -1 in X origin is
      //to lift the line above border, so lines drawn directly towards
      //X axis would be visible too)
      lineRGBA(screen,
               x_origin, 
               y_origin-1,
               min_x_end,
               min_y_end,
               255, 255, 0, 255);
      
    }
    
	*/

    // Print help
#define HELPSTRCOUNT 7
    const char helpstr[HELPSTRCOUNT][60] = { 
      "m - Toggle MotionDemo",
      "p - Toggle PTU demo",
      "e - Enable Emergency actions",
      "r - Disable Emergency actions",
      "1,0 - Set all IO outputs on ESC to HIGH/LOW",
      "w,a,s,d - Control PTU manually (PTUDemo must be OFF)",
      "arrows - Drive around manually (MotionDemo must be OFF)",
    };
    int help_i = 0;
    int text_x = 650;
    int text_y = 5;
    const int text_y_delta = 10;

    for(help_i = 0; 
        help_i < HELPSTRCOUNT;
        text_y += text_y_delta, ++help_i) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 helpstr[help_i], 
                 255, 0, 0, 150);
    }

    // print status reports
    text_y += text_y_delta;
    char diststr[255];
    sprintf(diststr, "Smallest distance to object: %.2fm at %.3f rad", 
            iSmallestDistanceToObject.distance, 
            iSmallestDistanceToObject.angle);
    
    stringRGBA(screen, 
               text_x, text_y, 
               diststr,
               0, 255, 255, 150);
    text_y += text_y_delta;

    if (iInterface.iIOBoardESC != NULL) {
      int m = iInterface.iIOBoardESC->GetDigitalMask(0, 1000);
      char bitstr[255];
      sprintf(bitstr, "IO port state now: %08x", m);
      
      stringRGBA(screen, 
                 text_x, text_y, 
                 bitstr,
                 0, 255, 255, 150);
      text_y += text_y_delta;
    }
    
    if (iMotionThreadActive) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 "MotionDemo is active!", 
                 0, 255, 0, 150);
      text_y += text_y_delta;
    }
    if (iPTUDemoActive) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 "PTUDemo is active!", 
                 0, 255, 0, 150);
      text_y += text_y_delta;
    }

    // Flip the double buffered screen. (Display the newly rendered screen)
    SDL_Flip(screen);

    // Unlock whole class mutex.
    Unlock();

    // Cut some slack :)
    ownSleep_ms(10);
  }

#endif

  dPrint(3,"SDLDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunCameraDemo(int aIterations)
{
  bool r;
  using namespace MaCI::Image;
  CImageData imgData;
  unsigned int imgSeq         = 0;
  float panAngleRad           = -M_PI/2.0;
  float tiltAngleRad          = -M_PI/10.0;
  int panStepCurrent          = 0;
  const int panSteps          = 10;
  float panStepSizeRad  = ( M_PI / 10 );

  if (iCameraThreadActive) {
    dPrint(1,"CameraDemo already active! Will not start again.");
    return -1;
    
  } else {
    dPrint(3,"CameraDemoThread starting...");
    iCameraThreadActive = true;

  }

  int iterations = 0;
  while(iDemoActive && 
        iCameraThreadActive && 
        (aIterations == -1 || iterations < aIterations)) {
    
    // Is ServoCtrl available, and is ServoDemo active?
    if (iInterface.iServoCtrl && 
        iPTUDemoActive) {

      // We have a fixed number of steps. 
      if (panStepCurrent < panSteps) {
        panAngleRad += panStepSizeRad;
        panStepCurrent++;
        
      } else { 
        panStepCurrent = 0;
        panStepSizeRad = -panStepSizeRad;
      }
      
      
      // Command Servo to move. (Move servos attached to Camera PTU)
      r = iInterface.iServoCtrl->SetPosition(panAngleRad, KServoCameraPTUPan);
      r &= iInterface.iServoCtrl->SetPosition(tiltAngleRad, KServoCameraPTUTilt);

      if (r) {
        float ppos,tpos;
        iInterface.iServoCtrl->GetPosition(KServoCameraPTUPan, ppos, 500);
        iInterface.iServoCtrl->GetPosition(KServoCameraPTUTilt, tpos, 500);
        dPrint(3,"Camera now pointing to pan:%.3f, tilt:%.3f",
               ppos, tpos);
        
        // Wait to stabilize
        ownSleep_ms(200);

      } else {
        // Some failure?
        ownSleep_ms(1000);

      }
      
    } else {
      dPrint(8,"No ServoCtrl available - skipping");

    }
    

    if (iInterface.iImageCameraFront) {
      
      // Read last available Image. This function reads a image from
      // the camera and stores it to the provided 'imgData'
      // object. The 'imgSeq' is the number to expect. In normal
      // processing, user should never change value of this variable
      // after starting to receive images. Note: the 'imgSeq' variable
      // is modified BY the 'GetImageData' function to reflect the
      // latest received imagedata sequence. See description of
      // 'GetImageData' function for more details.
      r = iInterface.iImageCameraFront->GetImageData(imgData, &imgSeq);

      if (r) {

        // COPY the received imagedata to class variable; for (possibly)
        // using it from another thread.
        
        // Lock the whole J2B2Demo instance. We are only using a single
        // lock for all shared operations for simplicity. You should
        // consider using multiple Locks depending on your application
        // implementation
        Lock();
        
        // The 'GetImage' function call of the CImageData class returns
        // the REAL data element from the CImageData container. The
        // resulting data is stored to image container class names
        // 'CImageContainer'. This class is a multi-function container
        // which is for example; able to convert the given data between
        // various formats. For example, the J2B2 always sends the data
        // in JPG format, but by using the CImageContainer classes
        // 'ConvertTo()' function, you can attempt to convert the data
        // to different format before using it.
        if (imgData.GetImage(iLastCameraImage, NULL, true) != true) {
          dPrint(1,"Failed to read ImageContainer data?");
        }
        
        // Release the previously locked class instance.
        Unlock();
      }
    } else {
      dPrint(5,"No ImageCameraFront available - skipping (sleep 1000ms)");
      ownSleep_ms(1000);
      
    }
    
    // Increment.
    iterations++;
  }

  dPrint(3,"CameraDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunMotionDemo(int aIterations)
{
  const float angspeed = M_PI / 2.0;
  const unsigned int turn_duration = 5000;
  bool dirLeft = true;
  ownTime_ms_t tbegin;

  if (iMotionThreadActive) {
    dPrint(1,"MotionDemo already active! Will not start again.");
    return -1;
    
  } else {
    dPrint(3,"MotionDemoThread starting...");
    iMotionThreadActive = true;

  } 

  // Verify that behaviours are ON!  
  //
  // \note When the behaviours are ON, the robot stops on bumper hit,
  // if they are OFF, the robot will try to keep going until breaking
  // something!
  iInterface.iBehaviourCtrl->SetStart();

  int iterations = 0;
  while(iDemoActive && 
        iMotionThreadActive && 
        (aIterations == -1 || iterations < aIterations)) {
    
    // Got MotionCtrl?
    if (iInterface.iMotionCtrl) {
      
      // Change direction.
      dirLeft ^= true;
      dPrint(2,"Turning %s for %u milliseconds",
             dirLeft ? "left" : "right", 
             turn_duration);

      // Run in loop, because SpeedCtrl requires continuous input (min interval < 500ms)
      tbegin = ownTime_get_ms();
      do {
        // Send 'Turning' command.
        iInterface.iMotionCtrl->SetSpeed(0.00, // Dont drive (or drink)
                                         dirLeft ? angspeed : -angspeed, // Turn at angspeed rad/s
                                         0.1); // Acceleration limit 0.1 m/s�
        
        // Sleep the turn_duration slice.
        ownSleep_ms(MIN(200,ownTime_get_ms_left(turn_duration, tbegin)));
      } while(ownTime_get_ms_left(turn_duration, tbegin) > 0);

      // 4. Stop!
      dPrint(2,"STOP!");
      iInterface.iMotionCtrl->SetStop();

    } else {
      dPrint(1,"No MotionCtrl available - Terminating MotionDemo thread.");
      break;

    }

    // 5. Increment iteration counter
    ++iterations;

    // 6. Round complete, we should be near an obstacle now, so enter
    // random again and Redo!
    dPrint(1,"Round complete. Robot is stopped. %d iterations executed",
           iterations);
  }


  dPrint(3,"MotionDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunSensorsDemo(int aIterations)
{
  using namespace MaCI;
  using namespace MaCI::Ranging;
  const float proximityAlertLimit = 0.30;
  int iterations = 0;


  TDistanceArray laserDistanceData;
  TDistanceArray bumperDistanceData;
  TDistanceHeader laserHeader;
  TDistanceHeader bumperHeader;
  Common::TTimestamp laserTimestamp;
  Common::TTimestamp bumperTimestamp;
  int bumperSeq = -1;
  int laserSeq = -1;

  // This is an example sequence only - it doesn't correctly use the
  // bumper and laser together, as there can be event on both
  // simultaneously.
  while(iDemoActive && (aIterations == -1 || iterations < aIterations)) {

    bool r;
    if (iInterface.iRangingBumpers) {
      r = iInterface.iRangingBumpers->GetDistanceArray(bumperDistanceData, 
                                                       &bumperHeader, &bumperTimestamp, &bumperSeq, 
                                                       0);
    } else {
      r = false;
      dPrint(8,"No iRangingBumpers available - skipping");

    }

    // Check status from bumpers
    if (r) {
      // If this branch is reached, there was a bumperevent available.
      for(EACH_IN_i(bumperDistanceData)) {
        if (i->distance >= 0.00) {
          dPrint(2,"Bumpers hit: %f meters @ %f rad", i->distance, i->angle);
          iSmallestDistanceToObject = *i;
        }
      }
    }
    
    // Waiting for Laser measure for maximum of 100ms. If you are
    // only reading data from Laser, you could make this longer -
    // too fast polling consumes processing time and gains nothing.
    
    if (iInterface.iRangingLaser) {
      r = iInterface.iRangingLaser->GetDistanceArray(laserDistanceData, 
                                                     &laserHeader, &laserTimestamp, &laserSeq, 
                                                     100);
      // Check laser status
      if (r) {
        // Copy distances
        Lock();
        iLastLaserDistanceArray = laserDistanceData;
        Unlock();
        
        // Check distances received.
        //
        // The "EACH_IN_i" is a macro using C++ iterators which operates
        // so that 'i' variable is set to point to each element in the
        // provided array in turn. When using it as
        // 'for(EACH_IN_i(<array>))' you can easily access all the
        // elements through the 'i' variable.
        iSmallestDistanceToObject.distance = 1000;
        for(EACH_IN_i(laserDistanceData)) {
          if (i->distance < iSmallestDistanceToObject.distance &&
              i->distance > 0) iSmallestDistanceToObject = *i;

          if (i->distance <= proximityAlertLimit && i->distance >= 0) {
            // Proximity alert.
            dPrint(3,"Proximity alert at direction %.3frad CCW from fw direction, %.3f meters free!",
                   i->angle,
                   i->distance);
          }
        }
      }
    } else {
      r = false;
      dPrint(8,"No iRangingLaser available - skipping (sleep 100ms)");
      ownSleep_ms(100);
    }
    
    // Increment
    ++iterations;
  }
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::ThreadFunction(const int aThreadNumber)
{
  int result = 0;
  switch(aThreadNumber) {
  case KThreadSensorsDemo:
    result = RunSensorsDemo(-1);
    break;

  case KThreadInfoDemo:
    result = RunInfoDemo(-1);
    break;

  case KThreadMotionDemo:
    result = RunMotionDemo(-1);
    break;
    
  case KThreadCameraDemo:
    result = RunCameraDemo(-1);
    break;

  case KThreadSDLDemo:
    result = RunSDLDemo(-1);
    break;

  default:
    dPrint(1,"WARNING: Invalid thread Number specified! (No implementation!)");
    abort();
    break;
  }
  return result;
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
