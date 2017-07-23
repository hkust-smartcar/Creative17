import processing.serial.*;

// The serial port:
Serial myPort;
int IMAGE_X = 100, IMAGE_Y = 100;
int CAM_W = 160, CAM_H = 120;
int PIXEL_UNIT = 4;
boolean [][] image = new boolean[CAM_H][CAM_W];

void printPixel(int x, int y, boolean c) {
  stroke(130);
  if (c) {
    //fill black
    fill(0);
  } else {
    //fill white
    fill(255);
  }
  rect(IMAGE_X + PIXEL_UNIT * x, IMAGE_Y + PIXEL_UNIT * y, PIXEL_UNIT, PIXEL_UNIT);
}

void setup() { //<>//
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  size(800, 800);
  fill(0);
  stroke(130);
  for (int i = 0; i < CAM_H; i++) {
    for (int j = 0; j < CAM_W; j++) {
      rect(IMAGE_X + PIXEL_UNIT * j, IMAGE_Y + PIXEL_UNIT * i, PIXEL_UNIT, PIXEL_UNIT);
    }
  }
  println("end of setup");
}  //end of setup

void draw() {
  //println("drawing");
  if (myPort.available() > 0) {
    int message = myPort.read();
    println(message);
    if (message == 170) {
      
      int counter = 0;
      int xCounter = 0, yCounter = 0;
      int imageByteSize = CAM_H * CAM_W / 8;
      while (counter < imageByteSize) {
        print('.');
        if (myPort.available() > 0) {
          message = myPort.read();
          
          //print 8 pixels horizontally for each byte
          for (int i = 0; i < 8; i++) {
            boolean pixelColor = (message >> (7-i) & 1) == 1;
            
            //redraw the image only when the pixel color changed
            if (image[yCounter][xCounter + i] != pixelColor) {
              image[yCounter][xCounter + i] = pixelColor;
              printPixel(xCounter + i, yCounter, pixelColor);
            }
          }
          
          xCounter += 8;
          
          //assume xCounter is a multiple of 8
          if(xCounter == CAM_W) {
            xCounter = 0;
            yCounter++;
          }
          counter++;
        }
      }  //end of while
      
    }  //end of the checking of the start of the image
  }  //end of the big if
}