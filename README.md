# Lane Detection
##### OpenCV C++ program that identifies and tracks lanes and their intersection. Can be used for autonomous driving features such as lane changing, blindspot detection, hill crest detection, turn detection and sign recognition. Works on input video, image or live video feed.

### Initial Goal:
Detect lanes and their respective intersection point on the horizon in order to track the point’s movement to determine if the vehicle is turning or changing slope.

### Demonstration:
* Input Video:Pikes Peak race track (on road and off road)
* [Youtube Demo](https://www.youtube.com/watch?v=-vmnam3ergc)

### Features
* Language: C++ (OpenCV)
* IDE: Xcode
* Signal Processing:
  * Canny edge detector
  * Hough line detector
  * Slope and turn tracking(incomplete)
  * Customizable ROI

### Requirements
* `OpenCV`
* `g++`
* video or image file, or video feed
* (optional) xcode to utilize the included project file

### How to use
I have only worked with this on osx so you may be on your own!
1. `brew install opencv`
2. Set path to input file in `LaneDetect.cpp`
3. Configure options in `LaneDetect.cpp`
  * `houghVote` determines the fidelity of the line recognition (higher means more lines kept)
  * Under the `// Set up windows` you can enable or disable windows of your choice
  * Also may be necessary to modify the ROI (region of intrest) to best capture the road
  * Can uncomment the `VideoWriter` line to write the output to file
4. Run program
  * Either using the xcode project file
  * OR `g++ $(pkg-config --cflags --libs opencv) LaneDetect.cpp` (this works on my system, but may vary. It should run on `g++` somehow!)

### Implementation
###### Block Diagram
![alt text](https://github.com/jake-g/lane-detection/raw/master/pics/block%20diagram.png "Final Result")

###### Original Frame
![alt text](https://github.com/jake-g/lane-detection/raw/master/pics/original.png "Original frame")

#### Edge Detection Implementation Basics
1. Apply Gaussian filter to smooth the image in order to remove the noise
2. Find the intensity gradients of the image

###### Canny Edge Detection
![alt text](https://github.com/jake-g/lane-detection/raw/master/pics/result.png "Edge Detection")

#### Line Detection Implementation Basics
1. Extract edge points and map to lines by representing in polar form
2. Store lines in an accumulator
3. Find stored lines of infinite length using thresholding and filtering
4. Convert infinite lines to finite lines

###### Line Detection result
![alt text](https://github.com/jake-g/lane-detection/raw/master/pics/result.png "Final Result")

#### Notes
Add up number on lines that are found within a threshold of a given rho,theta and
use that to determine a score.  Only lines with a good enough score are kept.

Calculation for the distance of the car from the center.  This should also determine
if the road in turning. Might not want to be in the center of the road for a turn.

Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
type of feed back loop might be good to self tune these parameters.

Added filter on theta angle to reduce horizontal and vertical lines.
Added image ROI to reduce false lines from things like trees/powerlines

#### Further Reading
See included `Documentation.pdf` <br>
General idea and some code modified from:
chapter 7 of Computer Vision Programming using the OpenCV Library
by Robert Laganiere, Packt Publishing, 2011.

### License
[GPL v2](https://www.gnu.org/licenses/gpl-2.0.txt) <br>
Created by [Jake Garrison](https://github.com/jake-g) and Brian Magnuson, 2015 <br>
