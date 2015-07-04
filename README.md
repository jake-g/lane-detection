# Lane Detection
##### C++ program that identifies and tracks lanes and their intersection. Can be used for autonomous driving features such as lane changing, blindspot detection, hill crest detection, turn detection and sign recognition. Works on input video, image or live video feed.

### Initial Goal:
Detect lanes and their respective intersection point on the horizon in order to track the point’s movement to determine if the vehicle is turning or changing slope.

### Demonstration:
* Input Video:Pikes Peak race track (on road and off road)
* [Youtube Demo](https://www.youtube.com/watch?v=-vmnam3ergc)

### Features
* Language: C++ (OpenCV)
* IDE: Xcode
* Signal Processing:
..* Canny edge detector
..* Hough line detector
..* Slope and turn tracking(incomplete)

![alt text](https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Logo Title Text 1")



BLOCK DIAGRAM

Edge Detection Implementation Basics
1. Apply Gaussian filter to smooth the image in order to remove the noise
2. Find the intensity
gradients of the image
Line Detection Implementation Basics
1. Extract edge points and map to lines by representing in polar form
2. Store lines in an accumulator
3. Find stored lines of infinite length using thresholding and filtering
4. Convert infinite lines to finite lines

### Requirements
Python: `pafy.py` <br>
Bash: `eyeD3` ; `curl`

### How to use
1. `git clone` to path of choice
2. `cd` to path
3. run `python audio-dl.py`
4. Enter valid soundcloud or youtube url
5. enter valid save path
6. The stream will download in the highest available quality

### License
[GPL v2](https://www.gnu.org/licenses/gpl-2.0.txt) <br>
Created by [Jake Garrison](https://github.com/jake-g) and Brian Magnuson, 2015 <br>
Thanks to [Luka Pusic](http://pusic.si) for making [scdl](https://github.com/lukapusic/soundcloud-dl) which I based my program off of.
