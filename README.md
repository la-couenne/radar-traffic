# Radar in traffic
![00](https://user-images.githubusercontent.com/38251711/119191240-a57cd780-ba7e-11eb-958f-fe609a34ed22.png)

Good morning all,

Here is a small assembly very practical for those who want to read a book in the traffic jams.

The principle is very simple, with OpenCV we detect the movements located in the center of the image captured by a webcam, if there is no movement for a few seconds it means that we are stopped, as soon as the vehicle in front of us begins to move, a beep is emitted.

# Demo:
https://youtu.be/C5yqAdc1bE0

# Install OpenCV:
'''shell
sudo apt-get install python-opencv
'''

# Sources:
https://opencv.org
