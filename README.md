# opencv_controller
Game Controller using OpenCV | Make Physical Therapy (Physiotherapy) Exciting 

Playing games like Need for Speed Most Wanted (NFS) using OpenCV (NO Keyboard required). Paper acts as a controller. (Works with any racing games and like GTA Vice City (tested)).

Usecase:
1)  Making Physiotherapy exercise exciting like arm exercises, thumb movements (Thumb Extension and Flexion)
2)  Replacement of Kinect which can cost more than $200 (Rs. 14000+)
2)  Being active
3)  Controller is free of cost 

Video:
0:00 Start of Video (purposely driven into the city to check the turning of the car (based on the angle))
1:30 Shows results at the end (not accurate now, need to work on that)
1:35 Pages used to control the game
1:38 2nd UI to just play games and no exercise

Paper:
Green: Forward and angle
Yellow: #Nitrous
Blue: Brakes and backwards

Working:
OpenCV will detect the color through #webcam, calculates the angle of rotation which is used for calculating the hand movements and direction of the car. 

Special thanks to Aaron Callard and Hatem Abou-zeid, PhD for guidance to use OpenCV efficiently. 

I will be uploading code in this week on #GitHub.

Please give feedback or suggestions to improve the experience. Thanks a lot :)

Credits:
OpenCV : https://opencv.org/
EA Sports for Need For Speed Most Wanted (2005) : https://www.ea.com/games/need-for-speed/need-for-speed-most-wanted-2005
directkeys.py: https://stackoverflow.com/questions/14489013/simulate-python-keypresses-for-controlling-a-game

How to use it:
Clone the repository
Start the game (Racing Game)
Execute nfs.py file


