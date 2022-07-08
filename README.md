# stero_cali_match

This project use opencv3.4.1 to realize the double camera's calibration and Disparity computation.

# environment
 - opencv3.4.10
 - vs-studio2019
 
# steps
1. use '拍照(单设备号).cpp' or '拍照(双设备号).cpp' to take a certain number of the calibration board.
2. use '标定.py' to realize stero-calibration, remember to adjust the parameter. The results and errors will be saved to 'caliResults.txt'.
3. use '匹配.py' to realize stero-match and disparity computation, showing the real-time disparity map.
