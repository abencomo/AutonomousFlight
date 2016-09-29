Steps to calibrate matrix vision camera:

1. Print out checkerboard
2. Take about 25 images of the checkerboard from different angles and distances
3. Create a folder 'mv_images' in the directory where calibrate.py is
4. Place the 25 images inside the mv_images folder. The images should be in *.jpg format. Change Line 39 in calibrate.py if using different format
5. Run calibrate.py to obtain the calibration parameters