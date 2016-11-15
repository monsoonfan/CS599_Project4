---------------------------------------------------------
# README - raytrace.c (for project 4)
# Mitchell Ralston - rmr5
# Reflectivity and Refractivity
---------------------------------------------------------

---------------------------------------------------------
# Application
Reads a JSON file full of scene objects (spheres, planes, quadrics are supported) and raycasts
them into an image file given as such:
---------------------------------------------------------

     usage:    raytrace <image width> <image height> <JSON input file> <P6 version PPM output file>
           width = number of pixels in the X direction
           height = number of pixels in the Y direction
     
     example:  raytrace 600 480 input.json output.ppm

---------------------------------------------------------
# Makefile targets of interest
The Makefile within the repository contains many targets. Most are for my own testing, but these
targets may be of interest for grading:
---------------------------------------------------------
       make all               (compile the program)
       make clean             (remove the current compiled program)

       make tg.project  (this will create the image given in example json from project assignment)
       
---------------------------------------------------------
# these are interesting, but will invoke emacs so I can view the result without
having to invoke GIMP or emacs separately
---------------------------------------------------------
       make tg.%  (will run a "good" JSON file, assuming the JSON with corresponding number exists)
       	    example:
		make tg.01  (will run on "test_good_01.json" with fixed 600x600 width height and test.ppm
		                   I have created many JSON files)
				   
       make testbad.%  (similar to testgood but for testing known error cases in JSON. I have 01-03 created)
