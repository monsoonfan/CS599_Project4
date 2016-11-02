#############################################
# Simple makefile for execution of project3
#############################################
all: raytrace.c
	gcc raytrace.c -o raytrace

clean:
	rm raytrace.exe

# assuming VERBOSE is set to 1 in raytrace.c
verbose: raytrace.c
	gcc raytrace.c -o raytrace_v

#############################################
# targets for testing quadrics
#############################################
ellipsoid:
	gcc raytrace.c -o raytrace
	./raytrace 1000 800 ellipsoid.json ellipsoid.ppm

cylinder:
	gcc raytrace.c -o raytrace
	./raytrace 1000 800 cylinder.json cylinder.ppm

cone:
	gcc raytrace.c -o raytrace
	./raytrace 1000 1000 cone.json cone.ppm

#############################################
# targets for testing good and bad json files
#############################################
testgood.%:
#	ifeq ($(wildcard("test.ppm")),)
#		echo "it's there"
#	endif
#	rm test.ppm
#	fi
	gcc raytrace.c -o raytrace
	./raytrace 1000 800 test_good_$*.json test.ppm
	emacs -geometry 120x60 test.ppm

testbad.%:
	gcc raytrace.c -o raytrace
	./raytrace 600 600 test_bad_$*.json test.ppm

testall:
	make testgood.01
	make testgood.02
	make testgood.10
	make testgood.11
	make testgood.12

#############################################
# targets for simple test configs
#############################################
testexample:
	gcc raytrace.c -o raytrace
	./raytrace 1000 800 example.json test.ppm
	emacs -geometry 200x60 test.ppm
