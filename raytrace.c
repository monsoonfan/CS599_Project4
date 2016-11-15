/*
---------------------------------------------------------------------------------------
CS599 Project 4
R Mitchell Ralston (rmr5)
11/1/16
--------------------------
Add reflection/refraction per Project4 as per assignment spec on BBLearn

Organization:
------------
variables are named with underscore naming convention
functions are named with camelCase naming convention

Questions:
---------
) do we render a light in a reflection? Take it's color (would need light intersection equation)
) error checking scenarios (what to do about missing rad/ang attenuations, for example?
---------------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <unistd.h>
 
#define DEFAULT_COLOR 0

// typdefs
typedef struct RGBPixel {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} RGBPixel ;

typedef struct RGBAPixel {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
} RGBAPixel ;

typedef struct PPM_file_struct {
  char magic_number;
  int width;
  int height;
  int alpha;
  int depth;
  char *tupltype;
  FILE* fh_out;
} PPM_file_struct ;

// coefficients of quadrics
typedef struct A_J {
  double A;
  double B;
  double C;
  double D;
  double E;
  double F;
  double G;
  double H;
  double I;
  double J;
} A_J ;

// these are flags which are set during parsing as values are stored, used for error checking the JSON only
typedef struct has_values {
  int has_width;
  int has_height;
  int has_color;
  int has_diffuse_color;
  int has_specular_color;
  int has_position;
  int has_normal;
  int has_radius;
  int has_direction;
  int has_radial_a0;
  int has_radial_a1;
  int has_radial_a2;
  int has_angular_a0;
  int has_theta;
  int has_reflectivity;
  int has_refractivity;
  int has_ior;
  int has_A;
  int has_B;
  int has_C;
  int has_D;
  int has_E;
  int has_F;
  int has_G;
  int has_H;
  int has_I;
  int has_J;
} has_values ;
  
// not using unions because this makes it easier to reference
typedef struct JSON_object {
  char *type;       // helpful to have both string and number reference for this
  int  typecode;    // 0 = camera, 1 = sphere, 2 = plane, 3 = cylinder, 4 = quadric, 5 = light
  double width;
  double height;
  double color[3];
  double diffuse_color[3];
  double specular_color[3];
  double position[3];
  double normal[3];
  double radius;
  double direction[3];
  double radial_a0;
  double radial_a1;
  double radial_a2;
  double angular_a0;
  double theta;
  double reflectivity;
  double refractivity;
  double ior;
  A_J coeffs;       // quadric coefficients
  has_values flags; // help with error checking, flag to make sure values are set
} JSON_object ;

// Will create an array of these separate from other JSON objects
typedef struct light_object {
  double color[3];
  double position[3];
  double direction[3];
  double radial_a0;
  double radial_a1;
  double radial_a2;
  double angular_a0;
  double theta;
  has_values flags;
} light_object ;

typedef struct light_object_struct {
  int num_lights;
  light_object light_objects[128];
} light_object_struct ;

// This may not be the best approach, and it's certainly not most efficient - to have an array that
// is always 128 "JSON_object"s large. But it's clean and all of the data related to the JSON
// scene file is in this one struct, filehandle and all, that's what I like about it.
typedef struct JSON_file_struct {
  FILE* fh_in;
  int width;
  int height;
  int num_objects;
  JSON_object js_objects[128];
} JSON_file_struct ;

// inline functions:
static inline double sqr (double v) {
  return v * v;
}

static inline void vNormalize (double* v) {
  double len = sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
}

static inline void vAdd(double* a, double* b, double* c) {
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
}

static inline void vSubtract(double* a, double* b, double* c) {
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
  //  printf("  vSubtract DBG: [%f,%f,%f] - [%f,%f,%f] = [%f, %f, %f]\n",a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2]);
}

static inline double vDot(double* a, double* b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline double vNorm(double* a) {
  return sqrt(vDot(a,a));
}

static inline void vScale(double* a, double s, double* c) {
  c[0] = s * a[0];
  c[1] = s * a[1];
  c[2] = s * a[2];
}

static inline void vCross(double* a, double* b, double* c) {
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

static inline double pDistance (double* a, double* b) {
  return sqrt(sqr(b[0]-a[0]) + sqr(b[1]-a[1]) + sqr(b[2]-a[2]));
}

static inline double degreesToRadians (double deg) {
  //return degrees = radians * 180.0 / M_PI;
  return deg * ( M_PI / 180 );
}

// END inline functions

// global variables
int OUTPUT_MAGIC_NUMBER = 6; // default to P6 PPM format
int MAX_RECURSION_LEVEL = 3; 
int VERBOSE             = 0; // controls logfile message level
int INFO                = 0; // controls if Info messages are printed, turn off prior to submission
int DBG                 = 0; // turns on the current set of DBG statements, whatever they are
int DBG2                = 0; 

double ambient_color[3] = {0.05,0.05,0.05}; // HACK: define these until ambient color from JSON supported

// global data structures
JSON_file_struct    INPUT_FILE_DATA;
RGBPixel           *RGB_PIXEL_MAP;
PPM_file_struct     OUTPUT_FILE_DATA;
RGBAPixel          *RGBA_PIXEL_MAP;
light_object_struct LIGHT_OBJECTS;

// functions
void  writePPM        (char *outfile,         PPM_file_struct *input);
void  message         (char message_code[],   char message[]        );
void  writePPMHeader  (FILE* fh              );
void  reportPPMStruct (PPM_file_struct *input);
void  reportPixelMap  (RGBPixel *pm          );
void  checkJSON       (JSON_object *scene);
void  printJSONObjectStruct (JSON_object jostruct);
void  storeDouble           (int obj_count, char* key, double value);
void  storeVector           (int obj_count, char* key, double* value);
void  renderScene           (JSON_object *scene, RGBPixel *image);

double getIntersections     (int index, double* Ro, double* Rd, double t_i);
double sphereIntersection   (double* Ro, double* Rd, double* C, double r);
double planeIntersection    (double* Ro, double* Rd, double* C, double* N);
double planeIntersectionOrig    (double* Ro, double* Rd, double* C, double* N);
double quadricIntersection  (double* Ro, double* Rd, double* C, A_J c, double* Nq);
double cylinderIntersection (double* Ro, double* Rd, double* C, double r);

void rayCast(double* Ro, double* Rd, double* color_in, double* color_out, int level);

double getColor        (double value1, double value2);
void   getObjectNormal (int index, double* Ro, double* Qn, double* N);
unsigned char clampColor (double color);

double fAng (int l_index, double* V);
double fRad (int l_index, double dl);
double Idiff (int o_index, int l_index, int c_index, double* N, double* L);
double Ispec (int o_index, int l_index, int c_index, double* V, double* R, double* N, double* L, double ns);

void  help();
int   computeDepth();
char* computeTuplType();
void  freeGlobalMemory ();
void  closeAndExit ();
void  reportScene();
double getCameraWidth();
double getCameraHeight();
void   populateLightArray ();

void getReflectionVector (double* L, double* N, double* R, int DBG_flag);
void getRefractionVector (double* Ur, double* n, double ior, double* Ut);

/* 
 ------------------------------------------------------------------
 Parser and functions from Dr. P - refactored and enhanced
 ------------------------------------------------------------------
*/
int line = 1;

// nextC() wraps the getc() function and provides error checking and line
// number maintenance
int nextC(FILE* json) {
  int c = fgetc(json);
  if (VERBOSE) printf("'%c'", c);
  if (c == '\n') {
    line += 1;
  }
  if (c == EOF) {
    fprintf(stderr, "Error: Unexpected end of file on line number %d.\n", line);
    exit(1);
  }
  return c;
}

// expectC() checks that the next character is d.  If it is not it emits
void expectC(FILE* json, int d) {
  int c = nextC(json);
  if (c == d) return;
  fprintf(stderr, "Error: Expected '%c' on line %d.\n", d, line);
  exit(1);    
}


// skipWSpace() skips white space in the file.
void skipWSpace(FILE* json) {
  int c = nextC(json);
  while (isspace(c)) {
    c = nextC(json);
  }
  ungetc(c, json);
}


// nextString() gets the next string from the file handle and emits an error
// if a string can not be obtained.
char* nextString(FILE* json) {
  char buffer[129];
  int c = nextC(json);
  if (c != '"') {
    fprintf(stderr, "Error: Expected string on line %d.\n", line);
    exit(1);
  }  
  c = nextC(json);
  int i = 0;
  while (c != '"') {
    if (i >= 128) {
      fprintf(stderr, "Error: Strings longer than 128 characters in length are not supported.\n");
      exit(1);      
    }
    if (c == '\\') {
      fprintf(stderr, "Error: Strings with escape codes are not supported.\n");
      exit(1);      
    }
    if (c < 32 || c > 126) {
      fprintf(stderr, "Error: Strings may contain only ascii characters.\n");
      exit(1);
    }
    buffer[i] = c;
    i += 1;
    c = nextC(json);
  }
  buffer[i] = 0;
  if (VERBOSE) printf("\nnS:  returning string--> (%s)\n",buffer);
  return strdup(buffer);
}

// Get the next number, check that fscanf returns one item
double nextNumber(FILE* json) {
  double value;
  int rval;
  rval = fscanf(json, "%lf", &value);
  if (rval != 1) {
    fprintf(stderr, "Error: expected valid number on line %d, did not find one\n",line);
    exit(1);
  }
  if (VERBOSE) printf("\nnN:  returning number--> (%lf)(%d)\n",value,rval);
  return value;
}

double* nextVector(FILE* json) {
  double* v = malloc(4*sizeof(double));
  expectC(json, '[');
  skipWSpace(json);
  v[0] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ',');
  skipWSpace(json);
  v[1] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ',');
  skipWSpace(json);
  v[2] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ']');
  if (VERBOSE) printf("nV: returning vector--> %d\n",v);
  return v;
}

// This is the big JSON parser function
void readScene(char* filename) {
  int c;
  int obj_count = 0;
  
  FILE* json = fopen(filename, "r");

  if (json == NULL) {
    fprintf(stderr, "Error: Could not open file \"%s\"\n", filename);
    exit(1);
  }
  
  skipWSpace(json);
  
  // Find the beginning of the list
  expectC(json, '[');

  skipWSpace(json);

  // Find all the objects in the JSON scene file
  int fail_safe = 0;
  while (1) {
    /* Error checking */
    // max supported objects * number of JSON lines per object (with margin of error)
    fail_safe++;
    if (fail_safe > 1280) message("Error","Do you have a ']' to terminate your JSON file?");
    if (obj_count > 128) message("Error","Maximum supported number of JSON objects is 128");

    /* Process file */
    c = fgetc(json);
    if (c == ']') {
      message("Error","No objects detected!\n");
      fclose(json);
      return;
    }
    if (c == '{') {
      skipWSpace(json);
      
      // Parse the object, getting the type first
      char* key = nextString(json);
      if (strcmp(key, "type") != 0) {
	fprintf(stderr, "Error: Expected \"type\" key on line number %d.\n", line);
	exit(1);
      }
      
      skipWSpace(json);
      expectC(json, ':');
      skipWSpace(json);
      
      // get the type of the object and store it at the index of the current object
      char* value = nextString(json);
      if (strcmp(value, "camera") == 0) {
	if (INFO) message("Info","Processing camera object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "camera";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 0;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "light") == 0) {
	if (INFO) message("Info","Processing light object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "light";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 5;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "sphere") == 0) {
	if (INFO) message("Info","Processing sphere object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "sphere";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 1;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "plane") == 0) {
	if (INFO) message("Info","Processing plane object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "plane";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 2;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "cylinder") == 0) {
	if (INFO) message("Info","Processing cylinder object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "cylinder";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 3;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "quadric") == 0) {
	if (INFO) message("Info","Processing quadric object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "quadric";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 4;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else {
	fprintf(stderr, "Error: Unknown type, \"%s\", on line number %d.\n", value, line);
	exit(1);
      }
      char* type = value; // store for error checking later
      skipWSpace(json);
      
      // This while processes the attributes of the object
      while (1) {
	// , }
	c = nextC(json);
	if (c == '}') {
	  // stop parsing this object and increment the object counter
	  obj_count++;
	  break;
	} else if (c == ',') {
	  // read another field
	  skipWSpace(json);
	  char* key = nextString(json);
	  skipWSpace(json);
	  expectC(json, ':');
	  skipWSpace(json);
	  // read values
	  if ((strcmp(key, "width") == 0) ||
	      (strcmp(key, "height") == 0) ||
	      (strcmp(key, "radius") == 0) ||
	      (strcmp(key, "radial-a0") == 0) ||
	      (strcmp(key, "radial-a1") == 0) ||
	      (strcmp(key, "radial-a2") == 0) ||
	      (strcmp(key, "angular-a0") == 0) ||
	      (strcmp(key, "theta") == 0) ||
	      (strcmp(key, "reflectivity") == 0) ||
	      (strcmp(key, "refractivity") == 0) ||
	      (strcmp(key, "ior") == 0) ||
	      (strcmp(key, "A") == 0) ||
	      (strcmp(key, "B") == 0) ||
	      (strcmp(key, "C") == 0) ||
	      (strcmp(key, "D") == 0) ||
	      (strcmp(key, "E") == 0) ||
	      (strcmp(key, "F") == 0) ||
	      (strcmp(key, "G") == 0) ||
	      (strcmp(key, "H") == 0) ||
	      (strcmp(key, "I") == 0) ||
	      (strcmp(key, "J") == 0)
	      ) {
	    double value = nextNumber(json);
	    // Error checking
	    // TODO add checks for project3 new params (like radial-a0 < 360, etc..)
	    //if (strcmp(type,"plane") == 0) message("Error","Plane has value that makes no sense(width,radius,etc)");
	    // store the value if pass checks
	    storeDouble(obj_count,key,value);
	  } else if ((strcmp(key, "color") == 0) ||
		     (strcmp(key, "diffuse_color") == 0) ||
		     (strcmp(key, "specular_color") == 0) ||
		     (strcmp(key, "position") == 0) ||
		     (strcmp(key, "normal") == 0) ||
		     (strcmp(key, "direction") == 0)) {
	    double* value = nextVector(json);
	    // Error checking
	    if (strcmp(type,"sphere") == 0 && strcmp(key,"normal") == 0)
	      message("Error","Sphere shouldn't have normal");
	    // store the value if pass checks
	    storeVector(obj_count,key,value);
	    free(value);
	  } else {
	    fprintf(stderr, "Error: Unknown property, \"%s\", on line %d.\n",key, line);
	    exit(1);
	  }
	  skipWSpace(json);
	} else {
	  fprintf(stderr, "Error: Unexpected value on line %d\n", line);
	  exit(1);
	}
      }
      skipWSpace(json);
      c = nextC(json);
      if (c == ',') {
	skipWSpace(json);
      } else if (c == ']') {
	fclose(json);
	if (INFO) printf("Info: Done reading scene file with %d objects\n",obj_count);
	return;
      } else {
	fprintf(stderr, "Error: Expecting ',' or ']' on line %d.\n", line);
	exit(1);
      }
    }
  }
}
/* 
 ------------------------------------------------------------------
 End pirated code - Parser and functions from Dr. P
 ------------------------------------------------------------------
*/


/*
 ------------------------------------------------------------------
                                 MAIN
 ------------------------------------------------------------------
*/
int main(int argc, char *argv[]) {
  // check for proper number of input args
  if (argc != 5) {
    help();
    return(EXIT_FAILURE);
  }

  // process input arguments and report what is being processed, store some variables, do some error checking
  int width = atoi(argv[1]);
  int height = atoi(argv[2]);
  char *infile = argv[3];
  char *outfile = argv[4];
  if (strcmp(infile,outfile)  == 0)
    {fprintf(stderr,"Error: input and output file names the same!\n"); return EXIT_FAILURE;}
  if (access(infile, F_OK) == -1)
    {fprintf(stderr,"Error: Input file \"%s\" does not exist!\n",infile); return EXIT_FAILURE;}
  
  if (INFO) message("Info","Processing the following arguments:");
  if (INFO) printf("          Input : %s\n",infile);
  if (INFO) printf("          Output: %s\n",outfile);
  if (INFO) printf("          Width : %d\n",width);
  if (INFO) printf("          Height: %d\n",height);

  INPUT_FILE_DATA.width = width;
  INPUT_FILE_DATA.height = height;

  // parse the JSON
  readScene(infile);
  populateLightArray();
  
  // error checking
  checkJSON(INPUT_FILE_DATA.js_objects);

  // report results
  if (INFO) reportScene();

  // initialize the image buffer
  RGB_PIXEL_MAP = malloc(sizeof(RGBPixel) * INPUT_FILE_DATA.width * INPUT_FILE_DATA.height );
  
  // render the scene
  renderScene(INPUT_FILE_DATA.js_objects,RGB_PIXEL_MAP);

  // write the image
  writePPM(outfile,&OUTPUT_FILE_DATA);

  // prepare to exit
  freeGlobalMemory();
  return EXIT_SUCCESS;
}
/* 
 ------------------------------------------------------------------
                               END MAIN
 ------------------------------------------------------------------
*/



/*
  ------------------------
  FUNCTION IMPLEMENTATIONS
  ------------------------
*/

/*
  --- message ---
  - 9/10/16
  - rmr5
  ---------------
  print a message to stdout consisting of a message code and a message to a given channel
  current valid channels to write to (stdout, stderr, etc) - will add fh later
  //void message (char channel[], char message_code[], char message[]) {
*/
void message (char message_code[], char message[]) {
  if(message_code == "Error") {
    fprintf(stderr,"%s: %s\n",message_code,message);
    closeAndExit();
    exit(-1);
  } else {
    printf("%s: %s\n",message_code,message);
  }
}

/*
  --- help ---
  - rmr5
  ---------------
  print usage to user when arguments invalid
*/
void help () {
  message("Error","Invalid arguments!");
  message("Usage","raycast width height input.json output.ppm");
}

/*
  --- freeGlobalMemory ---
  - 9/15/16
  - rmr5
  ---------------
  free up any globally malloc memory
*/
// TODO: make this more universal
// TODO: this is causing core dumps for only certain cases, don't understand it
void freeGlobalMemory () {
  if (INFO) message("Info","Freeing global memory...");
  free(RGB_PIXEL_MAP);
  //free(RGBA_PIXEL_MAP);
  if (INFO) message("Info","Done.");
}

/*
  --- closeAndExit ---
  - 9/15/16
  - rmr5
  ---------------
  prepare to gracefully exit the program by freeing memory and closing any open filehandles
  TODO: need to finish
*/
void closeAndExit () {
  freeGlobalMemory();
  //fclose(INPUT_FILE_DATA->fh_in);
  exit(-1);
}


//  small helper to assign proper depth to a P7 file
int computeDepth() {
  if ((strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA")) == 0) {
    return 4; 
  } else {
    return 3;
  }
}

// helper to assign preper tupltype in P7
char* computeTuplType() {
  if ((strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA")) == 0) {
    if (VERBOSE) printf("cD: returning tupltype RGB_ALPHA because input was %s\n",OUTPUT_FILE_DATA.tupltype);
    return "RGB_ALPHA"; 
  } else {
    if (VERBOSE) printf("cD: returning tupltype RGB because input was %s\n",OUTPUT_FILE_DATA.tupltype);
    return "RGB"; 
  }
}

// helper function to write the header to a file handle
void writePPMHeader (FILE* fh) {
  int magic_number = OUTPUT_MAGIC_NUMBER;

  // These values/header elements are the same regardless format
  if (INFO) printf("Info: Converting to format %d ...\n",magic_number);
  fprintf(fh,"P%d\n",magic_number);
  fprintf(fh,"# PPM file format %d\n",magic_number);
  fprintf(fh,"# written by ppmrw(rmr5)\n");
  // make some variable assignments from input -> output
  OUTPUT_FILE_DATA.magic_number = magic_number;
  OUTPUT_FILE_DATA.width        = INPUT_FILE_DATA.width;
  OUTPUT_FILE_DATA.height       = INPUT_FILE_DATA.height;
  OUTPUT_FILE_DATA.alpha        = 255;
  
  if (magic_number == 3 || magic_number == 6) {
    fprintf(fh,"%d %d\n",       OUTPUT_FILE_DATA.width,OUTPUT_FILE_DATA.height);
    fprintf(fh,"%d\n",          OUTPUT_FILE_DATA.alpha);
  } else if (magic_number == 7) {
    // HACK - hard-code these since the orig ppmrw code depended on readPPM to do this
    //OUTPUT_FILE_DATA.depth      = computeDepth();
    OUTPUT_FILE_DATA.depth      = 3;
    //OUTPUT_FILE_DATA.tupltype   = computeTuplType();
    OUTPUT_FILE_DATA.tupltype   = "RGB";
    fprintf(fh,"WIDTH %d\n",    OUTPUT_FILE_DATA.width);
    fprintf(fh,"HEIGHT %d\n",   OUTPUT_FILE_DATA.height);
    fprintf(fh,"DEPTH %d\n",    OUTPUT_FILE_DATA.depth);
    fprintf(fh,"MAXVAL %d\n",   OUTPUT_FILE_DATA.alpha);
    fprintf(fh,"TUPLTYPE %s\n", OUTPUT_FILE_DATA.tupltype);
    fprintf(fh,"ENDHDR\n");
  } else {
    message("Error","Trying to output unsupported format!\n");
  }
  if (INFO) message("Info","Done writing header");
}

/*
  --- writePPM ---
  - 9/13/16
  - rmr5
  ---------------
  Major function to write the actual output ppm file
  takes a output filename and an input PPM struct
  uses global data

  This function has case statements to support all supported formats 
*/
void writePPM (char *outfile, PPM_file_struct *input) {
  if (INFO) printf("Info: Writing file %s...\n",outfile);
  FILE* fh_out = fopen(outfile,"wb");

  // -------------------------- write header ---------------------------------
  writePPMHeader(fh_out);
  // ---------------------- done write header --------------------------------

  // -------------------------- write image ----------------------------------
  int pixel_index = 0;
  int modulo;

  switch(OUTPUT_FILE_DATA.magic_number) {
    // P3 format
    // Iterate over each pixel in the pixel map and write them byte by byte
  case(3):
    if (INFO) message("Info","Outputting format 3");
    while(pixel_index < (OUTPUT_FILE_DATA.width) * (OUTPUT_FILE_DATA.height)) {      
      fprintf(fh_out,"%3d %3d %3d",RGB_PIXEL_MAP[pixel_index].r,RGB_PIXEL_MAP[pixel_index].g,RGB_PIXEL_MAP[pixel_index].b);
      modulo = (pixel_index + 1) % (OUTPUT_FILE_DATA.width);
      if ( modulo == 0 ) {
	fprintf(fh_out,"\n");
      } else {
	fprintf(fh_out," ");
      }
      pixel_index++;
    }
    break;
    // P6 format
    // write the entire pixel_map in one command
  case(6):
    if (INFO) message("Info","Outputting format 6");
    fwrite(RGB_PIXEL_MAP, sizeof(RGBPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    break;
    // P7 format
  case(7):
    // write the entire pixel_map in one command, RGB writes from RGB pixel_map and RGBA writes from RGBA pixel_map
    if (INFO) message("Info","Outputting format 7");
    if (strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA") == 0) {
      if (INFO) message("Info","   output file will have alpha data");
      fwrite(RGBA_PIXEL_MAP, sizeof(RGBAPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    } else {
      if (INFO) message("Info","   output file is RGB only");
      fwrite(RGB_PIXEL_MAP, sizeof(RGBPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    }
    break;
  default:
    message("Error","Unrecognized output format");
  }

  // ---------------------- done write image ---------------------------------
  fclose(fh_out);
  if (INFO) reportPPMStruct(&OUTPUT_FILE_DATA);
  if (INFO) message("Info","Done writing PPM");
}

// helper function to visualize what's in a given PPM struct
void reportPPMStruct (PPM_file_struct *input) {
  message("Info","Contents of PPM struct:");
  printf("     magic_number: %d\n",input->magic_number);
  printf("     width:        %d\n",input->width);
  printf("     height:       %d\n",input->height);
  if (input->magic_number == 7) {
    printf("     max_value:    %d\n",input->alpha);
    printf("     depth:        %d\n",input->depth);
    printf("     tupltype:     %s\n",input->tupltype);
  } else {
    printf("     alpha:        %d\n",input->alpha);
  }
}

// small utility function to print the contents of a pixelMap
void reportPixelMap (RGBPixel *pm) {
  int index = 0;
  int fail_safe = 0;
  while(index < sizeof(pm) && fail_safe < 1000) {
    printf("rPM: [%d] = [%d,%d,%d]\n",index,pm[index].r,pm[index].g,pm[index].b);
    index++;
    fail_safe++;
  }
}

// helper to print out a JSON_object
void printJSONObjectStruct (JSON_object jostruct) {
  printf("type: %s\n",jostruct.type);
  printf("  d color: [%f, %f, %f]\n",jostruct.diffuse_color[0], jostruct.diffuse_color[1], jostruct.diffuse_color[2]);
  printf("  s color: [%f, %f, %f]\n",jostruct.specular_color[0], jostruct.specular_color[1], jostruct.specular_color[2]);
  printf("  color: [%f, %f, %f]\n",jostruct.color[0], jostruct.color[1], jostruct.color[2]);
  if (strcmp(jostruct.type,"camera") == 0) {
    printf(" width: %f\n",jostruct.width);
    printf("height: %f\n",jostruct.height);
  } else if (strcmp(jostruct.type,"light") == 0) {
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
  } else if (strcmp(jostruct.type,"sphere") == 0) {
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("   radius: %f\n",jostruct.radius);
  } else if (strcmp(jostruct.type,"plane") == 0) {
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("   normal: [%f, %f, %f]\n",jostruct.normal[0], jostruct.normal[1], jostruct.normal[2]);
  } else if (strcmp(jostruct.type,"quadric") == 0) {
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("        A: %f\n",jostruct.coeffs.A);
    printf("        B: %f\n",jostruct.coeffs.B);
    printf("        C: %f\n",jostruct.coeffs.C);
    printf("        D: %f\n",jostruct.coeffs.D);
    printf("        E: %f\n",jostruct.coeffs.E);
    printf("        F: %f\n",jostruct.coeffs.F);
    printf("        G: %f\n",jostruct.coeffs.G);
    printf("        H: %f\n",jostruct.coeffs.H);
    printf("        I: %f\n",jostruct.coeffs.I);
    printf("        J: %f\n",jostruct.coeffs.J);
  } else {
    printf("Error: unrecognized type\n");
  }
  printf("\n");
}

// helper to report the results of a scene parse
void reportScene () {
  int len_array = INPUT_FILE_DATA.num_objects;
  if (INFO) printf("\n\n---------------------\n");
  if (INFO) message("Info","PARSE RESULTS:");
  if (INFO) printf("---------------------\n");
  if (INFO) printf("Processed scene with %d objects:\n\n",len_array);
  for (int i = 0; i < len_array; i++) {
    printJSONObjectStruct(INPUT_FILE_DATA.js_objects[i]);
  }
}

// helper to store a double onto our JSON object file
void storeDouble(int obj_count, char* key, double value) {
  if (VERBOSE) printf("   sD: storing %s,%lf at %d\n",key,value,obj_count);

  // store the actual value, not sure how to say ".key" and get it to evaluate key so need these ifs
  if (strcmp(key,"width") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].width = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_width = 1;
  } else if (strcmp(key,"height") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].height = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_height = 1;
  } else if (strcmp(key,"radius") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].radius = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_radius = 1;
  } else if (strcmp(key,"radial-a0") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].radial_a0 = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_radial_a0 = 1;
  } else if (strcmp(key,"radial-a1") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].radial_a1 = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_radial_a1 = 1;
  } else if (strcmp(key,"radial-a2") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].radial_a2 = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_radial_a2 = 1;
  } else if (strcmp(key,"angular-a0") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].angular_a0 = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_angular_a0 = 1;
  } else if (strcmp(key,"theta") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].theta = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_theta = 1;
  } else if (strcmp(key,"reflectivity") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].reflectivity = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_reflectivity = 1;
  } else if (strcmp(key,"refractivity") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].refractivity = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_refractivity = 1;
  } else if (strcmp(key,"ior") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].ior = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_ior = 1;
  } else if (strcmp(key, "A") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.A = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_A = 1;
  } else if (strcmp(key, "B") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.B = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_B = 1;
  } else if (strcmp(key, "C") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.C = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_C = 1;
  } else if (strcmp(key, "D") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.D = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_D = 1;
  } else if (strcmp(key, "E") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.E = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_E = 1;
  } else if (strcmp(key, "F") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.F = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_F = 1;
  } else if (strcmp(key, "G") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.G = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_G = 1;
  } else if (strcmp(key, "H") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.H = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_H = 1;
  } else if (strcmp(key, "I") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.I = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_I = 1;
  } else if (strcmp(key, "J") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.J = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_J = 1;
  } else {
    // This should never happen
    message("Error","Interally trying to store unknown key type");
  }
}

// helper to store a vector onto our JSON object file
void storeVector(int obj_count, char* key, double* value) {
  if (VERBOSE) printf("   sV: storing %s at %d\n",key,obj_count);
  if (strcmp(key,"color") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].color[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].color[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].color[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_color = 1;
  } else if (strcmp(key,"diffuse_color") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].diffuse_color[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].diffuse_color[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].diffuse_color[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_diffuse_color = 1;
  } else if (strcmp(key,"specular_color") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].specular_color[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].specular_color[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].specular_color[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_specular_color = 1;
  } else if (strcmp(key,"position") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].position[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].position[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].position[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_position = 1;
  } else if (strcmp(key,"normal") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].normal[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].normal[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].normal[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_normal = 1;
  } else if (strcmp(key,"direction") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].direction[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].direction[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].direction[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_direction = 1;
  } else {
    // This should never happen
    message("Error","Interally trying to store unknown vector key type");
  }

}

// Render the scene
void renderScene(JSON_object *scene, RGBPixel *image) {

  ////////////
  // variables
  ////////////
  // number of pixels that represent height/width
  int M = INPUT_FILE_DATA.height;
  int N = INPUT_FILE_DATA.width;
  int pixmap_length = M * N;
  // this represents the center of the view plane
  double cx = 0;
  double cy = 0;
  double cz = 1;
  // make a view plane according to the (first) camera object in the JSON
  double w = getCameraWidth();
  double h = getCameraHeight();
  // height/width of each pixel
  double pixwidth = w / N;
  double pixheight = h / M;
  int i = 0; // pixelmap counter, since my pixelmap is a flat array
  
  if (INFO) printf("Info: rendering %d x %d image to memory ...\n",N,M);

  //////////////////////////////////////
  // copy of psuedo code from text/class
  //////////////////////////////////////
  // structure of every ray tracer you will ever encounter
  // go over all x/y values for a scene and check for intersections
  for (int y = M; y >= 0; y -= 1) { // y-axis was flipped, so run this backwards
    for (int x = 0; x < N; x += 1) {
      // origin
      double Ro[3] = {0,0,0}; // vector that represents a point that represents the origin
      // direction
      // Rd = normalize(P - Ro), origin in 0 so skip that, but need to normalize
      double Rd[3] = {
	cx - (w/2) + pixwidth * ( x + 0.5),
	cy - (h/2) + pixheight * ( y + 0.5),
	cz
      };

      // next, need to make Rd so that it's actually normalized
      vNormalize(Rd);

      // cast the ray from camera to pixel, rayCast will do the recursive ray tracing
      //      double* color_out = malloc(3*sizeof(double));
      double default_color[3] = {DEFAULT_COLOR,DEFAULT_COLOR,DEFAULT_COLOR};
      double color_out[3] = {0,0,0};
      if (DBG) printf("calling rC @ {%d,%d}\n",x,y);
      rayCast(Ro,Rd,default_color,color_out,0);
      RGB_PIXEL_MAP[i].r = clampColor(color_out[0]);
      RGB_PIXEL_MAP[i].g = clampColor(color_out[1]);
      RGB_PIXEL_MAP[i].b = clampColor(color_out[2]);
      //      free(color_out);
      i++; // increment the pixelmap counter
    }
  }
  if (VERBOSE) message("Info","Done rendering the image");
}

// Raycaster function - returns the color found by casting a ray from Ro in Rd direction
//double * rayCast(double* Ro, double* Rd, double* color) { // warning about returning local variable
void rayCast(double* Ro, double* Rd, double* color_in, double* color_out, int level) {
  
  // variables
  double best_t = INFINITY;
  int    best_t_index = 129;
  double object_color[3];
  double Qn[3];              // this will be the normal for the quadric at intersection point 
  
  // Base case
  if (level > MAX_RECURSION_LEVEL) {
    color_out[0] = color_in[0];
    color_out[1] = color_in[1];
    color_out[2] = color_in[2];
    return;
  }

  // loop over all objects in scene and find intersections, could also use objects != NULL
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o += 1) {
    // t stores if we have an intersection or not
    double t = 0;
    

    switch(INPUT_FILE_DATA.js_objects[o].typecode) {
    case 0: // skip the camera
      break;
    case 1:
      t = sphereIntersection(Ro,Rd,
			     INPUT_FILE_DATA.js_objects[o].position,
			     INPUT_FILE_DATA.js_objects[o].radius);
      break;
    case 2:	
      t = planeIntersection(Ro,Rd,
			    INPUT_FILE_DATA.js_objects[o].position,
			    INPUT_FILE_DATA.js_objects[o].normal);
      break;
    case 3:
      t = cylinderIntersection(Ro,Rd,
			       INPUT_FILE_DATA.js_objects[o].position,
			       INPUT_FILE_DATA.js_objects[o].radius);
      break;
    case 4:
      // Pass Qn into function so it can "return" Qn as the quadric normal
      t = quadricIntersection(Ro,Rd,
			      INPUT_FILE_DATA.js_objects[o].position,
			      INPUT_FILE_DATA.js_objects[o].coeffs,
			      Qn);
      break;
    case 5:
      break;
    default:
      message("Error","Unhandled typecode, camera/light/plane/sphere are supported");
    }

    if (t > 0 && t < best_t) {
      best_t = t;
      best_t_index = o;
      // Need the object color shadow case
      object_color[0] = INPUT_FILE_DATA.js_objects[o].color[0];
      object_color[1] = INPUT_FILE_DATA.js_objects[o].color[1];
      object_color[2] = INPUT_FILE_DATA.js_objects[o].color[2];
    }
  }

  // Now look at the t value and see if there was an intersection
  // remember that you could have multiple objects in from of each other, check for the smaller of the
  // t values, that hit first, shade that one
  if (best_t > 0 && best_t != INFINITY) {
    
    // Add the shading function, project3 main task
    if (DBG) printf("rC: (%s)[%d]\n",INPUT_FILE_DATA.js_objects[best_t_index].type,best_t_index);

    // now, the summation of all the lights in the scene
    for (int j = 0; j < LIGHT_OBJECTS.num_lights; j++) {
      // variables for each light
      double t_shadow = 0;
      int    best_t_shadow_index = 129; // then maintain this throughout code
      double best_t_shadow = INFINITY;
      
      if (DBG) printf("  light index (%d)\n",j);
      // shadow test for each light, first create new Ro for the shadow test
      double Ro_tmp[3];
      double Ro_new[3];
      if (DBG) printf("Rd: [%f, %f, %f]\n",Rd[0],Rd[1],Rd[2]); // Rd is normalized at this point
      if (DBG) printf("best_t: %f\n",best_t);
      vScale(Rd,best_t,Ro_tmp);
      if (DBG) printf("Ro: [%f, %f, %f]\n",Ro[0],Ro[1],Ro[2]);
      if (DBG) printf("Ro_tmp: [%f, %f, %f]\n",Ro_tmp[0],Ro_tmp[1],Ro_tmp[2]);
      vAdd(Ro,Ro_tmp,Ro_new);
      if (DBG) printf("Ro_new: [%f, %f, %f]\n",Ro_new[0],Ro_new[1],Ro_new[2]);
      
      // next, create new Rd for the shadow test and calculate distance to the light object
      double Rd_new[3];
      double light_position[3];
      light_position[0] = LIGHT_OBJECTS.light_objects[j].position[0];
      light_position[1] = LIGHT_OBJECTS.light_objects[j].position[1];
      light_position[2] = LIGHT_OBJECTS.light_objects[j].position[2];
      vSubtract(light_position,Ro_new,Rd_new);
      if (DBG) printf("Rd_new: [%f, %f, %f]\n",Rd_new[0],Rd_new[1],Rd_new[2]);
      double dl = pDistance(light_position,Ro_new); // distance from object to light
      double dummy_normal[3];                       // dummy value to pass into quadric intersection function
      
      // now iterate over each object in the scene and check for intersection, indicating a shadow
      for (int k = 0; k < INPUT_FILE_DATA.num_objects ; k++) { 
	if (DBG) printf("    object index (%d) is a (%d)\n",k,INPUT_FILE_DATA.js_objects[k].typecode);
	// skip lights, won't have intersections (leave camera or 1 object scene will not render)
	if (INPUT_FILE_DATA.js_objects[k].typecode == 5) continue;
	
	// how to deal with the object we are checking from itself, could shadow parts of itself from the light
	if (DBG) printf("    checking against (%d)\n",best_t_index);
	if (k == best_t_index) continue;

	// test for intersections to find shadows
	// TODO: use functionized code instead of this copy/paste hack
	if (DBG) printf("    testing intersections...[%f]\n",t_shadow);
	switch(INPUT_FILE_DATA.js_objects[k].typecode) {
	case 0: // skip the camera
	  break;
	case 1:
	  t_shadow = sphereIntersection(Ro_new,Rd_new,
					INPUT_FILE_DATA.js_objects[k].position,
					INPUT_FILE_DATA.js_objects[k].radius);
	  //	  if (DBG) printf("     case1[%f]\n",t_shadow);
	  break;
	case 2:	
	  t_shadow = planeIntersection(Ro_new,Rd_new,
				       INPUT_FILE_DATA.js_objects[k].position,
				       INPUT_FILE_DATA.js_objects[k].normal);
	  if (DBG) printf("     case2[%f]\n",t_shadow);
	  break;
	case 3:
	  t_shadow = cylinderIntersection(Ro_new,Rd_new,
					  INPUT_FILE_DATA.js_objects[k].position,
					  INPUT_FILE_DATA.js_objects[k].radius);
	  //	  if (DBG) printf("     case3[%f]\n",t_shadow);
	  break;
	case 4:
	  t_shadow = quadricIntersection(Ro_new,Rd_new,
					 INPUT_FILE_DATA.js_objects[k].position,
					 INPUT_FILE_DATA.js_objects[k].coeffs,
					 dummy_normal);
	  //	  if (DBG) printf("     case4[%f]\n",t_shadow);
	  break;
	case 5:
	  break;
	default:
	  message("Error","Unhandled typecode, camera/light/plane/sphere are supported");
	}
	
	// need a new best_t here to determine if we are in shadow or not
	if (t_shadow > 0 && t_shadow < best_t_shadow) {
	  best_t_shadow = t_shadow;
	  best_t_shadow_index = k;
	}

	// remember you need to clamp the distance to not go beyond the light's distance to any object beyond it
	if (DBG) printf("      b_t_s: %f, b_t_s_i: %d, dl: %f\n",best_t_shadow,best_t_shadow_index,dl);
	if (best_t_shadow > dl && best_t_shadow != INFINITY) goto SP;

	if (best_t_shadow_index != 129) {
	  // in a shadow, shade the pixel based on ambient light
	  /*
	  color_out[0] = getColor(object_color[0],ambient_color[0]);
	  color_out[1] = getColor(object_color[1],ambient_color[1]);
	  color_out[2] = getColor(object_color[2],ambient_color[2]);
	  */
	  color_out[0] = color_in[0];
	  color_out[1] = color_in[1];
	  color_out[2] = color_in[2];
	  if (DBG) printf("      in shadow, ambient color: [%f,%f,%f]\n",color_out[0],color_out[1],color_out[2]);
	} else SP: { 
	  // not in a shadow, shade the pixel based on light source
	  // use N,L,R,V & fRad/fAng functions
	  double N[3];
	  double L[3];
	  double R[3];
	  double V[3];

	  //N = normal of the object we are testing for shadows
	  getObjectNormal(best_t_index,Ro_new,Qn,N);
	  //L = the new vector to the light source from shadow test object
	  L[0] = Rd_new[0];
	  L[1] = Rd_new[1];
	  L[2] = Rd_new[2];
	  if (DBG) printf(" orig L: [%f, %f, %f]\n",L[0],L[1],L[2]);
	  //V = original ray from camera to first object, Rd is normalized here
	  V[0] = -Rd[0];
	  V[1] = -Rd[1];
	  V[2] = -Rd[2];
	  //R = reflection of L about N: R = 2(N dot L)N - L
	  getReflectionVector(L,N,R,DBG);

	  // Add reflectivity for project 4
	  double Ir[3];
	  double Kr = INPUT_FILE_DATA.js_objects[best_t_index].reflectivity;
	  
	  if (INPUT_FILE_DATA.js_objects[best_t_index].flags.has_reflectivity &&
		INPUT_FILE_DATA.js_objects[best_t_index].reflectivity > 0) {
	    level++;
	    //printf("DBG: calling rayCast at level %d\n",reflect_level);
	    // adjust this by slight amount so you don't have to ignore current object
	    double Ro_new_prime[3];
	    double epsilon = 0.03;
	    Ro_new_prime[0] = Ro_new[0] + Rd_new[0]*epsilon;
	    Ro_new_prime[1] = Ro_new[1] + Rd_new[1]*epsilon;
	    Ro_new_prime[2] = Ro_new[2] + Rd_new[2]*epsilon;
	    //	    rayCast(Ro_new_prime,R,color_in,reflected_color,reflect_level,0);
	    //rayCast(Ro_new_prime,R,INPUT_FILE_DATA.js_objects[best_t_index].diffuse_color,Ir,level);
	    rayCast(Ro_new_prime,R,color_in,Ir,level);
	  }

	  // Add refractivity for project 4
	  /*
	  double U[3];
	  //void getRefractionVector (double* Ur, double* n, double Pt, double* Ut) {
	  getRefractionVector(L,N,INPUT_FILE_DATA.js_objects[best_t_index].ior,U);
	  if (INPUT_FILE_DATA.js_objects[best_t_index].flags.has_refractivity &&
		INPUT_FILE_DATA.js_objects[best_t_index].refractivity > 0) {
	    double refracted_color[3];
	    //printf("DBG: calling rayCast at level %d\n",level);
	    rayCast(Ro_new,U,color_in,refracted_color,0,level);
	    color_out[0] += refracted_color[0] * reflectivity;
	    color_out[1] = refracted_color[1];
	    color_out[2] = refracted_color[2];
	  }
	  */
	  
	  // compute radial attenuation
	  vNormalize(N);
	  double r_atten = fRad(j, dl);
	  
	  // compute the angular attenuation
	  if (DBG) printf(" passing L to fAng: [%f, %f, %f]\n",L[0],L[1],L[2]);
	  vNormalize(L);
	  double a_atten = fAng(j, L);

	  // compute the diffuse contribution
	  double diffuse[3];
	  diffuse[0] = Idiff(best_t_index, j, 0, N, L);
	  diffuse[1] = Idiff(best_t_index, j, 1, N, L);
	  diffuse[2] = Idiff(best_t_index, j, 2, N, L);
	  
	  // compute the specular contribution
	  double specular[3];
	  int ns = 50;
	  //vNormalize(V);, this doesn't matter either way, V already normalize, can normalize again
	  vNormalize(R);
	  specular[0] = Ispec(best_t_index, j, 0, V, R, N, L, ns);
	  specular[1] = Ispec(best_t_index, j, 1, V, R, N, L, ns);
	  specular[2] = Ispec(best_t_index, j, 2, V, R, N, L, ns);

	  // Now, use the big equation to calculate the color for each pixel
	  double tmp0 = color_out[0]; // for the DBG statement TODO remove this DBG stuff
	  double tmp1 = color_out[1]; // for the DBG statement
	  double tmp2 = color_out[2]; // for the DBG statement
	  color_out[0] = r_atten * a_atten * (diffuse[0] + specular[0]) + Ir[0]*Kr;
	  color_out[1] = r_atten * a_atten * (diffuse[1] + specular[1]) + Ir[1]*Kr;
	  color_out[2] = r_atten * a_atten * (diffuse[2] + specular[2]) + Ir[2]*Kr;
	  
	  /*
	  if (DBG) printf("DBG co[0](%f): co(%f), r_a(%f), a_a(%f), d(%f), s(%f)\n"
		 ,color_out[0],tmp0,r_atten,a_atten,diffuse[0],specular[0]);
	  if (DBG) printf("DBG co[1](%f): co(%f), r_a(%f), a_a(%f), d(%f), s(%f)\n"
		 ,color_out[1],tmp1,r_atten,a_atten,diffuse[1],specular[1]);
	  if (DBG) printf("      co[2](%f): co(%f), r_a(%f), a_a(%f), d(%f), s(%f)\n"
		 ,color_out[2],tmp2,r_atten,a_atten,diffuse[2],specular[2]);
	  */
	}
      }
    }
  } else {
    // ray found no intersections, so simply pass the input color as output color
    color_out[0] = color_in[0];
    color_out[1] = color_in[1];
    color_out[2] = color_in[2];
  }
}

// helper function to blend 2 colors together, used only in ambient case
double getColor (double value1, double value2) {
  double rval;
  if (value1 == DEFAULT_COLOR && value2 == DEFAULT_COLOR) {
    return DEFAULT_COLOR;
  } else if (value1 == DEFAULT_COLOR) {
    return value2;
  } else if (value2 == DEFAULT_COLOR) {
    //return ((value1 + value2) / 2);
    return value1;
    /*
      if (diffuse) {
      simply multiply the like color components assuming they are between 0 and 1 at this point, as
      opposed to averaging them
      } else if (specular) {
      
      }
     */
  } else {
    // TODO fix this simple color blend
    rval = ((value1 + value2*9) / 10); // okay
    //rval = value1 * value2;         // way too dark
    //rval = ((value1 * 0.95) + (value2 * 0.1))/2; // still too dark, even trying to weight the terms 
    //rval = (value1 * 0.25) + (value2 * 0.95);   // best so far
    //if (DBG) printf("DBG: averaging %f and %f to %f\n",value1,value2,rval);
    return rval;
  }
}

// convert double color into int color value
unsigned char clampColor (double color) {
  if (color > 1.0) {
    color = 1;
  } else if (color < 0) {
    color = 0;
  }
  return round(color * 255);
}

/////////////////////////
// Intersection checkers
/////////////////////////
// Step 1. Find the equation for the object we are interested in
// Step 2. Parameterize the equation with a center point if needed
// Step 3. Substitute the equation for ray into our object equation
// Step 4. Solve for t.
//      4a. Rewrite the equation (flatten, get rid of parens). (maple/mathmatica will solve this, or algebra)
//      4b. rewrite the equation in terms of t, want to solve for t
// Step 5. Use the quadratic equation (if relevant) to solve for t
/////////////////////////

// Sphere intersection code (from http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter1.htm)
double sphereIntersection(double* Ro, double* Rd, double* C, double r) {
  // Xhit = pr + (tclose - a)*ur from the notes, same as
  // S = the set of points[xs, ys, zs], where (xs - xc)2 + (ys - yc)2 + (zs - zc)2 = r^2
  // (Ro[0] + Rd[0] * t - C[0])2 + (Ro[1] + Rd[1] * t - C[1])2 + (Ro[2] + Rd[2] * t - C[2])2 = r^2
  // or A*t^2 + B*t + C = 0
  double a = sqr(Rd[0]) + sqr(Rd[1]) + sqr(Rd[2]); // with normalized ray, should always be 1
  double b = 2 * (Rd[0] * (Ro[0] - C[0]) + Rd[1] * (Ro[1] - C[1]) + Rd[2] * (Ro[2] - C[2]));
  double c = sqr(Ro[0] - C[0]) + sqr(Ro[1] - C[1]) + sqr(Ro[2] - C[2]) - sqr(r);

  double disc = sqr(b) - 4 * a * c;
  if (disc < 0) return -1;

  // take the square root
  disc = sqrt(disc);
  
  //t0, t1 = (- B + (B^2 - 4*C)^1/2) / 2 where t0 is for (-) and t1 is for (+)
  // compute t0, if positive, this is the smaller of 2 intersections, we are done, return it
  double t0 = (-b - disc) / (2 * a);
  if (t0 > 0) return t0;

  double t1 = (-b + disc) / (2 * a);
  if (t1 > 0) return t1;

  // no intersection if we fall through to here
  return -1;
}

// plane intersection code
// arguments are: the ray (origin/direction), center of the plane, normal of the plane
double planeIntersectionNew(double* Ro, double* Rd, double* C, double* N) {
  
  // use <ax + by + cz = d> (where a,b,c are from the normal vector, x,y,z are the center of the plane)
  // solve for d first by using the normal and the point, then plug in the ray to look for solutions for t
  double d = N[0]*C[0] + N[1]*C[1] + N[2]*C[2];
  
  // a(Ro[0] + t*Rd[0] - C[0]) + b(Ro[1] + t*Rd[1] - C[1]) + c(Ro[2] + t*Rd[2] - C[2]) - d = 0
  // N[0](Ro[0] + t*Rd[0] - C[0]) + N[1](Ro[1] + t*Rd[1] - C[1]) + N[2](Ro[2] + t*Rd[2] - C[2]) - d = 0
  // t * (N[0]*Rd[0] + N[1]*Rd[1] + N[2]*Rd[2]) + N[0]*Rd[0] - N[0]*C[0] + N[1]*Rd[1] - N[1]*C[1] + N[2]*Rd[2] - N[2]*C[2] - d = 0
  double t;
  double t_terms = N[0]*Rd[0] + N[1]*Rd[1] + N[2]*Rd[2];
  double non_t_terms = N[0]*Ro[0] - N[0]*C[0] + N[1]*Ro[1] - N[1]*C[1] + N[2]*Ro[2] - N[2]*C[2] - d;
  t = -non_t_terms / 2*t_terms;

  VERBOSE = 0; // DBG TODO remove
  if (VERBOSE) printf("Ro[0]: %f, Ro[1]: %f, Ro[2]: %f\n",Ro[0],Ro[1],Ro[2]);
  if (VERBOSE) printf("Rd[0]: %f, Rd[1]: %f, Rd[2]: %f\n",Rd[0],Rd[1],Rd[2]);
  if (VERBOSE) printf("C0: %f, C1: %f, C2: %f\n",C[0],C[1],C[2]);
  if (VERBOSE) printf("N0: %f, N1: %f, N2: %f\n",N[0],N[1],N[2]);
  if (VERBOSE) printf("t solution: %f\n",t);
  
  VERBOSE = 0; // DBG TODO remove
  // found a solution
  if (t > 0) {
    return t;
  } else {
    return -1;
  }
}

double planeIntersection(double* Ro, double* Rd, double* C, double* N) {

  // if Vd = (Pn dot Rd) = 0, no intersection, so compute it first and return if no intersection
  double Vd = vDot(N,Rd);
  if (Vd == 0) return -1;

  // Now subtract ray origin from the point on the plane and dot it with normal
  double the_diff[3];
  vSubtract(C,Ro,the_diff);
  double V0 = vDot(N,the_diff);

  VERBOSE = 0; // DBG TODO remove
  if (VERBOSE) printf("Ro[0]: %f, Ro[1]: %f, Ro[2]: %f\n",Ro[0],Ro[1],Ro[2]);
  if (VERBOSE) printf("C0: %f, C1: %f, C2: %f\n",C[0],C[1],C[2]);
  if (VERBOSE) printf("td0: %f, td1: %f, td2: %f\n",the_diff[0],the_diff[1],the_diff[2]);

  double t = V0 / Vd;
  if (VERBOSE) printf("V0: %f, Vd: %f, t: %f\n",V0,Vd,t);

  if (t < 0) return -1; // plane intersection is behind origin, ignore it

  // if we got this far, plane intersects the ray in front of origin
  if (VERBOSE) printf("returning %f\n",t);
  VERBOSE = 0; // DBG TODO remove
  return t;
}

// Cylinder intersection code (from example in class) 
double cylinderIntersection(double* Ro, double* Rd, double* C, double r) {
  // x^2 + z^2 = r^2   using z instead of y will make it go up/down, instead of looking head on
  double a = (sqr(Rd[0]) + sqr(Rd[2])); // remember that x/y/z are in array form
  double b = (2 * (Ro[0] * Rd[0] - Rd[0] * C[0] + Ro[2] * Rd[2] - Rd[2] * C[2]));
  double c = sqr(Ro[0]) - 2*Ro[0] * C[0] + sqr(C[0]) + sqr(Ro[2]) - 2*Ro[2] * C[2] + sqr(C[2]) - sqr(r);

  // discriminant, remember the negative version is not real, imaginary, not 
  double disc = sqr(b) - 4 * a * c;
  if (disc < 0 ) return -1; // this is the signal that there was not an intersection

  // since we are using it more than once
  disc = sqrt(disc);
  
  double t0 = (-b - disc) / (2*a);
  if (t0 > 0) return t0; // smaller/lesser of the 2 values, needs to come first

  double t1 = (-b + disc) / (2*a);
  if (t1 > 0) return t1;

  // this is a default case if we have no intersection, could also just return t1, but this accounts
  // for "numeric stability" as numbers become very close to 0
  return -1;
}

// Quadric intersection code
double quadricIntersection(double* Ro, double* Rd, double* C, A_J c, double* Nq) {
  // Based on the siggraph documentation on
  // http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter4.htm
  // Had to add in the offsets to position the object, the siggraph page didn't have that
  //
  // Hand-solved the equation that has the offsets
  // into the Aq*t^2 + Bq*t + Cq = 0 format for quadratic solving
  double Aq = c.A*sqr(Rd[0]) + c.B*sqr(Rd[1]) + c.C*sqr(Rd[2]) + c.D*Rd[0]*Rd[1] + c.E*Rd[0]*Rd[2] + c.F*Rd[1]*Rd[2];
  double Bq = - 2*c.A*Rd[0]*C[0] - 2*c.B*Rd[1]*C[1] - 2*c.C*Rd[2]*C[2] - c.D*Rd[0]*C[1] - c.D*Rd[1]*C[0] -
    c.E*Rd[0]*C[2] - c.E*Rd[2]*C[0] - c.F*Rd[1]*C[2] - c.F*Rd[2]*C[1] + c.G*Rd[0] + c.H*Rd[1] + c.I*Rd[2];
  double Cq = c.A*sqr(C[0]) + c.B*sqr(C[1]) + c.C*sqr(C[2]) + c.D*C[0]*C[1] +
    c.E*C[0]*C[2] + c.F*C[1]*C[2] - c.G*C[0] - c.H*C[1] - c.I*C[2] + c.J;
  double rval = -1; // will make it easier to calculate the normal, need t for that

  // Some debug statements
  if (VERBOSE) printf("DBG : xyz=(%f,%f,%f) AJ=(%f,%f,%f,%f)\n",C[0],C[1],C[2],c.A,c.B,c.C,c.D);
  if (VERBOSE) printf("DBG : Rd's > x=%f, y=%f, z=%f\n",Rd[0],Rd[1],Rd[2]);
  if (VERBOSE) printf("DBG : Aq=%f, Bq=%f, Cq=%f)\n",Aq,Bq,Cq);
  
  // 1. Check Aq = 0 (If Aq = 0 then t = -Cq / Bq
  // 2.If Aq � 0, then check the discriminant (If Bq2 - 4AqCq < 0 then there is no intersection)
  // 3. Compute t0 and if t0 > 0 then done else compute t1
  if (Aq == 0) return -Cq / Bq;
  
  // discriminant 
  double disc = (sqr(Bq) - (4 * Aq * Cq));
  if (VERBOSE) printf("DBG : disc=(%f), where Bq^2=(%f) and 4ac=(%f)\n",disc,sqr(Bq),(4*Aq*Cq));
  if (disc < 0) return -1; // no intersection in this case
  
  // since we are using it more than once
  disc = sqrt(disc) ;
  
  double t0 = (-Bq - disc) / (2 * Aq);
  if (VERBOSE) printf("DBG : t0=%f\n",t0);
  if (t0 > 0) rval = t0; // smaller/lesser of the 2 values, needs to come first
  goto CN;

  double t1 = (-Bq + disc) / (2 * Aq);
  if (VERBOSE) printf("DBG : t1=%f\n",t1);
  if (t1 > 0) rval = t1;

  // surface normal
 CN: {
    double t = rval;
    double Rx = Ro[0] + t*Rd[0] - C[0];
    double Ry = Ro[1] + t*Rd[1] - C[1];
    double Rz = Ro[2] + t*Rd[2] - C[2];
    Nq[0] = c.A*(Rx) + c.B*(Ry) + c.C*(Rz) + c.D;
    Nq[1] = c.B*(Rx) + c.E*(Ry) + c.F*(Rz) + c.G;
    Nq[2] = c.C*(Rx) + c.F*(Ry) + c.H*(Rz) + c.I;
    vNormalize(Nq);
  }

  return rval;
}

// Helper functions to find the first camera object and get it's specified width/height
double getCameraWidth() {
  double w = 0.0;
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    if (INPUT_FILE_DATA.js_objects[o].typecode == 0) {
      w = INPUT_FILE_DATA.js_objects[o].width;
      if (w > 0) {
	if (INFO) printf("Info: Found camera object width %f\n",w);
	return w;
      } else {
	message("Error","Unsupported camera width less than zero");
      }
    }
  }
  return w;
}
double getCameraHeight() {
  double h = 0.0;
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    if (INPUT_FILE_DATA.js_objects[o].typecode == 0) {
      h = INPUT_FILE_DATA.js_objects[o].height;
      if (h > 0) {
	if (INFO) printf("Info: Found camera object height %f\n",h);
	return h;
      } else {
	message("Error","Unsupported camera height less than zero");
      }
    }
  }
  return h;
}

// helper function for JSON error checking (like does a sphere have a width, etc...)
void checkJSON (JSON_object *object) {
  if (INFO) message("Info","Checking JSON for errors...");
  // variables
  
  // code body
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    switch(object[o].typecode) {
    case 0: // camera
      if (!object[o].width || !object[o].height)
	message("Error","Camera object must have width and height properties");
      //      if (object[o].radius || sizeof(object[o].normal) > 0 || sizeof(object[o].color) > 0)
      if (object[o].radius)
	if (INFO) message("Warning","Ignoring camera object properties in excess of width/height");
      break;
    case 1: // sphere
      if (!object[o].radius)      
	message("Error","Sphere object is missing radius or is zero!");
      if (!object[o].flags.has_position)      
	message("Error","Sphere object is missing position!");
      if (!object[o].flags.has_color && !(object[o].flags.has_diffuse_color && object[o].flags.has_specular_color))
	message("Error","Sphere object is missing color!");
      if (object[o].width || object[o].height || sizeof(object[o].normal) > 0)
	if (INFO) message("Warning","Ignoring sphere object properties in excess of radius/position/color");
      break;
    case 2: // plane
      if (!object[o].flags.has_position)      
	message("Error","Plane object is missing position!");
      if (!object[o].flags.has_color && !(object[o].flags.has_diffuse_color && object[o].flags.has_specular_color))
	message("Error","Plane object is missing color!");
      if (!object[o].flags.has_normal)      
	message("Error","Plane object is missing normal!");
      break;
    case 3: // cylinder
      break;
    case 4:
      if (!object[o].flags.has_position)      
	message("Error","Quadric object is missing position!");
      if (!object[o].flags.has_color && !(object[o].flags.has_diffuse_color && object[o].flags.has_specular_color))
	message("Error","Quadric object is missing color!");
      if (!object[o].flags.has_A)      
	message("Error","Quadric object is missing A parameter!");
      if (!object[o].flags.has_B)      
	message("Error","Quadric object is missing B parameter!");
      if (!object[o].flags.has_C)      
	message("Error","Quadric object is missing C parameter!");
      if (!object[o].flags.has_D)      
	message("Error","Quadric object is missing D parameter!");
      if (!object[o].flags.has_E)      
	message("Error","Quadric object is missing E parameter!");
      if (!object[o].flags.has_F)      
	message("Error","Quadric object is missing F parameter!");
      if (!object[o].flags.has_G)      
	message("Error","Quadric object is missing G parameter!");
      if (!object[o].flags.has_H)      
	message("Error","Quadric object is missing H parameter!");
      if (!object[o].flags.has_I)      
	message("Error","Quadric object is missing I parameter!");
      if (!object[o].flags.has_J)      
	message("Error","Quadric object is missing J parameter!");
      break;
      // TODO: may need to add other checks for lights in project 3
    case 5:
      if (!object[o].flags.has_position)      
	message("Error","Light object is missing position!");
      if (!object[o].flags.has_color)      
	message("Error","Light  object is missing color!");
      if (!object[o].flags.has_radial_a2)      
	message("Error","Light  object is missing radial-a2!");
      break;
    default:
      message("Error","Un-caught error, was missed during parsing");
    }
  }
  if (INFO) message("Info","Done checking JSON for errors...");
}

// Helper to get populate the array of lights in the scene, will make shading easier to have this array
void populateLightArray () {
  if (INFO) message("Info","Populating array containing light objects...");
  
  // variables
  int light_count = 0;

  // populate the global array
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    if (INPUT_FILE_DATA.js_objects[o].typecode == 5) {
      LIGHT_OBJECTS.light_objects[light_count].color[0] = INPUT_FILE_DATA.js_objects[o].color[0];
      LIGHT_OBJECTS.light_objects[light_count].color[1] = INPUT_FILE_DATA.js_objects[o].color[1];
      LIGHT_OBJECTS.light_objects[light_count].color[2] = INPUT_FILE_DATA.js_objects[o].color[2];
      LIGHT_OBJECTS.light_objects[light_count].position[0] = INPUT_FILE_DATA.js_objects[o].position[0];
      LIGHT_OBJECTS.light_objects[light_count].position[1] = INPUT_FILE_DATA.js_objects[o].position[1];
      LIGHT_OBJECTS.light_objects[light_count].position[2] = INPUT_FILE_DATA.js_objects[o].position[2];
      LIGHT_OBJECTS.light_objects[light_count].direction[0] = INPUT_FILE_DATA.js_objects[o].direction[0];
      LIGHT_OBJECTS.light_objects[light_count].direction[1] = INPUT_FILE_DATA.js_objects[o].direction[1];
      LIGHT_OBJECTS.light_objects[light_count].direction[2] = INPUT_FILE_DATA.js_objects[o].direction[2];
      LIGHT_OBJECTS.light_objects[light_count].radial_a0 = INPUT_FILE_DATA.js_objects[o].radial_a0;
      LIGHT_OBJECTS.light_objects[light_count].radial_a1 = INPUT_FILE_DATA.js_objects[o].radial_a1;
      LIGHT_OBJECTS.light_objects[light_count].radial_a2 = INPUT_FILE_DATA.js_objects[o].radial_a2;
      LIGHT_OBJECTS.light_objects[light_count].angular_a0 = INPUT_FILE_DATA.js_objects[o].angular_a0;
      LIGHT_OBJECTS.light_objects[light_count].theta = INPUT_FILE_DATA.js_objects[o].theta;
      LIGHT_OBJECTS.light_objects[light_count].flags.has_direction = INPUT_FILE_DATA.js_objects[o].flags.has_direction;
      LIGHT_OBJECTS.light_objects[light_count].flags.has_angular_a0 = INPUT_FILE_DATA.js_objects[o].flags.has_angular_a0;
      LIGHT_OBJECTS.light_objects[light_count].flags.has_theta = INPUT_FILE_DATA.js_objects[o].flags.has_theta;
      light_count++;
    }
  }
  LIGHT_OBJECTS.num_lights = light_count;
  if (INFO) printf("Info: Done, found %d light objects\n",LIGHT_OBJECTS.num_lights);
}

// helper to return the normal for the given object
// pass in the quadric normal (which was already computed for the given point)
// if the object is not a quadric, then simply ignore it
void getObjectNormal (int index, double* Ro, double* Qn, double* N) {
  // 0 = camera, 1 = sphere, 2 = plane, 3 = cylinder, 4 = quadric, 5 = light
  if (INPUT_FILE_DATA.js_objects[index].typecode == 2) {
    N[0] = INPUT_FILE_DATA.js_objects[index].normal[0];
    N[1] = INPUT_FILE_DATA.js_objects[index].normal[1];
    N[2] = INPUT_FILE_DATA.js_objects[index].normal[2];
  } else if (INPUT_FILE_DATA.js_objects[index].typecode == 1) {
    vSubtract(Ro,INPUT_FILE_DATA.js_objects[index].position,N);
  } else if (INPUT_FILE_DATA.js_objects[index].typecode == 4) {
    N[0] = Qn[0];
    N[1] = Qn[1];
    N[2] = Qn[2];
  } else {
    //TODO: error handling
  }
}

// Angular attentuation function
// params:
//        - index (this is the index of the light object on the global JSON objects array)
//                use the index as opposed to passing in all the parameter values for the light itself
//        - a1    (this is the "attenuation tweak" constant
//        - Vo    (this is the vector from the object to the light, reverse it to get Vo)
double fAng (int l_index, double* V) {
  double a1 = LIGHT_OBJECTS.light_objects[l_index].angular_a0;
  double Vl[3]; // this is the vector from the spot in it's direction
  double Vo[3]; // reverse of the V vector

  // create the vectors
  Vl[0] = LIGHT_OBJECTS.light_objects[l_index].direction[0];
  Vl[1] = LIGHT_OBJECTS.light_objects[l_index].direction[1];
  Vl[2] = LIGHT_OBJECTS.light_objects[l_index].direction[2];
  vScale(V,-1,Vo);

  // compute the angles
  double alpha = vDot(Vl,Vo);
  // not sure why but this conversion takes the spot away
  double theta = degreesToRadians(LIGHT_OBJECTS.light_objects[l_index].theta);
  //double theta = LIGHT_OBJECTS.light_objects[l_index].theta;

  // now compute and return the value
  if (
      // not a spotlight
      LIGHT_OBJECTS.light_objects[l_index].flags.has_angular_a0 != 1 ||
      LIGHT_OBJECTS.light_objects[l_index].flags.has_direction !=1 ||
      LIGHT_OBJECTS.light_objects[l_index].flags.has_theta != 1
      ) { 
    return 1.0;
  } else if (alpha < cos(theta)) {
    return 0;
  } else {
    return pow(alpha,a1); 
  }
}

// Radial attenuation function
// params:
//        - index  (the index of the light on the light objects array so we can pull params from there)
//        - dl     (this is the distance from the light to the object
double fRad (int l_index, double dl) {
  double a2 = LIGHT_OBJECTS.light_objects[l_index].radial_a2;
  double a1 = LIGHT_OBJECTS.light_objects[l_index].radial_a1;
  double a0 = LIGHT_OBJECTS.light_objects[l_index].radial_a0;
  if (dl == INFINITY) {
    return 1;
  } else {
    return (1 / ( a2*sqr(dl) + a1*dl + a0)) ;
  }
}

// Diffuse reflection funciton
// params:
//        - o_index   (of the object from global JSON objects)
//        - l_index   (of the light we are being illuminated from)
//        - c_index   (which color, R/G/B?)
//        - N       (normal from the point on the object we are currently shading)
//        - L       (vector to the light source)
// I think Il is diffuse_color from JSON
double Idiff (int o_index, int l_index, int c_index, double* N, double* L) {

  // variables
  int Ka = 0; // Ka and Ia are just placeholders until ambient light is implemented
  int Ia = 1;
  double Kd = 1;
  //  double Il = LIGHT_OBJECTS.light_objects[l_index].color[c_index];
  double Il;
  double rval; // helps with debug

  if (INPUT_FILE_DATA.js_objects[o_index].flags.has_diffuse_color) {
    //    Kd = INPUT_FILE_DATA.js_objects[o_index].diffuse_color[c_index];
    Il = INPUT_FILE_DATA.js_objects[o_index].diffuse_color[c_index];
  } else {
    //    Kd = INPUT_FILE_DATA.js_objects[o_index].color[c_index];
    Il = INPUT_FILE_DATA.js_objects[o_index].color[c_index];
  }

  // calculations
  double N_dot_L = vDot(N,L);

  if (N_dot_L > 0) {
    rval = Ka*Ia + Kd*Il*N_dot_L;
  } else {
    rval = Ka*Ia;
  }
  //if (DBG) printf("DBG Idiff(%f): o_i(%d), c_i(%d), Il(%f), NdL(%f), N[%f,%f,%f], L[%f,%f,%f]\n",rval,o_index,c_index,Il,N_dot_L,N[0],N[1],N[2],L[0],L[1],L[2]);
  return rval;
}

// Specular reflection funciton
// specular[0] = Ispec(k, 0, V, R, N, L, ns);
//
// Current status 10/18pm - Still cannot get VdotR to give anything but a reflection back toward the camera
//      Unit tested the reflection vector, I believe it's correct
//      All inputs are normalized and I believe they are all correct
///     Tried the halfway vector and it's a total mess - oh wait, the 2nd way seems ok with ns=50
double Ispec (int o_index, int l_index, int c_index, double* V, double* R, double* N, double* L, double ns) {
  // variables
  double Ks;
  double rval; // for DBG
  double Il = LIGHT_OBJECTS.light_objects[l_index].color[c_index];

  // R = (2N dot L)N - L (where L is light vector V is the view vector, and V = -1 * Rd (vector from intersection back to Ro)

  if (INPUT_FILE_DATA.js_objects[o_index].flags.has_specular_color) {
    Ks = INPUT_FILE_DATA.js_objects[o_index].specular_color[c_index];
  } else {
    Ks = INPUT_FILE_DATA.js_objects[o_index].color[c_index];
  }

  // calculations
  double V_dot_R = vDot(V,R);
  double N_dot_L = vDot(N,L);
  double N_dot_R = vDot(N,R);
  double V_dot_L = vDot(V,L);

  if (V_dot_R > 0 && N_dot_L > 0) {
    //rval = Ks*Il*pow(V_dot_R,ns); // points toward camera always
    //rval = Ks*Il*pow(N_dot_L,ns); // points toward light
    //rval = Ks*Il*pow(N_dot_R,ns); // consumes the sphere unless ns is like 2000, then it seems similar to N_dot_L
    //rval = Ks*Il*pow(V_dot_L,ns);  // takes the spec away entirely

    // try the halfway vector: (L + V)/|L + V|
    double L_plus_V[3];
    vAdd(L,V,L_plus_V);
    /* This gives crazy result
    double H[3];
    double L_plus_V_length[3] = { L_plus_V[0], L_plus_V[1], L_plus_V[2] };
    vNormalize(L_plus_V_length);
    H[0] = L_plus_V[0] / L_plus_V_length[0];
    H[1] = L_plus_V[1] / L_plus_V_length[1];
    H[2] = L_plus_V[2] / L_plus_V_length[2];
    rval = vDot(N,H);
    */
    // This seems sane
    vNormalize(L_plus_V);
    double H = vDot(N,L_plus_V);
    rval = Ks*Il*pow(H,ns);
  } else {
    rval = 0;
  }
  if (DBG) printf("DBG Ispec(%f): o_i(%d), c_i(%d), Il(%f), VdR(%f), NdL(%f), V[%f,%f,%f], R[%f,%f,%f], \nN[%f,%f,%f], L[%f,%f,%f]\n",rval,o_index,c_index,Il,V_dot_R,N_dot_L,V[0],V[1],V[2],R[0],R[1],R[2],N[0],N[1],N[2],L[0],L[1],L[2]);
  return rval;
}

/*
Issues:
-------
- spotlights seem to be superceding other lights
- quadric normal appears to be incorrect, diffuse color is on wrong side of cylinder, always same for ellipsoid
  regardless of where I move the light

- looks like multiple lights aren't quite summing correctly (03.json)
- planes are shadowing each other adn other objects falsely, (12,16.json)
- what to do about when no specular color is given (like project example JSON)

// quadric normal - easy way, think of object as a density, equation <= 0 , normal would be awy from more dense part
// or, form a triangle by shooting multiple rays and get the normal of the traingle

// triangluation is the way that things are done, even in production raytracers

// he "aliased" color to diffuse_color, so old JSON also works, we don't have to do that but we can
// KaIa - could have Ka be some ambience from the object, Ia some ambiance from the light
// he won't test us on lights on wrong side of plane or inside of sphere

    //  } else if (abs(alpha) > theta) {
    // bounds on theta, less than 180 degrees
    // assume theta is degrees
    // if (Vobj dot Vlight = cos alpha) < cos theta ( where Vobj = |Vintersection - light position| normalized
    // cos alpha = Vobj dot Vlight, assume theta in degree, need a degree to radian conversion
    // Vlight is given in JSON (normalize this too, but he'll give us normalized direction)
    // (Vobj dot Vlight) ^ a0

*/

void getReflectionVector (double* L, double* N, double* R, int DBG_flag) {

  // variables
  double S[3]; // scaled vector

  // Formula: R = reflection of L about N: R = L - 2(N dot L)N
  double L_dot_N = vDot(L,N);
  L_dot_N *= 2;
  vScale(N, L_dot_N, S);

  R[0] = S[0] - L[0];
  R[1] = S[1] - L[1];
  R[2] = S[2] - L[2];

  if (DBG_flag) printf("L = [%f,%f,%f]\n",L[0],L[1],L[2]);
  if (DBG_flag) printf("N = [%f,%f,%f]\n",N[0],N[1],N[2]);
  if (DBG_flag) printf("S = [%f,%f,%f]\n",S[0],S[1],S[2]);
  if (DBG_flag) printf("L_dot_N: %f\n",L_dot_N);
  if (DBG_flag) printf("R = [%f,%f,%f]\n",R[0],R[1],R[2]);

  /* put this in main
  double N[3] = {0,1,0};
  double L[3] = {-1,1,0};
  double R[3];
  getReflectionVector(L,N,R,1);
  */
}

/*
  function to get vector from refraction
  use the equations from the 08b-raytrace text
 */
void getRefractionVector (double* Ur, double* n, double Pt, double* Ut) {

  // variables
  double Pr = 1; // ior for air
  double S[3];   // scaled vector
  double a[3];
  double b[3];
  double cos_phi;
  double sin_phi;

  // Calculations
  // first : get 'a' vector: a = (n X Ur)/||n X Ur||  <-- compute the cross-product and then normalize it
  double n_cross_Ur[3];
  vCross(n,Ur,n_cross_Ur);
  vNormalize(n_cross_Ur);
  // second: get 'b' vector: b = a X n
  vCross(a,n,b);
  // third : get sin_phi = Pr/Pt Ur dot b
  double Ur_scaled[3];
  vScale(Ur,(Pr/Pt),Ur_scaled);
  sin_phi = vDot(Ur_scaled,b);
  // fourth: get cos_phi = sqrt(1 - sin_phi^s)
  cos_phi = sqrt(1 - sqr(sin_phi));
  // fifth : solve for Ut = -n(cos_phi) + b(sin_phi)
  double n_scaled[3];
  double b_scaled[3];
  vScale(n,-cos_phi,n_scaled);
  vScale(b,sin_phi,b_scaled);
  vAdd(n_scaled,b_scaled,Ut);
}

// https://www.siggraph.org/education/materials/HyperGraph/raytrace/rtillumi.htm
//
// I = Ilocal + Kr * R + Kt * T where R is the intensity of light from the reflected ray and T is the intensity
// of light from the transmitted ray. Kr and Kt are the reflection and transmission coefficients. For a very
// specular surface, such as plastic, we sometimes do not compute a local intensity, Ilocal, but only use the
// reflected/transmitted intensity values.
//
// Some people will skip Snell's law and do a lookup instead
//
// Kr = reflection constant
// Kt = refraction constant
// Ilocal = Local shading (from Project #3)
// IR = recursive call to shade the reflection vector
// IT = recursive call to shade the refraction vector
// I = (1 - Kr - Kt) * Ilocal + Kr * IR + Kt + IT   < his adjusted equation
//
// He talked about 3 nested spheres, pushing ior onto stack, pushing/popping (but easy way is just to pass in the
// ior to the recursive function, this has the effect of making a stack.
// Talked about, for a sphere, have to look at t values to understand if you are inside it or not (1+,1- t)
// concentric circles are tricky
// have to know if you are entering or exiting an object - but he said he won't nest spheres, will have plane
// with spheres but they won't intersect. Not complicated nestings, intersections, etc...

