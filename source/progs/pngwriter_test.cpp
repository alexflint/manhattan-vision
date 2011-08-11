#include <pngwriter.h>

#include <stdlib.h>
#include <string>

#include <iostream>
#include <fstream>
using namespace std;

int main(int argc, char *argv[])
{

	   string filename(argv[1]);
	//filename = (*(argv+1));
	if(argc!=2)
	{

	return 0;
	}
	
	pngwriter image(1, 1, 0, "out.png");

    image.readfromfile(argv[1]);
   
   // Rename the file, add -mod to the basename.
   char * newfilename;

   char fl[1024];
   strcpy(fl, filename.c_str());

   newfilename = strtok (fl, "." );
   strcat(newfilename, "-mod.png" ); 
   image.pngwriter_rename(newfilename);

   std::cout << "File Name: " << argv[1] << ",  Color Type: " << image.getcolortype() << ", Bit Depth: " << image.getbitdepth() << std::endl;

	 std::cout << "Saving as " << newfilename;
   
   image.close();
	return 0;   
}
