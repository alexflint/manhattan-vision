#include <boost/gil/typedefs.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <iostream>
#include <sstream>

using namespace boost::gil;

int main(int args, char** argv)
{
	assert(args == 2);
	char* path = argv[1];

	rgb8_image_t img;
	png_read_image(path,img);

	std::stringstream red,green,blue;
	for (int x = 0; x < view(img).width(); x++)
		for (int y = 0; y < view(img).height(); y++)
			{
				red << (int)boost::gil::get_color(view(img)(x,y),boost::gil::red_t()) << ",";
				blue << (int)boost::gil::get_color(view(img)(x,y),boost::gil::blue_t()) << ",";
				green << (int)boost::gil::get_color(view(img)(x,y),boost::gil::green_t()) << ",";
			}


	std::cout << "int width = " << view(img).width() << ";\n";
	std::cout << "int height = " << view(img).height() << ";\n";

	gray8_image_t mono(img.dimensions());
	copy_pixels(view(img), view(mono));
	png_write_view("mono.png", view(mono));

	/*std::cout << "unsigned char buffer[][" << "width*height" << "]={\n";
		std::cout << "{" << red.str() << "}\n" << "{" << green.str() << "}\n" << "{" << blue.str() << "}\n};\n\n";
		std::cout << "const boost::gil::rgb8c_planar_view_t view = \n";
		std::cout << "\tboost::gil::planar_rgb_view(width,\n";
		std::cout << "\theight,buffer[0],buffer[1],buffer[2],width);\n";*/

	return 0;
}
