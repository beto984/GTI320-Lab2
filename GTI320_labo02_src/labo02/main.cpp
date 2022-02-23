/**
 * @file main.cpp  
 *
 * @brief GTI320 Labo 2 - Lance une fenêtre NanoGUI
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/formhelper.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/glcanvas.h>
#include <iostream>
#include <fstream>
#include <string>

#include "IcpApplication.h"

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>
#include <ctime>

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

void usage(const char* cmd)
{
  std::cout << "Usage : " << cmd << " [options] [OBJFOLDER]\n\n";
  std::cout << "  Loads .obj files from folder OBJFOLDER. Default is the current working directory.\n\n";
  std::cout << "  Options:\n";
  std::cout << "    -h, --help   Display this help.\n" << std::endl;
}

int main(int argc, char** argv) 
{
  const char* objFolder = "";
  for (int i=1; i<argc; ++i) 
    {
      if (std::string("-h").compare(argv[i]) == 0) 
        {
          usage(argv[0]);
          exit(0);
        }
      else if (std::string("--help").compare(argv[i]) == 0) 
        {
          usage(argv[0]);
          exit(0);
        }
      else
        {
          objFolder = argv[1];
        }
    }
  std::srand(std::time(0));

  nanogui::init();
  nanogui::ref<IcpApplication> app = new IcpApplication(objFolder);
  app->drawAll();
  app->setVisible(true);
  nanogui::mainloop();
  nanogui::shutdown();

  return 0;
}
