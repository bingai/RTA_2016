// STD includes
#include <iostream>

// Project includes
#include "openni_tools/capture/openni_capture.h"

// Utilities
#include <namaris/utilities/filesystem.hpp>

////////////////////////////////////////////////////////////////////////////////
void printHelp (char** argv)
{
  std::cout << "Capture single frames or sequences from an OpenNI device." << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << utl::fs::getBasename(std::string(argv[0])) << " <output directory> [parameters]" << std::endl;
  std::cout << "If no valid directory is provided runs in view-only mode." << std::endl;
  std::cout << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "    -h            print this message"                                   << std::endl;
  std::cout << "    -v            verbose mode"                                         << std::endl;
  std::cout << "    -p            filename prefix"                                      << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char** argv, std::string &dirname, std::string &filename_prefix, bool &print_help, bool &verbose)
{  
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
    
    if (curParameter == "-h" || curParameter == "-help")
      print_help = true;
    else if (curParameter == "-v")
      verbose = true;
    else if (curParameter == "-p")
    {
      if (i < argc+1 )
        filename_prefix = argv[++i];
      else
        filename_prefix = "";
    }
    else if (curParameter[0] != '-')
      dirname = curParameter;
    else 
      std::cout << "Unknown parameter '" << curParameter << "'" << std::endl;
  }
}

int main( int argc, char** argv )
{
  //----------------------------------------------------------------------------
  // Parse command line arguments
  //----------------------------------------------------------------------------
  
  std::string captureDirectory = "";
  std::string captureFilenamePrefix = "";
  bool viewOnly = false;
  bool verbose = false;
  bool print_help = false;
  int startFrameIndex = -1;
  int startOniCaptureIndex = -1;
  
  parseCommandLine(argc, argv, captureDirectory, captureFilenamePrefix,print_help, verbose);
  
  if (captureFilenamePrefix == "")
    captureFilenamePrefix = "capture";
  
  if (print_help)
  {
    printHelp(argv);
    return 0;
  }
  
  //----------------------------------------------------------------------------
  // Check command line arguments
  //----------------------------------------------------------------------------
  
  if (!utl::fs::isDirectory(captureDirectory))
  {
    viewOnly = true;    
    std::cout << "Capture directory provided does not exist or is not a directory." << std::endl;
    std::cout << "Running in view-only mode." << std::endl;
  }
    
  //----------------------------------------------------------------------------
  // Run data capture
  //----------------------------------------------------------------------------
  
  OpenNICapture dataCapture(OpenNICapture::DEPTH_RGB, captureDirectory, captureFilenamePrefix, viewOnly, verbose);
  dataCapture.run();
  return 0;
}