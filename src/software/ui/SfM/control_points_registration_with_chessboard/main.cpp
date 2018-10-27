// Copyright (c) 2018 William GELARD.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "AutomaticRegistration.hpp"
#include "third_party/cmdLine/cmdLine.h"


int main(int argc, char ** argv)
{

  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sSettingsFile;

  // required
  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  // Optional
  cmd.add( make_option('s', sSettingsFile, "settings_file") );

  try {
      if (argc < 2 || argc > 3) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file] a SfM_Data file \n"
      << "\n[Optional]\n"
      << "[-s|--settings_file] a specific settings_file for changing asymmetruc circles grid parameters\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--input_file " << sSfM_Data_Filename << std::endl;
  if(argc > 2)
            std::cout << "--settings_file " << sSettingsFile << std::endl;

  control_point_with_chessboard::AutomaticRegistration ar(sSfM_Data_Filename, sSettingsFile);
  ar.openProject();
  return 0;
}
