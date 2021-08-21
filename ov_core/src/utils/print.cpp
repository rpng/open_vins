/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "print.h"
#include <cstdarg>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace ov_core;

// Need to define the variable for everything to work
Printer::PrintLevel Printer::current_print_level = PrintLevel::ALL;

void Printer::setPrintLevel(PrintLevel level) {
  Printer::current_print_level = level;

  std::cout << "Setting printing level to: ";
  switch (current_print_level) {
  case PrintLevel::ALL:
    std::cout << "ALL";
    break;
  case PrintLevel::DEBUG:
    std::cout << "DEBUG";
    break;
  case PrintLevel::INFO:
    std::cout << "INFO";
    break;
  case PrintLevel::WARNING:
    std::cout << "WARNING";
    break;
  case PrintLevel::ERROR:
    std::cout << "ERROR";
    break;
  case PrintLevel::SILENT:
    std::cout << "SILENT";
    break;
  default:
    // Can never get here
    break;
  }

  std::cout << std::endl;
}

void Printer::debugPrint(PrintLevel level, const char location[], const char *format, ...) {
  // Only print for the current debug level
  if (static_cast<int>(level) < static_cast<int>(Printer::current_print_level)) {
    return;
  }

  // Print the location info first
  if (strlen(location) > MAX_FILE_PATH_LEGTH) {
    // Truncate the location length to the max size for the filepath
    printf("%s", &(location[strlen(location) - MAX_FILE_PATH_LEGTH]));
  } else {
    // Print the full location
    printf("%s", location);
  }

  // Print the rest of the args
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);

  // All prints get a new line!
  printf("\r\n");
}
