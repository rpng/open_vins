/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#ifndef OV_CORE_PRINT_H
#define OV_CORE_PRINT_H

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

namespace ov_core {

/**
 * @brief Printer for open_vins that allows for various levels of printing to be done
 *
 * To set the global verbosity level one can do the following:
 * @code{.cpp}
 * ov_core::Printer::setPrintLevel("WARNING");
 * ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::WARNING);
 * @endcode
 */
class Printer {
public:
  /**
   * @brief The different print levels possible
   *
   * - PrintLevel::ALL : All PRINT_XXXX will output to the console
   * - PrintLevel::DEBUG : "DEBUG", "INFO", "WARNING" and "ERROR" will be printed. "ALL" will be silenced
   * - PrintLevel::INFO : "INFO", "WARNING" and "ERROR" will be printed. "ALL" and "DEBUG" will be silenced
   * - PrintLevel::WARNING : "WARNING" and "ERROR" will be printed. "ALL", "DEBUG" and "INFO" will be silenced
   * - PrintLevel::ERROR : Only "ERROR" will be printed. All the rest are silenced
   * - PrintLevel::SILENT : All PRINT_XXXX will be silenced.
   */
  enum PrintLevel { ALL = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, SILENT = 5 };

  /**
   * @brief Set the print level to use for all future printing to stdout.
   * @param level The debug level to use
   */
  static void setPrintLevel(const std::string &level);

  /**
   * @brief Set the print level to use for all future printing to stdout.
   * @param level The debug level to use
   */
  static void setPrintLevel(PrintLevel level);

  /**
   * @brief The print function that prints to stdout.
   * @param level the print level for this print call
   * @param location the location the print was made from
   * @param line the line the print was made from
   * @param format The printf format
   */
  static void debugPrint(PrintLevel level, const char location[], const char line[], const char *format, ...);

  /// The current print level
  static PrintLevel current_print_level;

private:
  /// The max length for the file path.  This is to avoid very long file paths from
  static constexpr uint32_t MAX_FILE_PATH_LEGTH = 30;
};

} /* namespace ov_core */

/*
 * Converts anything to a string
 */
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/*
 * The different Types of print levels
 */
#define PRINT_ALL(x...) ov_core::Printer::debugPrint(ov_core::Printer::PrintLevel::ALL, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_DEBUG(x...) ov_core::Printer::debugPrint(ov_core::Printer::PrintLevel::DEBUG, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_INFO(x...) ov_core::Printer::debugPrint(ov_core::Printer::PrintLevel::INFO, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_WARNING(x...) ov_core::Printer::debugPrint(ov_core::Printer::PrintLevel::WARNING, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_ERROR(x...) ov_core::Printer::debugPrint(ov_core::Printer::PrintLevel::ERROR, __FILE__, TOSTRING(__LINE__), x);

#endif /* OV_CORE_PRINT_H */
