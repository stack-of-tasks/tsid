//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//


#ifndef __invdyn_statistics_H__
#define __invdyn_statistics_H__

#include <iostream>
#include <map>
#include <sstream>

#define STATISTICS_MAX_NAME_LENGTH 60

// Generic statistics exception class
struct StatisticsException
{
public:
  StatisticsException(std::string error) : error(error) { }
  std::string error;
};


/**
    @brief A class to compute statistics about quantities of interest.

    @code
    Statistics stat();
    @endcode

    The Statistics class can be used to keep track of the minimum,
    maximum, average of quantities of interest.

    To report the results just call:

    @code
    stat.report("Code ID");
    @endcode

    Thou can also provide an additional std::ostream& parameter to report() to
    redirect the logging on a different output. Also, you can use the
    get_total/min/max/average() methods to get the individual numeric data,
    without all the details of the logging. You can also extend Statistics to
    implement your own logging syntax.

    To report all the measurements:

    @code
    stat.report_all();
    @endcode

    Same as above, you can redirect the output by providing a std::ostream&
    parameter.

*/
class Statistics {
public:

  /** Constructor */
  Statistics();

  /** Destructor */
  ~Statistics();

  /** Tells if a quantity with a certain name exists */
  bool quantity_exists(std::string name);

  /** Record the value of the specified quantity */
  void store(std::string name, const double & value);

  /** Reset a certain quantity record */
  void reset(std::string name);

  /** Resets all the quantity records */
  void reset_all();

  /** Dump the data of a certain quantity record */
  void report(std::string name, int precision=2,
              std::ostream& output = std::cout);

  /** Dump the data of all the quantity records */
  void report_all(int precision=2, std::ostream& output = std::cout);

  /** Returns total execution of a certain quantity */
  long double get_total(std::string name);

  /** Returns average execution of a certain quantity */
  long double get_average(std::string name);

  /** Returns minimum execution of a certain quantity */
  long double get_min(std::string name);

  /** Returns maximum execution of a certain quantity */
  long double get_max(std::string name);

  /** Return last measurement of a certain quantity */
  long double get_last(std::string name);

  /**	Turn off statistics, all the Statistics::* methods return without doing
        anything after this method is called. */
  void turn_off();

  /** Turn on statistics, restore operativity after a turn_off(). */
  void turn_on();

protected:

  /** Struct to hold the quantity data */
  struct QuantityData {

    QuantityData() :
      total(0),
      min(0),
      max(0),
      last(0),
      stops(0) {
    }

    /** Cumulative total value */
    long double	total;

    /** Minimum value */
    long double	min;

    /** Maximum value */
    long double	max;

    /** Last value */
    long double last;

    /** How many times have this quantity been stored? */
    int	stops;
  };

  /** Flag to hold the statistics status */
  bool active;

  /** Pointer to the dynamic structure which holds the collection of quantity
      data */
  std::map<std::string, QuantityData >* records_of;

};

Statistics& getStatistics();

#endif
