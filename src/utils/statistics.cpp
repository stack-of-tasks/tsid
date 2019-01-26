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

#include <iomanip>      // std::setprecision
#include "tsid/utils/statistics.hpp"

using std::map;
using std::string;
using std::ostringstream;


Statistics& getStatistics()
{
  static Statistics s;
  return s;
}

Statistics::Statistics()
  : active(true)
{
  records_of = new map<string, QuantityData>();
}

Statistics::~Statistics()
{
  delete records_of;
}

bool Statistics::quantity_exists(string name)
{
  return (records_of->find(name) != records_of->end());
}

void Statistics::store(string name, const double & value)
{
  if (!active) return;

  // Just works if not already present
  records_of->insert(make_pair(name, QuantityData()));

  QuantityData& quant_info = records_of->find(name)->second;

  quant_info.stops++;

  // Update last value
  quant_info.last = value;

  // Update min/max
  if ( value >= quant_info.max )
    quant_info.max = value;
  if ( value <= quant_info.min || quant_info.min == 0 )
    quant_info.min = value;

  // Update total
  quant_info.total += value;
}

void Statistics::reset_all()
{
  if (!active) return;

  map<string, QuantityData>::iterator it;

  for (it = records_of->begin(); it != records_of->end(); ++it) {
    reset(it->first);
  }
}

void Statistics::report_all(int precision, std::ostream& output)
{
  if (!active) return;

  output<< "\n*** STATISTICS (min - avg - max - last - nSamples - total) ***\n";
  map<string, QuantityData>::iterator it;
  for (it = records_of->begin(); it != records_of->end(); ++it) {
    if(it->second.stops>0)
      report(it->first, precision, output);
  }
}

void Statistics::reset(string name)
{
  if (!active) return;

  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  quant_info.total = 0;
  quant_info.min = 0;
  quant_info.max = 0;
  quant_info.last = 0;
  quant_info.stops = 0;
}

void Statistics::turn_on()
{
  std::cout << "Statistics active." << std::endl;
  active = true;
}

void Statistics::turn_off()
{
  std::cout << "Statistics inactive." << std::endl;
  active = false;
}

void Statistics::report(string name, int precision, std::ostream& output)
{
  if (!active) return;

  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  string pad = "";
  for (std::string::size_type i = name.length(); i<STATISTICS_MAX_NAME_LENGTH; i++)
    pad.append(" ");

  output << name << pad;
  output << std::fixed << std::setprecision(precision)
         << (quant_info.min) << "\t";
  output << std::fixed << std::setprecision(precision)
         << (quant_info.total / (long double) quant_info.stops) << "\t";
  output << std::fixed << std::setprecision(precision)
         << (quant_info.max) << "\t";
  output << std::fixed << std::setprecision(precision)
         << (quant_info.last) << "\t";
  output << std::fixed << std::setprecision(precision)
         << quant_info.stops << "\t";
  output << std::fixed << std::setprecision(precision)
         << quant_info.total << std::endl;
}

long double Statistics::get_total(string name)
{
  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  return quant_info.total;

}

long double Statistics::get_average(string name)
{
  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  return (quant_info.total / (long double)quant_info.stops);

}

long double Statistics::get_min(string name)
{
  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  return quant_info.min;

}

long double Statistics::get_max(string name)
{
  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  return quant_info.max;

}

long double Statistics::get_last(string name)
{
  // Try to recover Quantity data
  if ( !quantity_exists(name)  )
    throw StatisticsException("Quantity not initialized.");

  QuantityData& quant_info = records_of->find(name)->second;

  return quant_info.last;
}
